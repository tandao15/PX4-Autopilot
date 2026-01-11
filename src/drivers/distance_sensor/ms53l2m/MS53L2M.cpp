/****************************************************************************
 *
 *   Copyright (c) 2018-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "MS53L2M.hpp"

#include <fcntl.h>

#include <lib/drivers/device/Device.hpp>

MS53L2M::MS53L2M(const char *port, uint8_t rotation) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
	_px4_rangefinder(0, rotation)
{
	// Store the port name.
	strncpy(_port, port, sizeof(_port) - 1);

	// Enforce null termination.
	_port[sizeof(_port) - 1] = '\0';

	device::Device::DeviceId device_id;
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;

	uint8_t bus_num = atoi(&_port[strlen(_port) - 1]); // Assuming '/dev/ttySx'

	if (bus_num < 10) {
		device_id.devid_s.bus = bus_num;
	}

	_px4_rangefinder.set_device_id(device_id.devid);

	// Use conservative distance bounds, to make sure we don't fuse garbage data
	_px4_rangefinder.set_min_distance(0.04f);	// Datasheet: 0.04m
	_px4_rangefinder.set_max_distance(7.9f);	// Datasheet: 8.0m
	_px4_rangefinder.set_fov(0.0488692f);
	_px4_rangefinder.set_device_type(DRV_DIST_DEVTYPE_MS53L2M);
	_px4_rangefinder.set_rangefinder_type(distance_sensor_s::MAV_DISTANCE_SENSOR_INFRARED);
}

MS53L2M::~MS53L2M()
{
	// Ensure we are truly inactive.
	stop();

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int
MS53L2M::collect()
{
	perf_begin(_sample_perf);

	uint8_t c;

	// Read from the sensor UART buffer one byte at a time
	while (::read(_file_descriptor, &c, 1) > 0) {
		if (_linebuf_index < sizeof(_linebuf) - 1) {
			_linebuf[_linebuf_index++] = c;

		} else {
			// Buffer overflow, reset
			_linebuf_index = 0;
		}

		// Check for line termination
		if (c == '\n') {
			_linebuf[_linebuf_index] = '\0'; // Null terminate

			// Parse the line
			// Format examples: "92\r\n" or "94,19.25\r\n"
			// We just want the first number.
			char *endptr = nullptr;
			long distance_mm = strtol(_linebuf, &endptr, 10);

			// Check if conversion was successful (at least one digit read)
			if (endptr != _linebuf && distance_mm >= 0) {
				float distance_m = static_cast<float>(distance_mm) / 1000.0f;
				_px4_rangefinder.update(hrt_absolute_time(), distance_m);
			}

			_linebuf_index = 0; // Reset for next line
		}
	}

	perf_end(_sample_perf);

	return PX4_OK;
}



int
MS53L2M::init()
{
	start();

	return PX4_OK;
}

int
MS53L2M::open_serial_port(const speed_t speed)
{
	// File descriptor initialized?
	if (_file_descriptor > 0) {
		// PX4_INFO("serial port already open");
		return PX4_OK;
	}

	// Configure port flags for read/write, non-controlling, non-blocking.
	int flags = (O_RDWR | O_NOCTTY | O_NONBLOCK);

	// Open the serial port.
	_file_descriptor = ::open(_port, flags);

	if (_file_descriptor < 0) {
		PX4_ERR("open failed (%i)", errno);
		return PX4_ERROR;
	}

	termios uart_config = {};

	// Store the current port configuration. attributes.
	tcgetattr(_file_descriptor, &uart_config);

	// Clear ONLCR flag (which appends a CR for every LF).
	uart_config.c_oflag &= ~ONLCR;

	// No parity, one stop bit.
	uart_config.c_cflag &= ~(CSTOPB | PARENB);

	// Set the input baud rate in the uart_config struct.
	int termios_state = cfsetispeed(&uart_config, speed);

	if (termios_state < 0) {
		PX4_ERR("CFG: %d ISPD", termios_state);
		::close(_file_descriptor);
		return PX4_ERROR;
	}

	// Set the output baud rate in the uart_config struct.
	termios_state = cfsetospeed(&uart_config, speed);

	if (termios_state < 0) {
		PX4_ERR("CFG: %d OSPD", termios_state);
		::close(_file_descriptor);
		return PX4_ERROR;
	}

	// Apply the modified port attributes.
	termios_state = tcsetattr(_file_descriptor, TCSANOW, &uart_config);

	if (termios_state < 0) {
		PX4_ERR("baud %d ATTR", termios_state);
		::close(_file_descriptor);
		return PX4_ERROR;
	}

	PX4_INFO("successfully opened UART port %s", _port);
	return PX4_OK;
}

void
MS53L2M::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}

void
MS53L2M::Run()
{
	// Ensure the serial port is open.
	open_serial_port();

	// Perform collection.
	collect();
}

void
MS53L2M::start()
{
	// Schedule the driver at regular intervals.
	ScheduleOnInterval(MS53L2M_MEASURE_INTERVAL);

	PX4_INFO("driver started");
}

void
MS53L2M::stop()
{
	// Clear the work queue schedule.
	ScheduleClear();

	// Ensure the serial port is closed.
	::close(_file_descriptor);
}
