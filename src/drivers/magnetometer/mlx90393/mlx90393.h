#pragma once

#include <drivers/device/i2c.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>

/* MLX90393 Command Definitions */
#define MLX90393_SB               0x10          /* SB = Start Burst Mode */
#define MLX90393_SW               0x20          /* SW = Start Wake-up on Change Mode */
#define MLX90393_SM               0x30          /* SB = Start Single Measurement Mode */
#define MLX90393_RM               0x40          /* RM = Read Measurement */
#define MLX90393_RR               0x50          /* RR = Read Register */
#define MLX90393_WR               0x60          /* WR = Write Register */
#define MLX90393_EX               0x80          /* EX = Exit Mode */
#define MLX90393_HR               0xD0          /* HR = Memory Recall */
#define MLX90393_HS               0xE0          /* HS = Memory Store */
#define MLX90393_RT               0xF0          /* RT = Reset */

/* MLX90393 Sensor Selection Bit Definitions */
#define MLX90393_T_BM             (1<<0)          /* Temperature Sensor Bitmask  */
#define MLX90393_X_BM             (1<<1)          /* Magnetometer X-Axis Bitmask */
#define MLX90393_Y_BM             (1<<2)          /* Magnetometer Y-Axis Bitmask */
#define MLX90393_Z_BM             (1<<3)          /* Magnetometer Z-Axis Bitmask */
#define MLX90393_ZYXT_BM          (MLX90393_Z_BM | MLX90393_Y_BM | MLX90393_X_BM | MLX90393_T_BM)

/* I2C Address */
#define MLX90393_I2C_ADDR         0x0C

/* Max measurement rate */
#define MLX90393_CONVERSION_INTERVAL     (1000000 / 100)

/* interface factories */
extern device::Device *MLX90393_SPI_interface(const I2CSPIDriverConfig &config);
extern device::Device *MLX90393_I2C_interface(const I2CSPIDriverConfig &config);

class MLX90393 : public I2CSPIDriver<MLX90393>
{
public:
	MLX90393(device::Device *interface, const I2CSPIDriverConfig &config);
	virtual ~MLX90393();

	static I2CSPIDriverBase *instantiate(const I2CSPIDriverConfig &config, int runtime_instance);
	static void print_usage();

	virtual int init();
	void print_status() override;
	void RunImpl();

private:
	int probe();

	int send_command(uint8_t cmd);
	int write_register(uint8_t reg, uint16_t val);
	int read_register(uint8_t reg, uint16_t &val);
	int reset();
	int collect();
	int measure();
	void start();

	PX4Magnetometer _px4_mag;
	device::Device *_interface;

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": comms_errors")};
	perf_counter_t _conf_errors{perf_alloc(PC_COUNT, MODULE_NAME": conf_errors")};
	perf_counter_t _range_errors{perf_alloc(PC_COUNT, MODULE_NAME": range_errors")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
};
