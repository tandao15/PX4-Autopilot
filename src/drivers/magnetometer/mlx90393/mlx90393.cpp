#include "mlx90393.h"
#include <px4_platform_common/time.h>
#include <unistd.h>

MLX90393::MLX90393(device::Device *interface, const I2CSPIDriverConfig &config) :
	I2CSPIDriver(config),
	_px4_mag(interface->get_device_id(), config.rotation),
	_interface(interface)
{
}

MLX90393::~MLX90393()
{
}

int MLX90393::init()
{
	reset();

	px4_usleep(10000);

	start();

	return PX4_OK;
}

int MLX90393::reset()
{
	return send_command(MLX90393_RT);
}

int MLX90393::probe()
{
	return PX4_OK;
}

void MLX90393::start()
{
	// Exit any mode
	send_command(MLX90393_EX);
	px4_usleep(1000);

	// Start Burst Mode (status + T + X + Y + Z)
	uint8_t cmd = MLX90393_SB | MLX90393_ZYXT_BM;
	send_command(cmd);

	ScheduleOnInterval(MLX90393_CONVERSION_INTERVAL);
}

void MLX90393::RunImpl()
{
	collect();
}

int MLX90393::collect()
{
	perf_begin(_sample_perf);

	// 发送读取命令 (Read Measurement)
	// 注意：ZYXT 顺序很重要
	uint8_t cmd = MLX90393_RM | MLX90393_ZYXT_BM;

	// Buffer to store: Status (1) + T(2) + X(2) + Y(2) + Z(2) = 9 bytes
	uint8_t report[9];

	// I2C 读取
	if (_interface->read(cmd, &report, sizeof(report)) != OK) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return PX4_ERROR;
	}

	// 1. 处理大小端 解析数据
	uint16_t t_raw = (report[1] << 8) | report[2];
	int16_t  x_raw = (int16_t)((report[3] << 8) | report[4]);
	int16_t  y_raw = (int16_t)((report[5] << 8) | report[6]);
	int16_t  z_raw = (int16_t)((report[7] << 8) | report[8]);

	// 2. 只有当 status byte 正常时才处理 (可选，MLX 的 status 比较复杂，先忽略)

	// 3. 单位转换：Raw -> Gauss
	// 假设 Gain = 0, Res = 0 (默认值)
	// X/Y Sensitivity ~ 0.161 uT/LSB
	// Z   Sensitivity ~ 0.294 uT/LSB
	// 100 uT = 1 Gauss
	const float sensitivity_xy = 0.161f;
	const float sensitivity_z  = 0.294f;
	const float ut_to_gauss    = 1.0f / 100.0f;

	float x_gauss = (float)x_raw * sensitivity_xy * ut_to_gauss;
	float y_gauss = (float)y_raw * sensitivity_xy * ut_to_gauss;
	float z_gauss = (float)z_raw * sensitivity_z  * ut_to_gauss;

	// 4. 温度转换
	// T(°C) = (ADC - 46244) / 45.2 + 25
	// Note: t_raw must be treated as unsigned.
	float temp_c = ((float)((uint16_t)t_raw) - 46244.0f) / 45.2f + 25.0f;

	// 5. 更新数据给 PX4
	_px4_mag.set_temperature(temp_c);
	_px4_mag.update(hrt_absolute_time(), x_gauss, y_gauss, z_gauss);

	perf_end(_sample_perf);
	return PX4_OK;
}

int MLX90393::send_command(uint8_t cmd)
{
	return _interface->write(cmd, nullptr, 0);
}

int MLX90393::write_register(uint8_t reg, uint16_t val)
{
	uint8_t buf[3];
	buf[0] = (val >> 8) & 0xFF; // High byte
	buf[1] = val & 0xFF;        // Low byte
	buf[2] = (reg & 0x3F) << 2; // Address shifted

	return _interface->write(MLX90393_WR, buf, 3);
}

int MLX90393::read_register(uint8_t reg, uint16_t &val)
{
	return PX4_ERROR; // Not implemented yet
}

int MLX90393::measure()
{
	return PX4_OK;
}

void MLX90393::print_status()
{
	I2CSPIDriverBase::print_status();
}
