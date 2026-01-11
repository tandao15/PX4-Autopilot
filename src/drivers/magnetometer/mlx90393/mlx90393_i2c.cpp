#include <px4_platform_common/px4_config.h>
#include <drivers/device/i2c.h>
#include "mlx90393.h"

class MLX90393_I2C : public device::I2C
{
public:
	MLX90393_I2C(const I2CSPIDriverConfig &config);
	virtual ~MLX90393_I2C() = default;

	virtual int     read(unsigned address, void *data, unsigned count);
	virtual int     write(unsigned address, void *data, unsigned count);

protected:
	virtual int     probe();

};

device::Device *MLX90393_I2C_interface(const I2CSPIDriverConfig &config)
{
	return new MLX90393_I2C(config);
}

MLX90393_I2C::MLX90393_I2C(const I2CSPIDriverConfig &config) :
	I2C(config)
{
}

int MLX90393_I2C::probe()
{
	uint8_t cmd = MLX90393_EX; // Exit mode command
	if (transfer(&cmd, 1, nullptr, 0) != OK) {
		return -EIO;
	}

	_retries = 1;
	return OK;
}

int MLX90393_I2C::read(unsigned address, void *data, unsigned count)
{
	uint8_t cmd = address;
	return transfer(&cmd, 1, (uint8_t *)data, count);
}

int MLX90393_I2C::write(unsigned address, void *data, unsigned count)
{
	uint8_t buf[32];

	if (sizeof(buf) < (count + 1)) {
		return -EIO;
	}

	buf[0] = address;
	memcpy(&buf[1], data, count);

	return transfer(&buf[0], count + 1, nullptr, 0);
}
