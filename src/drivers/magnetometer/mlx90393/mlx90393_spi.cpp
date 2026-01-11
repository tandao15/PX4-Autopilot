#include <px4_platform_common/px4_config.h>
#include <drivers/device/spi.h>
#include "mlx90393.h"

class MLX90393_SPI : public device::SPI
{
public:
	MLX90393_SPI(const I2CSPIDriverConfig &config);
	virtual ~MLX90393_SPI() = default;

	virtual int     init();
	virtual int     read(unsigned address, void *data, unsigned count);
	virtual int     write(unsigned address, void *data, unsigned count);
};

device::Device *MLX90393_SPI_interface(const I2CSPIDriverConfig &config)
{
	return new MLX90393_SPI(config);
}

MLX90393_SPI::MLX90393_SPI(const I2CSPIDriverConfig &config) :
	SPI(config)
{
}

int MLX90393_SPI::init()
{
	int ret = SPI::init();

	if (ret != OK) {
		DEVICE_DEBUG("SPI init failed");
		return -EIO;
	}

	return OK;
}

int MLX90393_SPI::read(unsigned address, void *data, unsigned count)
{
	uint8_t buf[32];

	if (sizeof(buf) < (count + 1)) {
		return -EIO;
	}

	buf[0] = address;
	memset(&buf[1], 0, count);

	int ret = transfer(&buf[0], &buf[0], count + 1);
	memcpy(data, &buf[1], count);
	return ret;
}

int MLX90393_SPI::write(unsigned address, void *data, unsigned count)
{
	uint8_t buf[32];

	if (sizeof(buf) < (count + 1)) {
		return -EIO;
	}

	buf[0] = address;
	memcpy(&buf[1], data, count);

	return transfer(&buf[0], nullptr, count + 1);
}
