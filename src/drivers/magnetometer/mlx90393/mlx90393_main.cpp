#include "mlx90393.h"
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <drivers/drv_sensor.h>

I2CSPIDriverBase *MLX90393::instantiate(const I2CSPIDriverConfig &config, int runtime_instance)
{
	device::Device *interface = nullptr;

	if (config.bus_type == BOARD_I2C_BUS) {
		interface = MLX90393_I2C_interface(config);

	} else if (config.bus_type == BOARD_SPI_BUS) {
		interface = MLX90393_SPI_interface(config);
	}

	if (interface == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	if (interface->init() != OK) {
		delete interface;
		PX4_DEBUG("no device on bus %i (devid 0x%x)", config.bus, config.spi_devid);
		return nullptr;
	}

	MLX90393 *dev = new MLX90393(interface, config);

	if (dev == nullptr) {
		delete interface;
		return nullptr;
	}

	if (OK != dev->init()) {
		delete dev;
		return nullptr;
	}

	return dev;
}

void MLX90393::print_usage()
{
	PRINT_MODULE_USAGE_NAME("mlx90393", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("magnetometer");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, true);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x0C);
	PRINT_MODULE_USAGE_PARAM_INT('R', 0, 0, 35, "Rotation", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" int mlx90393_main(int argc, char *argv[])
{
	int ch;
	using ThisDriver = MLX90393;
	BusCLIArguments cli{true, true};
	cli.default_i2c_frequency = 100000;
	cli.default_spi_frequency = 1000000;
	cli.i2c_address = MLX90393_I2C_ADDR;

	while ((ch = cli.getOpt(argc, argv, "R:")) != EOF) {
		switch (ch) {
		case 'R':
			cli.rotation = (enum Rotation)atoi(cli.optArg());
			break;
		}
	}

	const char *verb = cli.optArg();

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_MAG_DEVTYPE_MLX90393);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;
}
