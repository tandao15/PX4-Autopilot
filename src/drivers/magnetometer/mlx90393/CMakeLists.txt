px4_add_module(
	MODULE drivers__magnetometer__mlx90393
	MAIN mlx90393
	COMPILE_FLAGS
	SRCS
		mlx90393_i2c.cpp
		mlx90393_spi.cpp
		mlx90393_main.cpp
		mlx90393.cpp
	DEPENDS
		drivers_magnetometer
		px4_work_queue
	)
