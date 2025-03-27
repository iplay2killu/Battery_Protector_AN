idf_component_register(SRCS "lis3dh_reg.c" "hello_world_main.c"
                    PRIV_REQUIRES spi_flash driver
                    INCLUDE_DIRS ".")