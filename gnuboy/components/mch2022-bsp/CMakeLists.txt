idf_component_register(
    SRCS "hardware.c" "rp2040.c"
    INCLUDE_DIRS "."
    REQUIRES
        "bus-i2c"
        "sdcard"
        "spi-ili9341"
)
