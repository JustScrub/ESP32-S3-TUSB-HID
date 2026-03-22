idf_component_register(
    SRCS "nimble.c" "tusb_hid_example_main.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_driver_gpio bt nvs_flash
    )
