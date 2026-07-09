idf_component_register(
    SRCS "station_example_main.c"
           "audio_speaker.c"
           "imu_atk_ms601m.c"
           "sd_card.c"
    INCLUDE_DIRS "."
    REQUIRES
        nvs_flash
        esp_event
        esp_netif
        esp_wifi
        esp_wifi_remote
        esp_hosted
        esp_http_server
        esp_timer
        esp_video
        esp_cam_sensor
        esp_sccb_intf
        esp_driver_i2c
        esp_driver_i2s
        esp_driver_gpio
        esp_driver_jpeg
        esp_codec_dev
        esp_driver_uart
        esp_driver_sdmmc
        fatfs
)
