#include <stdio.h>
#include "u8g2.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "sdkconfig.h"
#include "u8g2_esp32_hal.h"

void task_test_LCD(void *ignore) 
{
    u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
    u8g2_esp32_hal.clk   = CONFIG_PIN_NUM_CLK;
    u8g2_esp32_hal.mosi  = CONFIG_PIN_NUM_MOSI;
    u8g2_esp32_hal.cs    = CONFIG_PIN_NUM_CS;
    u8g2_esp32_hal.dc    = CONFIG_PIN_NUM_DC;
    u8g2_esp32_hal.reset = CONFIG_PIN_NUM_RST;
    u8g2_esp32_hal_init(u8g2_esp32_hal);


    u8g2_t u8g2; // a structure which will contain all the data for one display

    u8g2_Setup_st7565_ea_dogm128_f(&u8g2, U8G2_R0, u8g2_esp32_msg_comms_cb, u8g2_esp32_msg_gpio_and_delay_cb);
    u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in sleep mode after this,

    u8g2_SetContrast(&u8g2, 30);
    u8g2_SetPowerSave(&u8g2, 0); // wake up display
    u8g2_ClearBuffer(&u8g2);
    u8g2_DrawBox(&u8g2, 10,20, 20, 30);
    u8g2_SetFont(&u8g2, u8g2_font_ncenB14_tr);
    u8g2_DrawStr(&u8g2, 0,15,"Hello World!");
    u8g2_SendBuffer(&u8g2);

    printf("All done!");

    vTaskDelete(NULL);
}

void app_main()
{
    xTaskCreate(&task_test_LCD, "LCD_TASK", 2048, NULL, 5, NULL);
}


