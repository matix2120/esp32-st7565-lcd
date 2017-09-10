#include <stdio.h>
#include <string.h>

#include "sdkconfig.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "u8g2_esp32_hal.h"

static const char *TAG = "u8g2_hal";

static spi_device_handle_t handle; // SPI handle.
static u8g2_esp32_hal_t u8g2_esp32_hal; // HAL state data.

#undef ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x)   do { esp_err_t rc = (x); if (rc != ESP_OK) { ESP_LOGE("err", "esp_err_t = %x", rc); assert(0 && #x);} } while(0);

#undef LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL 2

/*
 * Initialze the ESP32 HAL.
 */
void u8g2_esp32_hal_init(u8g2_esp32_hal_t u8g2_esp32_hal_param) {
    u8g2_esp32_hal = u8g2_esp32_hal_param;
} // u8g2_esp32_hal_init

/*
 * HAL callback function as prescribed by the U8G2 library.  This callback is invoked
 * to handle callbacks for communications.
 */
uint8_t u8g2_esp32_msg_comms_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    //printf("msg_comms_cb: Received a msg: %d\r\n", msg);
    switch(msg) {
        case U8X8_MSG_BYTE_SET_DC:
        {
            ESP_LOGD(TAG, "u8g2_esp32_msg_comms_cb: U8X8_MSG_BYTE_SET_DC: %d\n", arg_int);
            if (u8g2_esp32_hal.dc != U8G2_ESP32_HAL_UNDEFINED) {
                gpio_set_level(u8g2_esp32_hal.dc, arg_int);
            }
            break;
        }
        case U8X8_MSG_BYTE_INIT: 
        {
            ESP_LOGD(TAG, "u8g2_esp32_msg_comms_cb: U8X8_MSG_BYTE_INIT\n");
            if (u8g2_esp32_hal.clk == U8G2_ESP32_HAL_UNDEFINED ||
                    u8g2_esp32_hal.mosi == U8G2_ESP32_HAL_UNDEFINED ||
                    u8g2_esp32_hal.cs == U8G2_ESP32_HAL_UNDEFINED) {
                break;
            }

            spi_bus_config_t bus_config;
            bus_config.sclk_io_num   = u8g2_esp32_hal.clk; // CLK
            bus_config.mosi_io_num   = u8g2_esp32_hal.mosi; // MOSI
            bus_config.miso_io_num   = -1; // MISO
            bus_config.quadwp_io_num = -1; // Not used
            bus_config.quadhd_io_num = -1; // Not used
            ESP_LOGI(TAG, "... Initializing bus.");
            //vTaskDelay(100/portTICK_PERIOD_MS);
            ESP_ERROR_CHECK(spi_bus_initialize(VSPI_HOST, &bus_config, 1));

            spi_device_interface_config_t dev_config;
            dev_config.address_bits     = 0;
            dev_config.command_bits     = 0;
            dev_config.dummy_bits       = 0;
            dev_config.mode             = 0;
            dev_config.duty_cycle_pos   = 0;
            dev_config.cs_ena_posttrans = 0;
            dev_config.cs_ena_pretrans  = 0;
            dev_config.clock_speed_hz   = 10000000;
            dev_config.spics_io_num     = u8g2_esp32_hal.cs;
            dev_config.flags            = 0;
            dev_config.queue_size       = 200;
            dev_config.pre_cb           = NULL;
            dev_config.post_cb          = NULL;
            ESP_LOGI(TAG, "... Adding device bus.");
            ESP_ERROR_CHECK(spi_bus_add_device(VSPI_HOST, &dev_config, &handle));

          break;
        }

        case U8X8_MSG_BYTE_SEND: 
        {
            ESP_LOGD(TAG, "u8g2_esp32_msg_comms_cb: U8X8_MSG_BYTE_SEND\n");
            spi_transaction_t trans_desc;
            trans_desc.addr      = 0;
            trans_desc.cmd          = 0;
            trans_desc.flags     = 0;
            trans_desc.length    = 8 * arg_int; // Number of bits NOT number of bytes.
            trans_desc.rxlength  = 0;
            trans_desc.tx_buffer = arg_ptr;
            trans_desc.rx_buffer = NULL;

            ESP_LOGI(TAG, "... Transmitting %d bytes.", arg_int);
            ESP_ERROR_CHECK(spi_device_transmit(handle, &trans_desc));
            break;
        }
        default:
        ESP_LOGD(TAG, "u8g2_esp32_msg_comms_cb: NO CASE for %d\n", msg);
    }
    return 0;
} 

uint8_t u8g2_esp32_msg_gpio_and_delay_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) 
{
    switch(msg) 
    {

    // Initialize the GPIO and DELAY HAL functions.  If the pins for DC and RESET have been
    // specified then we define those pins as GPIO outputs.
        case U8X8_MSG_GPIO_AND_DELAY_INIT: {
            ESP_LOGD(TAG, "u8g2_esp32_msg_gpio_and_delay_cb: U8X8_MSG_GPIO_AND_DELAY_INIT\n");
            uint64_t bitmask = 0;
            if (u8g2_esp32_hal.dc != U8G2_ESP32_HAL_UNDEFINED) {
                bitmask = bitmask | (1<<u8g2_esp32_hal.dc);
            }
            if (u8g2_esp32_hal.reset != U8G2_ESP32_HAL_UNDEFINED) {
                bitmask = bitmask | (1<<u8g2_esp32_hal.reset);
            }

            gpio_config_t gpioConfig;
            gpioConfig.pin_bit_mask = bitmask;
            gpioConfig.mode         = GPIO_MODE_OUTPUT;
            gpioConfig.pull_up_en   = GPIO_PULLUP_DISABLE;
            gpioConfig.pull_down_en = GPIO_PULLDOWN_ENABLE;
            gpioConfig.intr_type    = GPIO_INTR_DISABLE;
            gpio_config(&gpioConfig);
            break;
        }

    // Set the GPIO reset pin to the value passed in through arg_int.
        case U8X8_MSG_GPIO_RESET:
        {
            ESP_LOGD(TAG, "u8g2_esp32_msg_gpio_and_delay_cb: U8X8_MSG_GPIO_RESET\n");
            if (u8g2_esp32_hal.reset != U8G2_ESP32_HAL_UNDEFINED) {
                gpio_set_level(u8g2_esp32_hal.reset, arg_int);
            }
            break;
        }
    // Delay for the number of milliseconds passed in through arg_int.
        case U8X8_MSG_DELAY_MILLI:
        ESP_LOGD(TAG, "u8g2_esp32_msg_gpio_and_delay_cb: U8X8_MSG_DELAY_MILLI\n");
            vTaskDelay(arg_int/portTICK_PERIOD_MS);
            break;
        default:
            printf("ERR, no case\r\n");
            break;
    }
    return 0;
} // u8g2_esp32_msg_gpio_and_delay_cb

