#include <string.h>
#include <driver/gpio.h>
#include <driver/i2c.h>
#include <driver/spi_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <lwip/err.h>
#include <lwip/sys.h>
#include <m95_eeprom.hpp>
#include <ssd1963.hpp>
#include <hr2046.hpp>

extern "C" {
    void app_main();
}

static const char *TAG = "example";

#define ESP_INTR_FLAG_DEFAULT 0

#define SPI2_PIN_MISO 13
#define SPI2_PIN_MOSI 11
#define SPI2_PIN_CLK 12
#define SPI2_PIN_CS0 10
#define SPI2_PIN_CS1 15

#define SPI3_PIN_MISO 39
#define SPI3_PIN_MOSI 38
#define SPI3_PIN_CLK 40
#define SPI3_PIN_CS0 41
#define SPI3_PIN_CS1 42

#define EEPROM_CS SPI3_PIN_CS0
#define LCD_BUS_CS SPI2_PIN_CS0
#define LCD_RESET 7
#define LCD_CS 6
#define LCD_WR 5
#define LCD_RS 4

#define TOUCH_CS SPI2_PIN_CS1
#define TOUCH_IRQ 3

void led_blink_task(void *pvParameter) {
    // gpio_reset_pin(GPIO_NUM_10);
    gpio_pad_select_gpio(GPIO_NUM_10);
    gpio_set_direction(GPIO_NUM_10, GPIO_MODE_OUTPUT);

    while (1) {
        gpio_set_level(GPIO_NUM_10, 1);
        vTaskDelay(100 / portTICK_PERIOD_MS);

        gpio_set_level(GPIO_NUM_10, 0);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

/*void print_task(void *pvParameter) {
 
	while(1) {
	    printf("Hello world 42!\n");
	    vTaskDelay(1000 / portTICK_RATE_MS);
	}
}*/

void eepromTest(void *pvParameter) {
    spi_bus_config_t bus = {};
    bus.data0_io_num = -1;
    bus.data1_io_num = -1;
    bus.data2_io_num = -1;
    bus.data3_io_num = -1;
    bus.data4_io_num = -1;
    bus.data5_io_num = -1;
    bus.data6_io_num = -1;
    bus.data7_io_num = -1;
    bus.miso_io_num = SPI3_PIN_MISO;
    bus.mosi_io_num = SPI3_PIN_MOSI;
    bus.sclk_io_num = SPI3_PIN_CLK;
    bus.quadhd_io_num = -1;
    bus.quadwp_io_num = -1;
    bus.max_transfer_sz = 4;

    printf("spi_bus_initialize SPI3_HOST\n");
    auto ret = spi_bus_initialize(SPI3_HOST, &bus, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    printf("EEPROM initializing...\n");
    M95EEPROM eeprom{SPI3_HOST, EEPROM_CS, 24};
    printf("EEPROM initialized!\n");

    eeprom.writeByte(0x00, 44);
    eeprom.writeByte(0x01, 24);

    uint8_t b0;
    uint8_t b1;
    eeprom.readByte(0x00, &b0);
    eeprom.readByte(0x01, &b1);
    printf("EEPROM read: [%d, %d]\n", b0, b1);

    static const auto str = "Hello World!";
    eeprom.writeRange(0x05, reinterpret_cast<const uint8_t*>(str), strlen(str));

    char buffer[24];
    eeprom.readRange(0x05, reinterpret_cast<uint8_t*>(buffer), strlen(str));

    printf("EEPROM read range: [%s]\n", buffer);

    while (1) {
        // Add your main loop handling code here.
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void lcdTest(void *pvParameter) {
    spi_bus_config_t bus = {};
    bus.data0_io_num = -1;
    bus.data1_io_num = -1;
    bus.data2_io_num = -1;
    bus.data3_io_num = -1;
    bus.data4_io_num = -1;
    bus.data5_io_num = -1;
    bus.data6_io_num = -1;
    bus.data7_io_num = -1;
    bus.miso_io_num = SPI2_PIN_MISO;
    bus.mosi_io_num = SPI2_PIN_MOSI;
    bus.sclk_io_num = SPI2_PIN_CLK;
    bus.quadhd_io_num = -1;
    bus.quadwp_io_num = -1;
    bus.max_transfer_sz = 4;

    printf("spi_bus_initialize SPI2_HOST\n");
    auto ret = spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    ssd1963_handle_t lcd;
    ssd1963_config_t config = {};
    config.bus_host = SPI2_HOST;
    config.bus_cs_pin = LCD_BUS_CS;
    config.cs_pin = LCD_CS;
    config.reset_pin = LCD_RESET;
    config.rs_pin = LCD_RS;
    config.wr_pin = LCD_WR;
    config.width = 800;
    config.height = 480;

    printf("ssd1963_init\n");
    ret = ssd1963_init(&config, &lcd);
    ESP_ERROR_CHECK(ret);

    printf("ssd1963_reset\n");
    ret = ssd1963_reset(lcd);
    ESP_ERROR_CHECK(ret);

    printf("ssd1963_enable\n");
    ret = ssd1963_enable(lcd);
    ESP_ERROR_CHECK(ret);

    /*printf("ssd1963_fill_rect\n");
    ssd1963_fill_rect(lcd, (800 / 6) * 0, 0, 800 / 6, 480, rgb_color_16(255, 0, 0));
    ssd1963_fill_rect(lcd, (800 / 6) * 1, 0, 800 / 6, 480, rgb_color_16(0, 255, 0));
    ssd1963_fill_rect(lcd, (800 / 6) * 2, 0, 800 / 6, 480, rgb_color_16(0, 0, 255));
    ssd1963_fill_rect(lcd, (800 / 6) * 3, 0, 800 / 6, 480, rgb_color_16(255, 255, 255));
    ssd1963_fill_rect(lcd, (800 / 6) * 4, 0, 800 / 6, 480, rgb_color_16(127, 127, 127));
    ssd1963_fill_rect(lcd, (800 / 6) * 5, 0, 800 / 6, 480, rgb_color_16(0, 0, 0));*/

    //ssd1963_fill_test(lcd);

    printf("ssd1963 DONE\n");

    /*hr2046_config_t touch_config = {};
    touch_config.spi_host = SPI2_HOST;
    touch_config.cs_pin = TOUCH_CS;
    touch_config.irq_pin = TOUCH_IRQ;

    hr2046_handle_t touch;
    
    ret = hr2046_init(&touch_config, &touch);
    ESP_ERROR_CHECK(ret);

    ret = hr2046_power_down(touch);
    ESP_ERROR_CHECK(ret);*/

    while (1) {
        vTaskDelay(100 / portTICK_PERIOD_MS);

        /*uint16_t touch_values[2];
        hr2046_read(touch, HR2046_CH_X, &touch_values[0]);
        hr2046_read(touch, HR2046_CH_X, &touch_values[0]);
        hr2046_read(touch, HR2046_CH_Y, &touch_values[1]);*/

        // hr2046_has_touch(touch, &touched);

        //printf("Touch: [%d, %d]\n", touch_values[0], touch_values[1]);

        /*
        uint16_t touch_values[3];
        hr2046_read_xyz(touch, &touch_values[0], &touch_values[1], &touch_values[2]);

        printf("Touch: [%d, %d] %d\n", touch_values[0], touch_values[1], touch_values[2]);
        */
    }
}

void app_main() {
    printf("app_main!\n");
    nvs_flash_init();

    //install gpio isr service
    auto ret = gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    ESP_ERROR_CHECK(ret);

    // Initialize NVS
    /*esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);*/

    ESP_LOGI(TAG, "Starting blink");
    //xTaskCreate(&led_blink_task, "led_blink_task", 2048, NULL, 5, NULL);
    //xTaskCreate(&print_task, "print_task", 2048, NULL, 5, NULL);
    //xTaskCreate(&eepromTest, "eepromTest", 8192, NULL, 4, NULL);
    xTaskCreate(&lcdTest, "lcdTest", 8192, NULL, 5, NULL);
}
