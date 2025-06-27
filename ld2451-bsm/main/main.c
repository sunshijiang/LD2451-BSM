/* UART asynchronous example, that uses separate RX and TX tasks

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"

static const int RX_BUF_SIZE = 1024;

#define TXD_PIN (GPIO_NUM_21)
#define RXD_PIN (GPIO_NUM_20)
#define LED_PIN (GPIO_NUM_5)
#define KEY_PIN (GPIO_NUM_9)
#define GPIO_OUTPUT_PIN_SEL  (1ULL<<LED_PIN)
#define GPIO_INPUT_PIN_SEL (1Ull<<KEY_PIN)

//中断

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    static int cnt = 0;
    cnt++;
    //当中断被触发时，LED状态变化
    gpio_set_level(LED_PIN, cnt % 2);
}

void LedGpioConfig(void)
{

    gpio_config_t gpio_init_struct = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pin_bit_mask = GPIO_OUTPUT_PIN_SEL
    };
    esp_rom_gpio_pad_select_gpio(LED_PIN);

    gpio_config( &gpio_init_struct );
    gpio_set_level(LED_PIN, 1);
}

void KeyGpioConfig(void)
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        //设置输入引脚
        .pin_bit_mask = GPIO_INPUT_PIN_SEL,
        //设置输入模式
        .mode = GPIO_MODE_INPUT,
        //使能上拉电阻
        .pull_up_en = 1,
        //失能下拉电阻
        .pull_down_en = 0
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(ESP_INTR_FLAG_EDGE);
    gpio_isr_handler_add(KEY_PIN, gpio_isr_handler, (void*)NULL);
    gpio_intr_enable(KEY_PIN);


}

bool GetKeyValue(void)
{
    //如果按键状态为0
    if( gpio_get_level(KEY_PIN) == 0 )
    {
        //延时消抖，使用该延时需要加入对应头文件
        vTaskDelay(100 / portTICK_PERIOD_MS);
        // 如果按键状态还是0，说明按键真的按下
        if( gpio_get_level(KEY_PIN) == 0 )
        {
            return 0;
        }
    }
    return 1;
}

void LedOn(void)
{
    gpio_set_level(LED_PIN, 0);
}

void LedOff(void)
{
    gpio_set_level(LED_PIN, 1);
}


void init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

int sendData(const char* logName, const char* data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}

// static void tx_task(void *arg)
// {
//     static const char *TX_TASK_TAG = "TX_TASK";
//     esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
//     while (1) {
//         sendData(TX_TASK_TAG, "Hello world");
//         vTaskDelay(2000 / portTICK_PERIOD_MS);
//     }
// }

static void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE + 1);
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
        }
    }
    free(data);
}

void app_main(void)
{
    init();
    int cnt = 0;
    int time = 0;
    xTaskCreate(rx_task, "uart_rx_task", 1024 * 2, NULL, configMAX_PRIORITIES - 1, NULL);
    // xTaskCreate(tx_task, "uart_tx_task", 1024 * 2, NULL, configMAX_PRIORITIES - 2, NULL);

    LedGpioConfig();
    KeyGpioConfig();
    // while(1) {
    //     //因ESP32S3采用的是RTOS方式运行，必须在死循环中加入延时让其能够正常运转
    //     vTaskDelay(20 / portTICK_PERIOD_MS);

    //     //如果按键有按下
    //     if( GetKeyValue() == 0 )
    //     {
    //         //使LED状态取反。LED引脚为GPIO48
    //         gpio_set_level(LED_PIN, cnt = !cnt);
    //     }
    // }
    while(1)
    {
        printf("time: %d\n", time++);
        vTaskDelay(1000 /portTICK_PERIOD_MS);
    }
}
