/* main.c
   ESP-IDF Example: Smart house with ISR + Mutex
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc/adc_oneshot.h"

/* --- Hardware config --- */
#define LED_GPIO      10
#define BUTTON_GPIO    9
#define BUZZER_GPIO    5
#define GAS_SENSOR_ADC ADC_CHANNEL_0   // GPIO0 trên ESP32-C3
#define GAS_THRESHOLD 2000

static const char *TAG = "SMART_HOUSE";

/* --- LED commands --- */
typedef enum {
    CMD_LIGHT_OFF = 0,
    CMD_LIGHT_ON  = 1,
    CMD_LIGHT_TOGGLE = 2
} light_cmd_t;

/* --- Gas event --- */
typedef struct {
    int gas_detected; // 0 = bình thường, 1 = có gas
} gas_event_t;

/* --- Queues --- */
static QueueHandle_t xQueueLight = NULL;
static QueueHandle_t xQueueGas = NULL;

/* --- Semaphore / Mutex --- */
static SemaphoreHandle_t xButtonSemaphore = NULL;
static SemaphoreHandle_t xLogMutex = NULL;

/* --- Internal states --- */
static bool light_state = false;

/* ------------------- LED Controller Task ------------------- */
void ControllerTask(void *pvParameters)
{
    light_cmd_t cmd;
    for (;;) {
        if (xQueueReceive(xQueueLight, &cmd, portMAX_DELAY) == pdTRUE) {
            if (xSemaphoreTake(xLogMutex, portMAX_DELAY)) {
                switch (cmd) {
                    case CMD_LIGHT_ON:
                        light_state = true;
                        gpio_set_level(LED_GPIO, 1);
                        ESP_LOGI(TAG, "Controller: LIGHT ON");
                        break;
                    case CMD_LIGHT_OFF:
                        light_state = false;
                        gpio_set_level(LED_GPIO, 0);
                        ESP_LOGI(TAG, "Controller: LIGHT OFF");
                        break;
                    case CMD_LIGHT_TOGGLE:
                        light_state = !light_state;
                        gpio_set_level(LED_GPIO, light_state ? 1 : 0);
                        ESP_LOGI(TAG, "Controller: LIGHT TOGGLED -> %s", light_state ? "ON":"OFF");
                        break;
                }
                xSemaphoreGive(xLogMutex);
            }
        }
    }
}

/* ------------------- Button Task (ISR-based) ------------------- */
void ButtonTask(void *pvParameters)
{
    for (;;) {
        if (xSemaphoreTake(xButtonSemaphore, portMAX_DELAY)) {
            light_cmd_t cmd = CMD_LIGHT_TOGGLE;
            if (xQueueSend(xQueueLight, &cmd, pdMS_TO_TICKS(10)) == pdTRUE) {
                if (xSemaphoreTake(xLogMutex, portMAX_DELAY)) {
                    ESP_LOGI(TAG, "ButtonTask: enqueued TOGGLE");
                    xSemaphoreGive(xLogMutex);
                }
            }
        }
    }
}

/* ISR Handler for Button */
static void IRAM_ATTR button_isr_handler(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xButtonSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* ------------------- Simulated Remote Task ------------------- */
void SimTask(void *pvParameters)
{
    bool send_on = true;
    for (;;) {
        light_cmd_t cmd = send_on ? CMD_LIGHT_ON : CMD_LIGHT_OFF;
        if (xQueueSend(xQueueLight, &cmd, 0) == pdTRUE) {
            if (xSemaphoreTake(xLogMutex, portMAX_DELAY)) {
                ESP_LOGI(TAG, "SimTask: sent %s", send_on ? "ON" : "OFF");
                xSemaphoreGive(xLogMutex);
            }
            send_on = !send_on;
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

/* ------------------- Logger Task ------------------- */
void LoggerTask(void *pvParameters)
{
    for (;;) {
        if (xSemaphoreTake(xLogMutex, portMAX_DELAY)) {
            ESP_LOGI(TAG, "Logger: light_state=%s", light_state ? "ON" : "OFF");
            xSemaphoreGive(xLogMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

/* ------------------- Gas Sensor Task ------------------- */
void GasSensorTask(void *pvParameters)
{
    gas_event_t event;

    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t adc1_cfg = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    adc_oneshot_new_unit(&adc1_cfg, &adc1_handle);

    adc_oneshot_chan_cfg_t adc_chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    adc_oneshot_config_channel(adc1_handle, GAS_SENSOR_ADC, &adc_chan_cfg);

    for (;;) {
        int adc_val;
        adc_oneshot_read(adc1_handle, GAS_SENSOR_ADC, &adc_val);

        event.gas_detected = (adc_val > GAS_THRESHOLD) ? 1 : 0;
        xQueueSend(xQueueGas, &event, pdMS_TO_TICKS(10));

        if (xSemaphoreTake(xLogMutex, portMAX_DELAY)) {
            ESP_LOGI(TAG, "GasSensorTask: MQ-2 ADC=%d", adc_val);
            xSemaphoreGive(xLogMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/* ------------------- Buzzer Task ------------------- */
void BuzzerTask(void *pvParameters)
{
    gas_event_t event;
    gpio_reset_pin(BUZZER_GPIO);
    gpio_set_direction(BUZZER_GPIO, GPIO_MODE_OUTPUT);

    for (;;) {
        if (xQueueReceive(xQueueGas, &event, portMAX_DELAY)) {
            if (event.gas_detected) {
                if (xSemaphoreTake(xLogMutex, portMAX_DELAY)) {
                    ESP_LOGW(TAG, "GAS DETECTED! BUZZER ON");
                    xSemaphoreGive(xLogMutex);
                }
                gpio_set_level(BUZZER_GPIO, 1);
                vTaskDelay(pdMS_TO_TICKS(500));
                gpio_set_level(BUZZER_GPIO, 0);
                vTaskDelay(pdMS_TO_TICKS(500));
            } else {
                gpio_set_level(BUZZER_GPIO, 0);
            }
        }
    }
}

/* ------------------- Main ------------------- */
void app_main(void)
{
    ESP_LOGI(TAG, "Smart house system starting...");

    // LED init
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 0);
    light_state = false;

    // Button init
    gpio_reset_pin(BUTTON_GPIO);
    gpio_set_direction(BUTTON_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_GPIO, GPIO_PULLUP_ONLY);

    // Init ISR for button
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_GPIO, button_isr_handler, NULL);
    gpio_set_intr_type(BUTTON_GPIO, GPIO_INTR_NEGEDGE); // active low press

    // Queues
    xQueueLight = xQueueCreate(10, sizeof(light_cmd_t));
    xQueueGas   = xQueueCreate(5, sizeof(gas_event_t));

    // Semaphores / Mutex
    xButtonSemaphore = xSemaphoreCreateBinary();
    xLogMutex        = xSemaphoreCreateMutex();

    if (!xQueueLight || !xQueueGas || !xButtonSemaphore || !xLogMutex) {
        ESP_LOGE(TAG, "Failed to create queues or semaphores");
        return;
    }

    // Create tasks with priorities
    xTaskCreatePinnedToCore(ControllerTask, "ControllerTask", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(ButtonTask, "ButtonTask", 3072, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(SimTask, "SimTask", 3072, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(LoggerTask, "LoggerTask", 2048, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(GasSensorTask, "GasSensorTask", 3072, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(BuzzerTask, "BuzzerTask", 2048, NULL, 3, NULL, 0);

    ESP_LOGI(TAG, "All tasks started with ISR + Mutex sync.");
}
