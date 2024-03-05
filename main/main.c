#include <stdio.h>
#include "esp_rom_gpio.h"
#include "driver/gpio.h"
#include <string.h>
#include "esp_log.h"
#include "esp_spiffs.h"
#include "esp_can.h"
#include "freertos/queue.h"

#include "driver/spi_master.h"
#include "freertos/semphr.h"

SemaphoreHandle_t spi_mux = NULL;

can_interface_t CAN1;
can_interface_t CAN2;


#define CAN_QUEUE_SIZE 25
static QueueHandle_t canQueue;

TaskHandle_t can1Handle = NULL;
TaskHandle_t can2Handle = NULL;
TaskHandle_t busReceiveHandle = NULL;
uint64_t heardFromCan2;
uint64_t heardFromCan1;

#define TAG "MAIN"

bool SPI_Init(void)
{
  esp_err_t ret;

  gpio_num_t miso = GPIO_NUM_14;
  gpio_num_t mosi = GPIO_NUM_13;
  gpio_num_t sclk = GPIO_NUM_3;

  gpio_reset_pin(miso);
  gpio_reset_pin(mosi);
  gpio_reset_pin(sclk);

  gpio_set_direction(miso, GPIO_MODE_INPUT);
  gpio_set_direction(mosi, GPIO_MODE_OUTPUT);
  gpio_set_direction(sclk, GPIO_MODE_OUTPUT);

  ESP_LOGI(TAG, "MISO: %d, MOSI: %d, SCLK: %d", miso, mosi, sclk);

  // Configuration for the SPI bus
  spi_bus_config_t bus_cfg = {
      .miso_io_num = miso,
      .mosi_io_num = mosi,
      .sclk_io_num = sclk,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = 0 // no limit
  };
  // Initialize SPI bus
  ret = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);

  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to initialize SPI bus.");
    return false;
  }

  return true;
}

static void bus_receive_task(void *pvParameters)
{

  while (1)
  {
    can_t message;
    // Take can driver mutex
    // Process all messages in the queue
    if (xQueueReceive(canQueue, &message, portMAX_DELAY) == pdPASS)
    {
      if (message.id & 0x80000000)
      {
        message.id &= ~0x80000000; // Clear the MSB
        message.extended = true;
      }
      else
      {
        message.extended = false;
      }
      if (message.bus_number == 1)
      {
        heardFromCan1 = xTaskGetTickCount();
      }
      else
      {
        heardFromCan2 = xTaskGetTickCount();
      }
      // NORMALLY I would send this out via TWAI
    }
  }
}

void IRAM_ATTR bus_interrupt_callback(void *arg)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  // Cast arg back to the correct type
  TaskHandle_t taskHandle = (TaskHandle_t)arg;

  // Notify the task
  vTaskNotifyGiveFromISR(taskHandle, &xHigherPriorityTaskWoken);

  if (xHigherPriorityTaskWoken)
  {
    portYIELD_FROM_ISR();
  }
}

void handlerTask(void *pvParameters)
{
  can_interface_t *can_instance = (can_interface_t *)pvParameters;

  // Get the handle of the current task
  TaskHandle_t currentTaskHandle = xTaskGetCurrentTaskHandle();

  // Register the interrupt handler for the CAN interface
  gpio_isr_handler_add(can_instance->interrupt_pin, bus_interrupt_callback, (void *)currentTaskHandle);

  // Clear the RXnOVR flag
  MCP2515_clearRXnOVR(can_instance->controller_instance);

  can_t message;
  while (1)
  {
    // Wait indefinitely for a notification
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    xSemaphoreTake(spi_mux, portMAX_DELAY);
    if (esp_can_receive(can_instance, &message) == ESP_OK)
    {
      message.bus_number = can_instance->busNumber;
      xQueueSendToBack(canQueue, &message, 0);
    }
    xSemaphoreGive(spi_mux);
  }
}

void app_main(void)
{
  if (SPI_Init() != true)
  {
    ESP_LOGE(TAG, "Failed to initialize SPI");
    return;
  }

  spi_mux = xSemaphoreCreateMutex();

  canQueue = xQueueCreate(CAN_QUEUE_SIZE, sizeof(can_t));

  gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);

  CAN1 = init_mcp2515_can_interface(GPIO_NUM_45, GPIO_NUM_9, CAN_250KBPS, MCP_16MHZ);
  CAN1.busNumber = 1;
  CAN2 = init_mcp2515_can_interface(GPIO_NUM_38, GPIO_NUM_17, CAN_250KBPS, MCP_16MHZ);
  CAN2.busNumber = 2;

  // xTaskCreate(bus_receive_task, "BusReceiveTask", 1024 * 5, NULL, 5, &busReceiveHandle);
  xTaskCreatePinnedToCore(bus_receive_task, "BusReceiveTask", 1024 * 5, NULL, 5, &busReceiveHandle, 0);
  xTaskCreatePinnedToCore(handlerTask, "HandlerTaskCAN1", 1024 * 4, &CAN1, configMAX_PRIORITIES - 1, &can1Handle, 0);
  xTaskCreatePinnedToCore(handlerTask, "HandlerTaskCAN2", 1024 * 4, &CAN2, configMAX_PRIORITIES - 2, &can2Handle, 0);

  heardFromCan1 = xTaskGetTickCount();
  heardFromCan2 = heardFromCan1;
  while (true)
  {
    if (xTaskGetTickCount() - heardFromCan1 > pdMS_TO_TICKS(10000))
    {
      uint8_t eflg = MCP2515_getErrorFlags(CAN2.controller_instance);
      ESP_LOGE(TAG, "CAN 1 Error Flags: %d", eflg);
      ESP_LOGE(TAG, "No messages have been received in 10 seconds");
      esp_restart();
    }
    else if (xTaskGetTickCount() - heardFromCan1 > pdMS_TO_TICKS(1000))
    {
      uint8_t eflg = MCP2515_getErrorFlags(CAN1.controller_instance);
      ESP_LOGE(TAG, "CAN 1 Error Flags: %d", eflg);
      ESP_LOGE(TAG, "CAN1 has not been heard from in 1 seconds");
      MCP2515_clearRXnOVR(CAN1.controller_instance);
    }
    if (xTaskGetTickCount() - heardFromCan2 > pdMS_TO_TICKS(10000))
    {
      uint8_t eflg = MCP2515_getErrorFlags(CAN2.controller_instance);
      ESP_LOGE(TAG, "CAN 2 Error Flags: %d", eflg);
      ESP_LOGE(TAG, "No messages have been received in 10 seconds");
      esp_restart();
    }
    else if (xTaskGetTickCount() - heardFromCan2 > pdMS_TO_TICKS(1000))
    {
      uint8_t eflg = MCP2515_getErrorFlags(CAN2.controller_instance);
      ESP_LOGE(TAG, "CAN 2 Error Flags: %d", eflg);
      ESP_LOGE(TAG, "CAN2 has not been heard from in 1 seconds");
      MCP2515_clearRXnOVR(CAN2.controller_instance);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
