#include <string.h>
#include "esp_can.h"
#include "esp_log.h"

esp_err_t esp_can_send(can_interface_t *can_instance, const can_t *message)
{
  switch (can_instance->controller_type)
  {
  case CAN_CONTROLLER_TWAI:
    return ESP_OK;
  case CAN_CONTROLLER_MCP2515:
    return MCP2515_sendMessageAfterCtrlCheck(can_instance->controller_instance, message);
  }
  return ESP_FAIL;
}

esp_err_t esp_can_receive(can_interface_t *can_instance, can_t *message)
{
  switch (can_instance->controller_type)
  {
  case CAN_CONTROLLER_TWAI:
    return ESP_OK;
  case CAN_CONTROLLER_MCP2515:
    return MCP2515_readMessageAfterStatCheck(can_instance->controller_instance, message);
  }
  return ESP_FAIL;
}
