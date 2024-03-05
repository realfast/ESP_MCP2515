#ifndef __ESP_CAN_H__
#define __ESP_CAN_H__
#include "can.h"
#include "mcp2515.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

  esp_err_t esp_can_send(can_interface_t *can_instance, const can_t *message);
  esp_err_t esp_can_receive(can_interface_t *can_instance, can_t *message);

#ifdef __cplusplus
}
#endif

#endif // __ESP_CAN_H__