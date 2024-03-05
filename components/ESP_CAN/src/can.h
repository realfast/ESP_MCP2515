#ifndef __CAN_H__
#define __CAN_H__

#include <stdint.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/twai.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

  typedef enum
  {
    CAN_CONTROLLER_TWAI,
    CAN_CONTROLLER_MCP2515
  } can_controller_type_t;

  typedef struct can_interface
  {
    can_controller_type_t controller_type;
    void *controller_instance;   // Generic pointer to the controller instance
    uint32_t canSpeed;           // Speed in kbps
    uint8_t busNumber;           // Bus number
    gpio_num_t interrupt_pin;    // Int pin
    TaskHandle_t task_handle;    // Task handle
    SemaphoreHandle_t semaphore; // Semaphore
  } can_interface_t;

  typedef struct
  {
    // The order of these bits must match deprecated message flags for compatibility reasons
    uint32_t extended : 1;     /**< Extended Frame Format (29bit ID) */
    uint32_t rtr : 1;          /**< Message is a Remote Frame */
    uint32_t ss : 1;           /**< Transmit as a Single Shot Transmission. Unused for received. */
    uint32_t self : 1;         /**< Transmit as a Self Reception Request. Unused for received. */
    uint32_t dlc_non_comp : 1; /**< Message's Data length code is larger than 8. This will break compliance with ISO 11898-1 */
    uint32_t reserved : 27;    /**< Reserved bits */

    uint32_t id;        /**< 11 or 29 bit identifier */
    uint8_t length;     /**< Data length code */
    uint8_t data[8];    /**< Data bytes (not relevant in RTR frame) */
    uint8_t bus_number; /**< Bus number */
  } can_t;

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */
