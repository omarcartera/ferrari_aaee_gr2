#ifndef __MAIN_H
#define __MAIN_H
#include "stm32f0xx_hal.h"
#include "stm32072b_eval.h"

/* Definition for CANx clock resources */
#define CANx                            CAN
#define CANx_CLK_ENABLE()               __HAL_RCC_CAN1_CLK_ENABLE()
#define CANx_GPIO_CLK_ENABLE()          __HAL_RCC_GPIOD_CLK_ENABLE()

#define CANx_FORCE_RESET()              __HAL_RCC_CAN1_FORCE_RESET()
#define CANx_RELEASE_RESET()            __HAL_RCC_CAN1_RELEASE_RESET()

/* Definition for USARTx Pins */
#define CANx_TX_PIN                    GPIO_PIN_1
#define CANx_TX_GPIO_PORT              GPIOD
#define CANx_TX_AF                     GPIO_AF0_CAN
#define CANx_RX_PIN                    GPIO_PIN_0
#define CANx_RX_GPIO_PORT              GPIOD
#define CANx_RX_AF                     GPIO_AF0_CAN

/* Definition for USARTx's NVIC */
#define CANx_RX_IRQn                   CEC_CAN_IRQn
#define CANx_RX_IRQHandler             CEC_CAN_IRQHandler

#endif /* __MAIN_H */
