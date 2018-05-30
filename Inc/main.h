/**
  ******************************************************************************
  * File Name          : main.hpp
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define DIAG_F_R_Pin GPIO_PIN_0
#define DIAG_F_R_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_1
#define LED_GPIO_Port GPIOB
#define DIAG_F_L_Pin GPIO_PIN_2
#define DIAG_F_L_GPIO_Port GPIOB
#define DIAG_B_R_Pin GPIO_PIN_10
#define DIAG_B_R_GPIO_Port GPIOB
#define DIAG_B_L_Pin GPIO_PIN_11
#define DIAG_B_L_GPIO_Port GPIOB
#define INA_B_R_Pin GPIO_PIN_12
#define INA_B_R_GPIO_Port GPIOB
#define INB_B_R_Pin GPIO_PIN_13
#define INB_B_R_GPIO_Port GPIOB
#define INA_B_L_Pin GPIO_PIN_14
#define INA_B_L_GPIO_Port GPIOB
#define INB_B_L_Pin GPIO_PIN_15
#define INB_B_L_GPIO_Port GPIOB
#define PWM_B_R_Pin GPIO_PIN_8
#define PWM_B_R_GPIO_Port GPIOA
#define PWM_B_L_Pin GPIO_PIN_9
#define PWM_B_L_GPIO_Port GPIOA
#define PWM_F_R_Pin GPIO_PIN_10
#define PWM_F_R_GPIO_Port GPIOA
#define PWM_F_L_Pin GPIO_PIN_11
#define PWM_F_L_GPIO_Port GPIOA
#define INB_F_L_Pin GPIO_PIN_15
#define INB_F_L_GPIO_Port GPIOA
#define INA_F_L_Pin GPIO_PIN_3
#define INA_F_L_GPIO_Port GPIOB
#define INB_F_R_Pin GPIO_PIN_4
#define INB_F_R_GPIO_Port GPIOB
#define INA_F_R_Pin GPIO_PIN_5
#define INA_F_R_GPIO_Port GPIOB
#define BUT_Pin GPIO_PIN_8
#define BUT_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
#define PWM_B_R TIM1->CCR1
#define PWM_B_L TIM1->CCR2
#define PWM_F_R TIM1->CCR3
#define PWM_F_L TIM1->CCR4
/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
