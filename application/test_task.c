/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       test_task.c/h
  * @brief      buzzer warning task.��������������
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "test_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "remote_control.h"
#include "tim.h"
#include "default_task.h"
/**
  * @brief          test task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          test
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
uint8_t arm=0;
uint8_t flight_mode=0;


void test_task(void const * argument)
{

    while(1)
    {
      switch (rc_ctrl.rc.s[1])
      {
      case 1:
        Quad_Drone_x.arm=DISARMED;
        break;
      case 2:
        Quad_Drone_x.arm=TEST;
        break;
      case 3:
        Quad_Drone_x.arm=ARMED;
        break;      
      default:
				arm=1;
        break;
      }
      switch (rc_ctrl.rc.s[0])
      {
      case 1:
				Quad_Drone_x.last_flight_mode=Quad_Drone_x.flight_mode;
				Quad_Drone_x.flight_mode=FLIGHT_MODE_ANGLE;
        flight_mode=LEVEL;
        break;
      case 2:
        // flight_mode=ACRO;
        Quad_Drone_x.last_flight_mode=Quad_Drone_x.flight_mode;
				Quad_Drone_x.flight_mode=FLIGHT_MODE_HORIZEN;

        break; 
      case 3:
        // flight_mode=HORIZEN;
        Quad_Drone_x.last_flight_mode=Quad_Drone_x.flight_mode;
				Quad_Drone_x.flight_mode=FLIGHT_MODE_ACRO;
        break;      
      default:
        break;
      }      
      if(arm==0)//
      {
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1000);
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 1000);
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 1000);
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 1000);
      }
        osDelay(10);
    }
}



