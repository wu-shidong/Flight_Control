#include "main.h"
#include "cmsis_os.h"
#include "tim.h"
#include "usart.h"
#include "default_task.h"
// #include "remote_control.h"
#include "bsp_rc.h"
#include "bsp_fric.h"
#include "test_task.h"
#include "user_pid.h"
// #include "INS_task.h"
/*
                   _ooOoo_
                  o8888888o
                  88" . "88
                  (| -_- |)
                  O\  =  /O
               ____/`---'\____
             .'  \\|     |//  `.
            /  \\|||  :  |||//  \
           /  _||||| -:- |||||-  \
           |   | \\\  -  /// |   |
           | \_|  ''\---/''  |   |
           \  .-\__  `-`  ___/-. /
         ___`. .'  /--.--\  `. . __
      ."" '<  `.___\_<|>_/___.'  >'"".
     | | :  `- \`.;`\ _ /`;.`/ - ` : | |
     \  \ `-.   \_ __\ /__ _/   .-` /  /
======`-.____`-.___\_____/___.-`____.-'======
                   `=---='
*/
/* USER CODE BEGIN Header_DefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_DefaultTask */
drone_t Quad_Drone_x;//正叉结构四旋翼无人机
motor_t motor[4];
float pitch_angle_set=0.0f;
float roll_angle_set=0.0f;
float yaw_gyro_set=0.0f;
float throttle_set=0.0f;
float yaw_gyro_fb =0.0f;
pid_struct_t level_pitch_angle_pid;//自稳模式下俯仰轴角度环pid
pid_struct_t level_pitch_gyro_pid;//自稳模式下俯仰轴角速度环pid
pid_struct_t level_roll_angle_pid;//自稳模式下横滚轴角度环pid
pid_struct_t level_roll_gyro_pid;//自稳模式下横滚轴角速度环pid
// pid_struct_t level_yaw_angle_pid;//自稳模式下偏航轴角度环pid（没啥意义）
pid_struct_t level_yaw_gyro_pid;//自稳模式下偏航轴角速度环pid
// void speed_to_control_vector(int16_t ch_throttle,int16_t ch_pitch,int16_t ch_roll ,int16_t yaw);
void motor_init(void);
void drone_level_control(void);
void motor_cmd(int16_t motor_0,int16_t motor_1 ,int16_t motor_2,int16_t motor_3);
void drone_level_init(void);
void motor_no_force(void);
void flight_feedback_update(void);
void acro_init(void);
void acro_control(void);
void DefaultTask(void const * argument)
{
  /* USER CODE BEGIN DefaultTask */

	remote_control_init();
  Tim_Init();
  motor_init();
  drone_level_init();
  acro_init();
  /* Infinite loop */
  for(;;)
  {
    if(Quad_Drone_x.arm==ARMED)
    {
      
//       if(flight_mode==LEVEL)
//       {
// //        motor_no_force();
// 				 drone_level_control();
//       }
      if(Quad_Drone_x.flight_mode==FLIGHT_MODE_ACRO)
      {
        acro_control();
      }
    }else{
        motor_no_force();
    }
    osDelay(10);
  }
  /* USER CODE END DefaultTask */
}
// int16_t motor_Speed_limit(int16_t speed)
// {
//   int16_t speed_out;
//   if(speed>1000)
//   {
//     speed=1000;
//   }else if(speed<0)
//   {
//     speed=0;
//   }
//   speed_out=speed;
//   return speed_out;
// }
// /**
//  * @brief 开环飞控
// */
// fp32 pitch_buf=0.1f;
// fp32 pitch_debuff=0.1f;
// fp32 roll_buff=0.1f;
// fp32 roll_debuff=0.1f;
// fp32 yaw_buff=0.1f;
// fp32 yaw_debuff=0.1f;
// void speed_to_control_vector(int16_t ch_throttle,int16_t ch_pitch,int16_t ch_roll ,int16_t yaw)
// {
//   int16_t standard_speed;
//   ch_throttle=ch_throttle+660;
//   standard_speed=ch_throttle+IDLING;
//   motor[0].speed_set=standard_speed;
//   motor[1].speed_set=standard_speed;
//   motor[2].speed_set=standard_speed;
//   motor[3].speed_set=standard_speed;
//   motor_cmd(motor[0].speed_set,motor[1].speed_set,motor[2].speed_set,motor[3].speed_set);
// }


void motor_no_force(void)//电机无力
{
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1000);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 1000);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 1000);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 1000);
}
void motor_init(void)
{
	osDelay(1000);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1000);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 1000);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 1000);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 1000);
}
void motor_cmd(int16_t motor_0,int16_t motor_1 ,int16_t motor_2,int16_t motor_3)
{

  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, motor_0+1000);
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, motor_1+1000);
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, motor_2+1000);
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, motor_3+1000);
}

void acro_init(void)
{
  pid_init(&Quad_Drone_x.acro_pitch_gyro_pid,100,0,60,300,300);
  pid_init(&Quad_Drone_x.acro_roll_gyro_pid,100,0,60,300,300);
  pid_init(&Quad_Drone_x.acro_yaw_gyro_pid,300,0,20,500,500);
}
void flight_feedback_update(void)//飞控反馈值更新
{
  Quad_Drone_x.acro_pitch_gyro_set=((float)rc_ctrl.rc.ch[1]/660.0f)*ACRO_PITCH_GYRO_LIMIT;
  Quad_Drone_x.acro_roll_gyro_set=((float)rc_ctrl.rc.ch[0]/660.0f)*ACRO_ROLL_GYRO_LIMIT;
  Quad_Drone_x.acro_yaw_gyro_set=((float)rc_ctrl.rc.ch[2]/660.0f)*ACRO_YAW_GYRO_LIMIT;
  Quad_Drone_x.acro_throttle_set=(((float)rc_ctrl.rc.ch[3]+660.0f)/1320)*(THROTTLE_LIMIT-IDLING);//油门打到最低端为-660所以要补偿回去
  Quad_Drone_x.pitch_gyro=bmi088_real_data.gyro[1];
  Quad_Drone_x.roll_gyro=bmi088_real_data.gyro[0];
  Quad_Drone_x.yaw_gyro=bmi088_real_data.gyro[2];
}
void acro_control(void)
{
  flight_feedback_update();
  Quad_Drone_x.acro_pitch_gyro_cpst=pid_calc(&Quad_Drone_x.acro_pitch_gyro_pid,
                                              Quad_Drone_x.acro_pitch_gyro_set,
                                              Quad_Drone_x.pitch_gyro);

  Quad_Drone_x.acro_roll_gyro_cpst=pid_calc(&Quad_Drone_x.acro_roll_gyro_pid,
                                            Quad_Drone_x.acro_roll_gyro_set,
                                            Quad_Drone_x.roll_gyro); 

  Quad_Drone_x.acro_yaw_gyro_cpst=pid_calc(&Quad_Drone_x.acro_yaw_gyro_pid,
                                              -Quad_Drone_x.acro_yaw_gyro_set,
                                              Quad_Drone_x.yaw_gyro);  

  Quad_Drone_x.motor[0].speed_set= Quad_Drone_x.acro_throttle_set
                                  -Quad_Drone_x.acro_pitch_gyro_cpst
                                  +Quad_Drone_x.acro_roll_gyro_cpst
                                  -Quad_Drone_x.acro_yaw_gyro_cpst;

  Quad_Drone_x.motor[1].speed_set= Quad_Drone_x.acro_throttle_set
                                  -Quad_Drone_x.acro_pitch_gyro_cpst
                                  -Quad_Drone_x.acro_roll_gyro_cpst
                                  +Quad_Drone_x.acro_yaw_gyro_cpst;  

  Quad_Drone_x.motor[2].speed_set= Quad_Drone_x.acro_throttle_set
                                  +Quad_Drone_x.acro_pitch_gyro_cpst
                                  +Quad_Drone_x.acro_roll_gyro_cpst
                                  +Quad_Drone_x.acro_yaw_gyro_cpst;
																	
  Quad_Drone_x.motor[3].speed_set=Quad_Drone_x.acro_throttle_set
                                  +Quad_Drone_x.acro_pitch_gyro_cpst
                                  -Quad_Drone_x.acro_roll_gyro_cpst
                                  -Quad_Drone_x.acro_yaw_gyro_cpst;

if(Quad_Drone_x.motor[0].speed_set<0) 
{
  Quad_Drone_x.motor[0].speed_set=0;
}else if(Quad_Drone_x.motor[0].speed_set>THROTTLE_LIMIT-IDLING)
{
  Quad_Drone_x.motor[0].speed_set=THROTTLE_LIMIT-IDLING;
}

if(Quad_Drone_x.motor[1].speed_set<0) 
{
  Quad_Drone_x.motor[1].speed_set=0;
}else if(Quad_Drone_x.motor[1].speed_set>THROTTLE_LIMIT-IDLING)
{
  Quad_Drone_x.motor[1].speed_set=THROTTLE_LIMIT-IDLING;
}                                 
if(Quad_Drone_x.motor[2].speed_set<0) 
{
  Quad_Drone_x.motor[2].speed_set=0;
}else if(Quad_Drone_x.motor[2].speed_set>THROTTLE_LIMIT-IDLING)
{
  Quad_Drone_x.motor[2].speed_set=THROTTLE_LIMIT-IDLING;
} 
if(Quad_Drone_x.motor[3].speed_set<0) 
{
  Quad_Drone_x.motor[3].speed_set=0;
}else if(Quad_Drone_x.motor[3].speed_set>THROTTLE_LIMIT-IDLING)
{
  Quad_Drone_x.motor[3].speed_set=THROTTLE_LIMIT-IDLING;
} 
 motor_cmd(Quad_Drone_x.motor[0].speed_set+IDLING,
            Quad_Drone_x.motor[1].speed_set+IDLING,
            Quad_Drone_x.motor[2].speed_set+IDLING,
            Quad_Drone_x.motor[3].speed_set+IDLING);
}

/**
 * @brief drone_level_init函数为自稳模式状态机初始化
 * 在level状态机下，飞行器控制模式为自稳模式,
 * 控制器采用双环PID控制，外环为角度环，内环为速度环。
 * @anchor Victor
 * @date 2023.6.23
*/
void drone_level_init(void)
{
  pid_init(&level_pitch_angle_pid,0,0,0,500,500);
  pid_init(&level_roll_angle_pid,0,0,0,500,500);
  pid_init(&level_pitch_gyro_pid,0.5,0.003,0.25,500,500);
  pid_init(&level_roll_gyro_pid,0.3,0.003,0.2,500,500);
  pid_init(&level_yaw_gyro_pid,500,0,0,500,500);
}
/**
 * @brief drone_level_control为自稳模式状态机
 * 在level状态机下，飞行器控制模式为自稳模式,
 * 控制器采用双环PID控制，外环为角度环，内环为速度环。
 * @anchor Victor
 * @date 2023.6.23
*/

void drone_level_control(void)
{
	pitch_angle_set=(rc_ctrl.rc.ch[1]*1.0f/660)*LEVEL_PITCH_ANGLE_LIMIT;
  roll_angle_set=(rc_ctrl.rc.ch[0]/660)*LEVEL_ROLL_ANGLE_LIMIT;
  yaw_gyro_set=((float)rc_ctrl.rc.ch[2]/660.0f)*LEVEL_YAW_GYRO_LIMIT;
  throttle_set=(((float)rc_ctrl.rc.ch[3]+660.0f)/1320)*LEVEL_THROTTLE_LIMIT;//油门打到最低端为-660所以要补偿回去
  // static float pitch_angle_fb =0.0f;
  // static float roll_angle_fb =0.0f;
  // static float pitch_gyro_fb =0.0f;
  // static float roll_gyro_fb =0.0f;
  // yaw_gyro_fb =0.0f;
 
    // pitch_angle_fb=pid_calc(&level_pitch_angle_pid,pitch_angle_set,INS_angle[1]);
    // pitch_gyro_fb=pid_calc(&level_pitch_gyro_pid,pitch_angle_fb,bmi088_real_data.gyro[1]);
    // roll_angle_fb=pid_calc(&level_roll_angle_pid,roll_angle_set,INS_angle[2]);
//    roll_gyro_fb=pid_calc(&level_roll_gyro_pid,roll_angle_fb,bmi088_real_data.gyro[0]);
    yaw_gyro_fb=pid_calc(&level_yaw_gyro_pid,-yaw_gyro_set,bmi088_real_data.gyro[2]);


  // motor[0].speed_set=throttle_set-pitch_gyro_fb+roll_gyro_fb+yaw_gyro_fb;
  // motor[1].speed_set=throttle_set-pitch_gyro_fb-roll_gyro_fb-yaw_gyro_fb;
  // motor[2].speed_set=throttle_set+pitch_gyro_fb-roll_gyro_fb+yaw_gyro_fb;
  // motor[3].speed_set=throttle_set+pitch_gyro_fb+roll_gyro_fb-yaw_gyro_fb;
  motor[0].speed_set=throttle_set-yaw_gyro_fb;
  motor[1].speed_set=throttle_set+yaw_gyro_fb;
  motor[2].speed_set=throttle_set+yaw_gyro_fb;
  motor[3].speed_set=throttle_set-yaw_gyro_fb;

 if(motor[0].speed_set<0)
 {
   motor[0].speed_set=0;
	 
 }else if(motor[0].speed_set>LEVEL_SPEED_LIMIT)
 {
   motor[0].speed_set=LEVEL_SPEED_LIMIT;
 }

 if(motor[1].speed_set<0)
 {
   motor[1].speed_set=0;
 }else if(motor[1].speed_set>LEVEL_SPEED_LIMIT)
 {
   motor[1].speed_set=LEVEL_SPEED_LIMIT;
 }
 if(motor[2].speed_set<0)
 {
   motor[2].speed_set=0;
 }else if(motor[2].speed_set>LEVEL_SPEED_LIMIT)
 {
   motor[2].speed_set=LEVEL_SPEED_LIMIT;
 }
 if(motor[3].speed_set<0)
 {
   motor[3].speed_set=0;
 }else if(motor[3].speed_set>LEVEL_SPEED_LIMIT)
 {
   motor[3].speed_set=LEVEL_SPEED_LIMIT;
 }





	
	
	
//  motor_cmd(motor[0].speed_set+IDLING,motor[1].speed_set+IDLING,motor[2].speed_set+IDLING,motor[3].speed_set+IDLING);

}
