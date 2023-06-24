#ifndef __DEFAULT_H
#define __DEFAULT_H
#include "struct_typedef.h"
#include "user_pid.h"
#include "remote_control.h"
#include "INS_task.h"
#define IDLING 270
#define THROTTLE_DEADBAND 60
#define THROTTLE_LIMIT 1200

#define LEVEL_PITCH_ANGLE_LIMIT 0.5f 
#define LEVEL_ROLL_ANGLE_LIMIT 0.5f
#define LEVEL_YAW_GYRO_LIMIT 0.5f
#define LEVEL_THROTTLE_LIMIT 900
#define LEVEL_SPEED_LIMIT 1000

#define ACRO_PITCH_GYRO_LIMIT 2.0f
#define ACRO_ROLL_GYRO_LIMIT 2.0f
#define ACRO_YAW_GYRO_LIMIT 4.0f

#define LEVEL_PITCH_ANGLE_KP 2.5f
#define LEVEL_PITCH_ANGLE_KI 0.0f
#define LEVEL_PITCH_ANGLE_KD 0.0f
#define LEVEL_PITCH_ANGLE_LIM_I 100f
#define LEVEL_PITCH_ANGLE_LIM_OUT 500f

typedef struct
{
  fp32 speed_set;
  int16_t give_current;
} motor_t;

typedef enum
{
  FLIGHT_MODE_ANGLE,
  FLIGHT_MODE_ACRO,
  FLIGHT_MODE_HORIZEN,
  FLIGHT_MODE_DETECT
} flight_mode_e ;
typedef enum
{
  ARMED,
  DISARMED,
  TEST
}arm_mode_e;


typedef struct 
{
  /* data */
  const RC_ctrl_t *drone_RC; //ң����ָ��, the point to remote control

  flight_mode_e flight_mode; //state machine. ���п���ģʽ״̬��
  flight_mode_e last_flight_mode;//last state machine. ��һ�η��п���ģʽ״̬��
  motor_t motor[4];//�ĸ����
  fp32 pitch_gyro;//pitch���ٶ�ʵ��ֵ
  fp32 roll_gyro;//roll���ٶ�ʵ��ֵ
  fp32 yaw_gyro;//yaw���ٶ�ʵ��ֵ
  arm_mode_e arm;//����״̬
  pid_struct_t acro_roll_gyro_pid;//�ֶ�ģʽ��roll���ٶȻ�pid 
  pid_struct_t acro_pitch_gyro_pid;//�ֶ�ģʽ��pitch���ٶȻ�pid 
  pid_struct_t acro_yaw_gyro_pid;//�ֶ�ģʽ��yaw���ٶȻ�pid 
  fp32 acro_pitch_gyro_set;//�ֶ�ģʽ��pitch���ٶ�Ŀ��ֵ
  fp32 acro_roll_gyro_set;//�ֶ�ģʽ��roll���ٶ�Ŀ��ֵ
  fp32 acro_yaw_gyro_set;//�ֶ�ģʽ��yaw���ٶ�Ŀ��ֵ
  fp32 acro_throttle_set;//�ֶ�ģʽ������Ŀ��ֵ
  fp32 acro_pitch_gyro_cpst;//�ֶ�ģʽ��pitch���ٶȲ���ֵ
  fp32 acro_roll_gyro_cpst;//�ֶ�ģʽ��roll���ٶȲ���ֵ
  fp32 acro_yaw_gyro_cpst;//�ֶ�ģʽ��yaw���ٶȲ���ֵ
}drone_t;

extern drone_t Quad_Drone_x;//����ṹ���������˻�
extern void DefaultTask(void const * argument);




#endif
