#ifndef TEST_TASK_H
#define TEST_TASK_H
#include "struct_typedef.h"
enum  //Flight mode
{
  LEVEL,//自稳模式
  HORIZEN,//半自稳模式
  ACRO//手动模式
};
extern uint8_t arm;
extern uint8_t flight_mode;

#endif
