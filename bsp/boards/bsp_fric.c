#include "bsp_fric.h"
#include "main.h"
extern TIM_HandleTypeDef htim1;

void Tim_Init (void)
{
    HAL_TIM_Base_Start(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

void fric_off(void)
{
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, FRIC_OFF);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, FRIC_OFF);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, FRIC_OFF);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, FRIC_OFF);
}
void fric_on(uint16_t cmd)
{
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, cmd);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, cmd);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, cmd);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, cmd);

}


