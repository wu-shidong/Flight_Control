#ifndef BSP_FRIC_H
#define BSP_FRIC_H
#include "struct_typedef.h"

#define FRIC_UP 1400
#define FRIC_DOWN 1320
#define FRIC_OFF 1000

extern void fric_off(void);
extern void fric_on(uint16_t cmd);
extern void Tim_Init (void);
#endif
