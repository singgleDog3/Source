#ifndef _POWER_CONTROL_H
#define _POWER_CONTROL_H

void power_control_pin_on(void);
void power_control_pin_off(void);
void power_control_pin_init(void);

void HC244_enable(void);
void HC244_disable(void);
void HC244_ctrl_pin_init(void);

#endif

