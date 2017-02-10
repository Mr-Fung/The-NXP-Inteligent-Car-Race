#ifndef _SERVO_H
#define _SERVO_H

void slope_left(int bottom,int top);
void slope_right(int bottom,int top);
void servo(void);
void slope(void);
void servo_delay(int ms);
void PIDangle_init(void); 
float PID_angle(float angle);

#endif