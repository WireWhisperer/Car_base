#ifndef TRACE_H
#define TRACE_H

#include "gray_with_MCU.h"
#include "Motor.h"
#include "clock.h"

void Rotate(float speed, float yaw_target);
void Patrol_Trace(Track_Clock spin_dir,float left_speed, float right_speed, float speed_add, float yaw_current);

#endif