#ifndef motors_H
#define motors_H

#include "stm32f7xx.h"                  // Device header


typedef struct //stepper
{
  int current_pos_step;
  int current_speed_sps; // step per second
  int target_pos_step;
  int target_speed_sps;
  int max_speed_sps;
  int max_accel_spss;
  int gear_ratio;
  int steps_per_turn;
  _Bool dir;
  _Bool enable;
} stepper;

typedef struct //servo
{
  int current_pos_tic;
  int current_speed_tps; // tic per second, compare
  int target_pos_tic;
  int target_speed_tps;  
  int max_speed_tps;
  int max_accel_tpss;
} servo;

typedef struct //join
{
  int current_pos_rad;
  int current_speed_rps;
  int max_angle_rad;
  int min_angle_rad;
} join;


//stepper object
extern stepper stepper1;
extern stepper stepper2;
extern stepper stepper3;
extern stepper stepper4;

//servo object
extern servo servo1;
extern servo servo2;

//join object
extern join join1;
extern join join2;
extern join join3;
extern join join4;
extern join join5;
extern join join6;

void init_servo_param(void);
void init_stepper_param(void);
void init_join_param(void);


#endif
