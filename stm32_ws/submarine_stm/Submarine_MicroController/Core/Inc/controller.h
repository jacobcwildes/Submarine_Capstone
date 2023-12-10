#ifndef __CONTROLLER_H
#define __CONTROLLER_H


#include <sys/types.h>
#include <stdlib.h>
#include <math.h>
#include "planner.h"
#include "perception.h"

struct __attribute__((__packed__)) stepper_instruction {
  uint8_t a_one;
  uint8_t a_two;
  uint8_t b_one;
  uint8_t b_two;
};

struct __attribute__((__packed__)) actuator_command {
  float rightPropThrust;
  float leftPropThrust; 
  float camVerticalDuty;
  float camHorizontalDuty;
  struct stepper_instruction left_stepper;
  struct stepper_instruction right_stepper;
  struct envData in;
  struct goalCommand com;
  int left_current;
  int right_current;
};

struct actuator_command controller(struct goalCommand com_data, struct envData in);
void propellor_control(struct actuator_command *act);
void servo_control(struct actuator_command *act);
void stepper_control(struct actuator_command *act);



#endif
