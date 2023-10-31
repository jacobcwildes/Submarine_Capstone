#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <sys/types.h>
#include "planner.h"
#include "state_estimation.h"



struct __attribute__((__packed__)) stepper_instruction {
  uint8_t a_one;
  uint8_t a_two;
  uint8_t b_one;
  uint8_t b_two;
};

struct __attribute__((__packed__)) actuator_command {
  uint8_t rightPropThrust;
  uint8_t leftPropThrust; 
  float camVerticalDuty;
  float camHorizontalDuty;
  struct stepper_instruction left_stepper;
  struct stepper_instruction right_stepper;
  struct state s;
  struct goalCommand c;
};

struct actuator_command controller(struct goalCommand com_data, struct state s);
void propellor_control(struct actuator_command *act, struct goalCommand com);
void servo_control(struct actuator_command *act, struct goalCommand com);
void stepper_control(struct actuator_command *act, struct goalCommand com);
void bouyancyUp(struct actuator_command act);
void bouyancyDown(struct actuator_command act);
void rotateCCW(struct actuator_command act);
void rotateCW(struct actuator_command act);

#endif
