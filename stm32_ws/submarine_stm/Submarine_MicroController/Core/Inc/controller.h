#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <sys/types.h>

struct actuator_command controller(com_data, state);
void propellor_control(struct actuator_command *act, struct goalCommand com);
void servo_control(struct actuator_command *act, struct goalCommand com);
void stepper_control(struct actuator_command *act, struct goalCommand com);

struct __attribute__((__packed__)) actuator_command {
  uint8_t rightPropThrust;
  uint8_t leftPropThrust; 
  float camVerticalDuty = (0.000392 * (float)camUpDown) + 0.025;
  float camHorizontalDuty = (0.000392 * (float)camLeftRight) + 0.025;
  struct stepper_instruction left_stepper;
  struct stepper_instruction right_stepper;
  struct state s;
};

struct __attribute__((__packed__)) stepper_instruction {
  uint8_t a_one;
  uint8_t a_two;
  uint8_t b_one;
  uint8_t b_two;
};


#endif
