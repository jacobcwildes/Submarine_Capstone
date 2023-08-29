#ifndef __ACTUATOR_H
#define __ACTUATOR_H

void MTR_DRV_INIT(uint8_t currentValue, uint8_t decay, uint8_t reset, uint8_t sleep);

void transmitData(void);

void updateProps(void);
void updateServos(void);

void leftBalastOpenStep(void);
void leftBalastCloseStep(void);
void rightBalastOpenStep(void);
void rightBalastCloseStep(void);
void updateSteppers(void);

#endif
