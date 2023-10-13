#ifndef COMINTERFACE_H
#define COMINTERFACE_H

#include <stdint.h>

int comInterfaceInit(void);
int comInterfaceAddSample(uint32_t sample);
int comInterfaceSetHeader(uint16_t samplePeriod, uint16_t channels);

#endif // COMINTERFACE_H
