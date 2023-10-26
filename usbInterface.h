#ifndef USB_INTERFACE_H
#define USB_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include "pico/stdlib.h"


// Function prototypes

bool usbInterfaceSendBuffer(void* buffer, uint32_t length);
void usbInterfaceRegisterCallback(void (*callback)(void* buffer, uint32_t length));

#ifdef __cplusplus
}
#endif

#endif // USB_INTERFACE_H
