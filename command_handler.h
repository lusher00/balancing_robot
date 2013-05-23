#ifndef _COMMAND_HANDLER_H_
#define _COMMAND_HANDLER_H_

#include "stdint.h"

extern void command_handler(void);
extern void command_handler_init(void);
extern void print_params(uint8_t uart);
extern void print_update(uint8_t uart);
extern void print_angle(uint8_t uart);
extern void print_debug(uint8_t uart);
extern void print_debug2(uint8_t uart);
extern void print_control_surfaces(uint8_t uart);

#endif
