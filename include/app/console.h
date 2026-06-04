#ifndef AXIS0_CONSOLE_H
#define AXIS0_CONSOLE_H

/*
 * console.h
 *
 * USB CDC / UART 文本调试接口。
 * 后台任务调用，禁止在 PWM ISR 中调用。
 */

#include <stddef.h>
#include "app/axis_state_machine.h"
#include "app/parameter_table.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    Axis0ParameterContext params;
    Axis0Context *axis;
    char line[128];
    size_t len;
} Axis0Console;

void console_init(Axis0Console *console, Axis0Context *axis, EncoderMt6701AbzState *encoder);
void console_poll(Axis0Console *console);
bool console_handle_line(Axis0Console *console, const char *line, char *response, size_t response_len);

#ifdef __cplusplus
}
#endif

#endif /* AXIS0_CONSOLE_H */
