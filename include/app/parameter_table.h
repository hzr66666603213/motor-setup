#ifndef AXIS0_PARAMETER_TABLE_H
#define AXIS0_PARAMETER_TABLE_H

/*
 * parameter_table.h
 *
 * Axis0 调试参数表。
 * 用于 USB CDC/UART 文本命令的 get/set 范围检查和字段映射。
 */

#include <stdbool.h>
#include "app/axis0_types.h"
#include "drivers/encoder_mt6701_abz.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    Axis0Context *axis;
    EncoderMt6701AbzState *encoder;
} Axis0ParameterContext;

bool axis0_param_get(Axis0ParameterContext *ctx, const char *name, float *value);
bool axis0_param_set(Axis0ParameterContext *ctx, const char *name, const char *value_text);
const char *axis0_state_to_string(Axis0StateId state);
bool axis0_parse_state(const char *text, Axis0StateId *state);
bool axis0_parse_control_mode(const char *text, Axis0ControlMode *mode);

#ifdef __cplusplus
}
#endif

#endif /* AXIS0_PARAMETER_TABLE_H */
