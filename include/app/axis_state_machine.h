#ifndef AXIS0_STATE_MACHINE_H
#define AXIS0_STATE_MACHINE_H

/*
 * axis_state_machine.h
 *
 * ODrive v3.6 Axis0 状态机。
 * 1kHz 更新外环/校准推进；后台更新通信和慢速保护。
 */

#include "app/axis0_types.h"
#include "app/calibration.h"
#include "control/position_controller.h"
#include "control/velocity_controller.h"
#include "drivers/current_sensor.h"
#include "drivers/drv8301.h"
#include "drivers/encoder_mt6701_abz.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    Axis0CalibrationContext calibration;
    VelocityController *velocity_controller;
    PositionController *position_controller;
    CurrentSensorConfig *current_sensor;
    EncoderMt6701AbzState *encoder;
    Drv8301 *drv0;               /* M0/Axis0 DRV8301 */
    Drv8301 *drv1;               /* M1/Axis1 DRV8301；共享 EN_GATE/nFAULT，必须处理 */
} Axis0StateMachineContext;

void axis_request_state(Axis0Context *axis, Axis0StateId requested_state);
void axis_update_1khz(Axis0Context *axis, Axis0StateMachineContext *sm, float dt_s);
void axis_update_background(Axis0Context *axis, Axis0StateMachineContext *sm);
bool axis_is_ready_for_closed_loop(const Axis0Context *axis);

#ifdef __cplusplus
}
#endif

#endif /* AXIS0_STATE_MACHINE_H */
