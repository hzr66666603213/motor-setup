#ifndef CURRENT_CONTROLLER_H
#define CURRENT_CONTROLLER_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;                 /* V/A */
    float ki;                 /* V/(A*s) */
    float kaw;                /* 1/s, back-calculation anti-windup */
    float integrator_d_v;     /* V */
    float integrator_q_v;     /* V */
    float integrator_limit_v; /* V */
    float max_voltage_v;      /* circular vector limit, V */
} CurrentController;

typedef struct {
    float id_ref_a;
    float iq_ref_a;
    float id_measured_a;
    float iq_measured_a;
    float theta_rad;
    float vbus_v;
    float dt_s;
    bool enable;
    bool fault_active;
} CurrentControllerInput;

typedef struct {
    float vd_unsat_v;
    float vq_unsat_v;
    float vd_v;
    float vq_v;
    float v_alpha_v;
    float v_beta_v;
    float integrator_d_v;
    float integrator_q_v;
    bool saturation_active;
    bool valid;
} CurrentControllerOutput;

void current_controller_init(CurrentController *controller,
                             float kp,
                             float ki,
                             float max_voltage_v);
void current_controller_reset(CurrentController *controller);
void current_controller_set_gains(CurrentController *controller, float kp, float ki);
void current_controller_set_antiwindup(CurrentController *controller,
                                       float kaw,
                                       float integrator_limit_v);

void current_controller_update(CurrentController *controller,
                               float id_target_a,
                               float iq_target_a,
                               float id_measured_a,
                               float iq_measured_a,
                               float vbus_v,
                               float dt_s,
                               float *vd_v,
                               float *vq_v);

void current_controller_update_dq(CurrentController *controller,
                                  const CurrentControllerInput *input,
                                  CurrentControllerOutput *output);

void current_controller_tune_from_rl(CurrentController *controller,
                                     float phase_resistance_ohm,
                                     float phase_inductance_h,
                                     float bandwidth_hz,
                                     float voltage_limit_v);

void current_controller_clarke_vw(float iv_a,
                                  float iw_a,
                                  float *iu_a,
                                  float *i_alpha_a,
                                  float *i_beta_a);
void current_controller_park(float i_alpha_a,
                             float i_beta_a,
                             float theta_rad,
                             float *id_a,
                             float *iq_a);
void current_controller_inverse_park(float vd_v,
                                     float vq_v,
                                     float theta_rad,
                                     float *v_alpha_v,
                                     float *v_beta_v);
float current_controller_ramp_toward(float current,
                                     float target,
                                     float rate_per_s,
                                     float dt_s);
bool current_controller_is_finite_output(const CurrentControllerOutput *output);

#ifdef __cplusplus
}
#endif

#endif /* CURRENT_CONTROLLER_H */
