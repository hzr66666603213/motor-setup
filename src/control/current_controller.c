#include "control/current_controller.h"

#include "foc/foc_math.h"

#include <math.h>

static bool cc_finite(float x)
{
    return isfinite(x) != 0;
}

static void cc_zero_output(CurrentControllerOutput *output)
{
    if (output == 0) {
        return;
    }

    output->vd_unsat_v = 0.0f;
    output->vq_unsat_v = 0.0f;
    output->vd_v = 0.0f;
    output->vq_v = 0.0f;
    output->v_alpha_v = 0.0f;
    output->v_beta_v = 0.0f;
    output->integrator_d_v = 0.0f;
    output->integrator_q_v = 0.0f;
    output->saturation_active = false;
    output->valid = false;
}

void current_controller_init(CurrentController *controller,
                             float kp,
                             float ki,
                             float max_voltage_v)
{
    if (controller == 0) {
        return;
    }

    controller->kp = kp;
    controller->ki = ki;
    controller->kaw = 0.0f;
    controller->integrator_d_v = 0.0f;
    controller->integrator_q_v = 0.0f;
    controller->integrator_limit_v = max_voltage_v;
    controller->max_voltage_v = max_voltage_v;
}

void current_controller_reset(CurrentController *controller)
{
    if (controller == 0) {
        return;
    }

    controller->integrator_d_v = 0.0f;
    controller->integrator_q_v = 0.0f;
}

void current_controller_set_gains(CurrentController *controller, float kp, float ki)
{
    if (controller == 0) {
        return;
    }

    controller->kp = kp;
    controller->ki = ki;
}

void current_controller_set_antiwindup(CurrentController *controller,
                                       float kaw,
                                       float integrator_limit_v)
{
    if (controller == 0) {
        return;
    }

    controller->kaw = (kaw > 0.0f) ? kaw : 0.0f;
    controller->integrator_limit_v =
        (integrator_limit_v > 0.0f) ? integrator_limit_v : controller->max_voltage_v;
    controller->integrator_d_v = foc_clamp(controller->integrator_d_v,
                                           -controller->integrator_limit_v,
                                           controller->integrator_limit_v);
    controller->integrator_q_v = foc_clamp(controller->integrator_q_v,
                                           -controller->integrator_limit_v,
                                           controller->integrator_limit_v);
}

void current_controller_clarke_vw(float iv_a,
                                  float iw_a,
                                  float *iu_a,
                                  float *i_alpha_a,
                                  float *i_beta_a)
{
    const float iu = -(iv_a + iw_a);
    if (iu_a != 0) {
        *iu_a = iu;
    }
    if (i_alpha_a != 0) {
        *i_alpha_a = iu;
    }
    if (i_beta_a != 0) {
        *i_beta_a = (iv_a - iw_a) * 0.57735026919f;
    }
}

void current_controller_park(float i_alpha_a,
                             float i_beta_a,
                             float theta_rad,
                             float *id_a,
                             float *iq_a)
{
    const float c = cosf(theta_rad);
    const float s = sinf(theta_rad);
    if (id_a != 0) {
        *id_a = (c * i_alpha_a) + (s * i_beta_a);
    }
    if (iq_a != 0) {
        *iq_a = (-s * i_alpha_a) + (c * i_beta_a);
    }
}

void current_controller_inverse_park(float vd_v,
                                     float vq_v,
                                     float theta_rad,
                                     float *v_alpha_v,
                                     float *v_beta_v)
{
    const float c = cosf(theta_rad);
    const float s = sinf(theta_rad);
    if (v_alpha_v != 0) {
        *v_alpha_v = (c * vd_v) - (s * vq_v);
    }
    if (v_beta_v != 0) {
        *v_beta_v = (s * vd_v) + (c * vq_v);
    }
}

float current_controller_ramp_toward(float current,
                                     float target,
                                     float rate_per_s,
                                     float dt_s)
{
    if (!cc_finite(current) || !cc_finite(target) ||
        !cc_finite(rate_per_s) || !cc_finite(dt_s) ||
        rate_per_s <= 0.0f || dt_s <= 0.0f) {
        return current;
    }

    const float step = rate_per_s * dt_s;
    if (current < target - step) {
        return current + step;
    }
    if (current > target + step) {
        return current - step;
    }
    return target;
}

bool current_controller_is_finite_output(const CurrentControllerOutput *output)
{
    return (output != 0) &&
           cc_finite(output->vd_unsat_v) &&
           cc_finite(output->vq_unsat_v) &&
           cc_finite(output->vd_v) &&
           cc_finite(output->vq_v) &&
           cc_finite(output->v_alpha_v) &&
           cc_finite(output->v_beta_v) &&
           cc_finite(output->integrator_d_v) &&
           cc_finite(output->integrator_q_v);
}

void current_controller_update_dq(CurrentController *controller,
                                  const CurrentControllerInput *input,
                                  CurrentControllerOutput *output)
{
    cc_zero_output(output);
    if (controller == 0 || input == 0 || output == 0) {
        if (controller != 0) {
            current_controller_reset(controller);
        }
        return;
    }

    const bool input_ok =
        cc_finite(input->id_ref_a) &&
        cc_finite(input->iq_ref_a) &&
        cc_finite(input->id_measured_a) &&
        cc_finite(input->iq_measured_a) &&
        cc_finite(input->theta_rad) &&
        cc_finite(input->vbus_v) &&
        cc_finite(input->dt_s) &&
        cc_finite(controller->kp) &&
        cc_finite(controller->ki) &&
        cc_finite(controller->kaw) &&
        cc_finite(controller->max_voltage_v) &&
        cc_finite(controller->integrator_limit_v) &&
        input->dt_s > 0.0f &&
        input->vbus_v > 1.0f &&
        controller->max_voltage_v > 0.0f &&
        controller->integrator_limit_v > 0.0f;

    if (!input->enable || input->fault_active || !input_ok) {
        current_controller_reset(controller);
        return;
    }

    const float id_error = input->id_ref_a - input->id_measured_a;
    const float iq_error = input->iq_ref_a - input->iq_measured_a;
    const float vbus_limit = 0.57735026919f * input->vbus_v;
    const float limit_v = (controller->max_voltage_v < vbus_limit) ?
                          controller->max_voltage_v :
                          vbus_limit;

    const float vd_unsat =
        (controller->kp * id_error) + controller->integrator_d_v;
    const float vq_unsat =
        (controller->kp * iq_error) + controller->integrator_q_v;
    float vd_sat = vd_unsat;
    float vq_sat = vq_unsat;
    foc_limit_voltage(&vd_sat, &vq_sat, limit_v);

    controller->integrator_d_v +=
        ((controller->ki * id_error) +
         (controller->kaw * (vd_sat - vd_unsat))) * input->dt_s;
    controller->integrator_q_v +=
        ((controller->ki * iq_error) +
         (controller->kaw * (vq_sat - vq_unsat))) * input->dt_s;
    controller->integrator_d_v = foc_clamp(controller->integrator_d_v,
                                           -controller->integrator_limit_v,
                                           controller->integrator_limit_v);
    controller->integrator_q_v = foc_clamp(controller->integrator_q_v,
                                           -controller->integrator_limit_v,
                                           controller->integrator_limit_v);

    output->vd_unsat_v = vd_unsat;
    output->vq_unsat_v = vq_unsat;
    output->vd_v = vd_sat;
    output->vq_v = vq_sat;
    current_controller_inverse_park(vd_sat,
                                    vq_sat,
                                    input->theta_rad,
                                    &output->v_alpha_v,
                                    &output->v_beta_v);
    output->integrator_d_v = controller->integrator_d_v;
    output->integrator_q_v = controller->integrator_q_v;
    output->saturation_active =
        (fabsf(vd_sat - vd_unsat) > 1.0e-6f) ||
        (fabsf(vq_sat - vq_unsat) > 1.0e-6f);
    output->valid = current_controller_is_finite_output(output);
    if (!output->valid) {
        current_controller_reset(controller);
        cc_zero_output(output);
    }
}

void current_controller_update(CurrentController *controller,
                               float id_target_a,
                               float iq_target_a,
                               float id_measured_a,
                               float iq_measured_a,
                               float vbus_v,
                               float dt_s,
                               float *vd_v,
                               float *vq_v)
{
    if (controller == 0 || vd_v == 0 || vq_v == 0 ||
        !cc_finite(id_target_a) || !cc_finite(iq_target_a) ||
        !cc_finite(id_measured_a) || !cc_finite(iq_measured_a) ||
        !cc_finite(vbus_v) || !cc_finite(dt_s) ||
        vbus_v <= 1.0f || dt_s <= 0.0f) {
        if (controller != 0) {
            current_controller_reset(controller);
        }
        if (vd_v != 0) {
            *vd_v = 0.0f;
        }
        if (vq_v != 0) {
            *vq_v = 0.0f;
        }
        return;
    }

    const float id_error_a = id_target_a - id_measured_a;
    const float iq_error_a = iq_target_a - iq_measured_a;
    const float bus_limited_v = 0.57735026919f * vbus_v;
    const float max_voltage_v =
        (controller->max_voltage_v < bus_limited_v) ?
        controller->max_voltage_v :
        bus_limited_v;

    controller->integrator_d_v += controller->ki * id_error_a * dt_s;
    controller->integrator_q_v += controller->ki * iq_error_a * dt_s;
    controller->integrator_d_v = foc_clamp(controller->integrator_d_v,
                                           -controller->integrator_limit_v,
                                           controller->integrator_limit_v);
    controller->integrator_q_v = foc_clamp(controller->integrator_q_v,
                                           -controller->integrator_limit_v,
                                           controller->integrator_limit_v);

    float vd_cmd_v = controller->kp * id_error_a + controller->integrator_d_v;
    float vq_cmd_v = controller->kp * iq_error_a + controller->integrator_q_v;
    const float vd_pre_limit_v = vd_cmd_v;
    const float vq_pre_limit_v = vq_cmd_v;

    foc_limit_voltage(&vd_cmd_v, &vq_cmd_v, max_voltage_v);

    controller->integrator_d_v += vd_cmd_v - vd_pre_limit_v;
    controller->integrator_q_v += vq_cmd_v - vq_pre_limit_v;
    controller->integrator_d_v = foc_clamp(controller->integrator_d_v,
                                           -controller->integrator_limit_v,
                                           controller->integrator_limit_v);
    controller->integrator_q_v = foc_clamp(controller->integrator_q_v,
                                           -controller->integrator_limit_v,
                                           controller->integrator_limit_v);

    if (vd_v != 0) {
        *vd_v = vd_cmd_v;
    }
    if (vq_v != 0) {
        *vq_v = vq_cmd_v;
    }
}

void current_controller_tune_from_rl(CurrentController *controller,
                                     float phase_resistance_ohm,
                                     float phase_inductance_h,
                                     float bandwidth_hz,
                                     float voltage_limit_v)
{
    if (controller == 0 ||
        phase_resistance_ohm <= 0.0f ||
        phase_inductance_h <= 0.0f ||
        bandwidth_hz <= 0.0f ||
        voltage_limit_v <= 0.0f) {
        return;
    }

    const float wc_rad_s = FOC_TWO_PI_F * bandwidth_hz;
    controller->kp = phase_inductance_h * wc_rad_s;
    controller->ki = phase_resistance_ohm * wc_rad_s;
    controller->kaw = wc_rad_s;
    controller->max_voltage_v = voltage_limit_v;
    controller->integrator_limit_v = voltage_limit_v;
    current_controller_reset(controller);
}
