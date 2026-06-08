#ifndef SIMULINK_FOC_ENTRY_H
#define SIMULINK_FOC_ENTRY_H

int foc_sim_step_wrapper(double ia_a,
                         double ib_a,
                         double ic_a,
                         double mechanical_angle_rad,
                         double mechanical_velocity_rad_s,
                         double vbus_v,
                         double id_target_a,
                         double iq_target_a,
                         double dt_s,
                         double pole_pairs_double,
                         double encoder_offset_rad,
                         double *id_a,
                         double *iq_a,
                         double *vd_v,
                         double *vq_v,
                         double *v_alpha_v,
                         double *v_beta_v,
                         double *duty_u,
                         double *duty_v,
                         double *duty_w);

#endif
