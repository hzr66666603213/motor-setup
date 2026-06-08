#ifndef AXIS0_DEFAULT_CONFIG_H
#define AXIS0_DEFAULT_CONFIG_H

/*
 * axis0_default_config.h
 *
 * ODrive v3.6 + 2804 外转子无刷 + MT6701 ABZ 的低风险初始参数。
 * 这些参数用于第一次低压、限流、空载学习调试，不适合直接带载运行。
 */

#include "app/axis0_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AXIS0_DEFAULT_POLE_PAIRS              7u
/* 重要：2804 不同厂家/绕组版本极对数可能不同，必须实测确认后修改。 */

#define AXIS0_DEFAULT_CURRENT_LIMIT_A         1.0f
#define AXIS0_DEFAULT_CALIB_CURRENT_A         0.5f
#define AXIS0_DEFAULT_VOLTAGE_LIMIT_V         3.0f
#define AXIS0_DEFAULT_VELOCITY_LIMIT_RAD_S    20.0f

#define AXIS0_DEFAULT_MT6701_PPR              1024
#define AXIS0_DEFAULT_MT6701_CPR              4096

#define AXIS0_DEFAULT_PWM_HZ                  20000.0f
#define AXIS0_DEFAULT_CURRENT_LOOP_HZ         20000.0f
#define AXIS0_DEFAULT_VELOCITY_LOOP_HZ        1000.0f
#define AXIS0_DEFAULT_POSITION_LOOP_HZ        500.0f

/* 保守电流环参数。实际应根据电机 R/L、采样增益和 PWM 频率重新整定。 */
#define AXIS0_DEFAULT_CURRENT_KP              0.05f
#define AXIS0_DEFAULT_CURRENT_KI              100.0f
#define AXIS0_DEFAULT_VELOCITY_KP             0.02f
#define AXIS0_DEFAULT_VELOCITY_KI             0.2f
#define AXIS0_DEFAULT_POSITION_KP             2.0f

#define AXIS0_DEFAULT_VBUS_MIN_V              8.0f
#define AXIS0_DEFAULT_VBUS_MAX_V              28.0f
#define AXIS0_DEFAULT_OVERCURRENT_A           2.0f
#define AXIS0_DEFAULT_OVERTEMP_C              70.0f

/* 返回一份默认配置；调用者可复制到 Axis0Context.config 后再按实测结果修改。 */
Axis0Config axis0_default_config_make(void);

#ifdef __cplusplus
}
#endif

#endif /* AXIS0_DEFAULT_CONFIG_H */
