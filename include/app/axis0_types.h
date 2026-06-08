#ifndef AXIS0_TYPES_H
#define AXIS0_TYPES_H

/*
 * axis0_types.h
 *
 * ODrive v3.6 + 2804 + MT6701 ABZ 学习框架的公共类型定义。
 * 第一阶段只启用 Axis0，Axis1 资源保留但默认不参与控制。
 * 所有物理量使用 SI 单位：
 * - 电流 A
 * - 电压 V
 * - 角度 rad
 * - 速度 rad/s
 * - 力矩 Nm
 * - 时间 s
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 控制模式说明：
 * - 该枚举描述“命令如何被解释”，不等于状态机是否已经允许输出。
 * - 即使 control_mode 是 TORQUE/VELOCITY/POSITION，只要 state 不是
 *   AXIS0_STATE_CLOSED_LOOP_CONTROL，20 kHz ISR 仍应输出安全 duty。
 * - 后台通信任务修改该字段时，真实固件中应和 input_* 一起做双缓冲提交，
 *   避免 ISR 读到模式和目标值不匹配的中间状态。
 */
typedef enum {
    AXIS0_CONTROL_MODE_IDLE = 0,     /* 空闲模式：电流目标为 0，PWM 不应输出有效力矩 */
    AXIS0_CONTROL_MODE_TORQUE,       /* 力矩模式：input_torque_nm / torque_constant 得到 iq 目标 */
    AXIS0_CONTROL_MODE_VELOCITY,     /* 速度模式：速度 PI 输出 iq 目标 */
    AXIS0_CONTROL_MODE_POSITION      /* 位置模式：位置 P 输出速度目标，再由速度 PI 输出 iq */
} Axis0ControlMode;

/*
 * Axis0 主状态机说明：
 * - 状态机负责“什么时候允许功率级上电”，ISR 负责“闭环状态下每周期怎么算”。
 * - 校准状态保持非阻塞：由 1 kHz/后台周期性调用 update，而不是 delay 等待。
 * - FAULT 是锁存状态；清故障前必须先确认 PWM/EN_GATE 已关闭，
 *   再由用户显式命令恢复到 IDLE 或重新校准。
 */
typedef enum {
    AXIS0_STATE_BOOT = 0,                    /* 上电初始化，功率级保持关闭 */
    AXIS0_STATE_IDLE,                        /* 空闲等待命令，PWM 和 DRV8301 默认关闭 */
    AXIS0_STATE_CURRENT_OFFSET_CALIBRATION,  /* 电流采样零偏校准 */
    AXIS0_STATE_MOTOR_CALIBRATION,           /* 相电阻和相电感校准 */
    AXIS0_STATE_ENCODER_CALIBRATION,         /* MT6701 ABZ 方向和电角度零位校准 */
    AXIS0_STATE_READY,                       /* 校准完成，允许用户请求闭环 */
    AXIS0_STATE_CLOSED_LOOP_CONTROL,         /* 闭环控制运行 */
    AXIS0_STATE_FAULT                        /* 故障状态，功率级强制关闭 */
} Axis0StateId;

/*
 * 电机参数和与电机直接相关的安全限幅。
 *
 * 调参建议：第一次接真实 2804 电机时，先只改 current_limit_a、
 * calibration_current_a 和 voltage_limit_v，保持它们非常保守；极对数、
 * 相电流比例和编码器方向确认后，再逐步提高速度/电流限制。
 */
typedef struct {
    uint8_t pole_pairs;              /* 极对数，无单位；2804 默认 7，但必须实测确认 */
    float phase_resistance_ohm;      /* 相电阻，ohm；初始为 0，校准后填入 */
    float phase_inductance_h;        /* 相电感，H；初始为 0，校准后填入 */
    float torque_constant_nm_per_a;  /* 转矩常数，Nm/A；未知时只能低风险调试 */
    float current_limit_a;           /* 默认电流限幅，A */
    float calibration_current_a;     /* 校准电流限幅，A */
    float voltage_limit_v;           /* 电压矢量限幅，V */
    float velocity_limit_rad_s;      /* 速度限幅，rad/s */
    bool position_limit_enabled;     /* 是否启用软件位置限位 */
    float position_min_rad;          /* 位置下限，rad */
    float position_max_rad;          /* 位置上限，rad */
} Axis0MotorConfig;

/*
 * MT6701 ABZ 增量编码器配置。
 *
 * PPR/CPR 容易混淆：PPR 是 A/B 单通道每机械圈的脉冲数，
 * CPR 是经过四倍频后的计数数，通常 CPR = PPR * 4。
 * 因为 ABZ 不是绝对角，上电后必须通过 index 或校准流程确定电角度零位。
 */
typedef struct {
    int32_t encoder_ppr;             /* ABZ 每转脉冲数 PPR，默认 1024 */
    int32_t encoder_cpr;             /* 四倍频计数 CPR，默认 4096 */
    int encoder_direction;           /* 方向，+1 或 -1 */
    float encoder_offset_rad;        /* 编码器零位偏移，rad */
    bool use_index_z;                /* 是否使用 Z/index；第一版允许 false */
} Axis0EncoderConfig;

/*
 * 控制周期和控制器增益。
 *
 * 这里保存的是“配置目标值”，实际定时器是否真的运行在这些频率，
 * 由 board/HAL 初始化负责。调试时应同时用示波器或计数器确认实际 ISR 周期，
 * 否则 PI 积分项会因为 dt_s 不匹配而表现异常。
 */
typedef struct {
    float pwm_frequency_hz;          /* PWM 频率，Hz，默认 20000 */
    float current_loop_hz;           /* 电流环频率，Hz，默认 20000 */
    float velocity_loop_hz;          /* 速度环频率，Hz，默认 1000 */
    float position_loop_hz;          /* 位置环频率，Hz，默认 500 */
    float current_kp;                /* 电流环比例增益，V/A */
    float current_ki;                /* 电流环积分增益，V/(A*s) */
    float velocity_kp;               /* 速度环比例增益，A/(rad/s) */
    float velocity_ki;               /* 速度环积分增益，A/rad */
    float position_kp;               /* 位置环比例增益，(rad/s)/rad */
} Axis0ControlConfig;

/*
 * 保护阈值配置。
 *
 * 快速保护通常在 ISR 内检查（例如相电流、vbus、DRV nFAULT），
 * 慢速保护可在 100 Hz/1 kHz 任务中检查（例如温度、通信超时）。
 * 阈值应留有硬件测量误差余量，不要贴着电源或电机极限设置。
 */
typedef struct {
    float vbus_min_v;                /* 母线欠压阈值，V */
    float vbus_max_v;                /* 母线过压阈值，V */
    float phase_overcurrent_a;       /* 相电流过流阈值，A */
    float overtemperature_c;         /* 过温阈值，degC */
    bool encoder_error_enable;       /* 编码器错误保护使能 */
    bool drv_fault_enable;           /* DRV8301 nFAULT 保护使能 */
} Axis0ProtectionConfig;

typedef struct {
    Axis0MotorConfig motor;          /* 电机和安全限幅 */
    Axis0EncoderConfig encoder;      /* MT6701 ABZ 配置 */
    Axis0ControlConfig control;      /* 控制频率和控制器增益 */
    Axis0ProtectionConfig protection;/* 保护阈值 */
    Axis0StateId startup_state;      /* 默认启动状态，必须为 IDLE/BOOT，不自动闭环 */
} Axis0Config;

typedef struct {
    float ia_a;                      /* U/A 相电流，A，由 ISR 更新 */
    float ib_a;                      /* V/B 相电流，A，由 ISR 更新 */
    float ic_a;                      /* W/C 相电流，A，由 ISR 更新 */
    float i_alpha_a;                 /* alpha 轴电流，A，由 ISR 更新 */
    float i_beta_a;                  /* beta 轴电流，A，由 ISR 更新 */
    float id_a;                      /* d 轴电流，A，由 ISR 更新 */
    float iq_a;                      /* q 轴电流，A，由 ISR 更新 */
    float vd_v;                      /* d 轴电压指令，V，由 ISR 更新 */
    float vq_v;                      /* q 轴电压指令，V，由 ISR 更新 */
    float v_alpha_v;                 /* alpha 轴电压指令，V，由 ISR 更新 */
    float v_beta_v;                  /* beta 轴电压指令，V，由 ISR 更新 */
    float duty_a;                    /* A/U 相 duty，0..1，由 ISR 更新 */
    float duty_b;                    /* B/V 相 duty，0..1，由 ISR 更新 */
    float duty_c;                    /* C/W 相 duty，0..1，由 ISR 更新 */
    float vbus_v;                    /* 母线电压，V，由 ISR/慢速 ADC 更新 */
    float mechanical_angle_rad;      /* 机械角，rad，由编码器模块更新 */
    float electrical_angle_rad;      /* 电角度，rad，由 ISR 计算 */
    float velocity_rad_s;            /* 机械速度，rad/s，由编码器模块更新 */
} Axis0RealtimeState;

typedef struct {
    float input_torque_nm;           /* 输入力矩，Nm，由后台命令写入 */
    float input_velocity_rad_s;      /* 输入速度，rad/s，由后台命令写入 */
    float input_position_rad;        /* 输入位置，rad，由后台命令写入 */
    float iq_target_a;               /* 外环输出 iq 目标，A，由 1kHz/500Hz 任务写，ISR 读 */
    float id_target_a;               /* d 轴目标，A，第一版通常为 0 */
    Axis0ControlMode control_mode;   /* 控制模式，由后台命令写入 */
} Axis0Command;

/*
 * Axis0 顶层上下文。
 *
 * 并发约定：
 * - rt：高速 ISR 写，后台只读或快照读取；
 * - cmd/requested_state/config：后台写，ISR 只读必要字段；
 * - fault_flags：ISR 和保护路径都可能置位，清零只能由受控的清故障流程执行。
 *
 * 这个学习框架先用单个结构体表达数据流，移植到真实固件时建议把
 * “命令输入”和“实时观测”拆成双缓冲/环形日志，减少临界区时间。
 */
typedef struct {
    Axis0Config config;              /* 运行配置 */
    Axis0RealtimeState rt;           /* 实时状态，主要 ISR 更新 */
    Axis0Command cmd;                /* 命令双缓冲的简化骨架 */
    Axis0StateId state;              /* 当前状态 */
    Axis0StateId requested_state;    /* 请求状态 */
    uint32_t fault_flags;            /* fault.h 中定义的 bitmask */
    bool current_offset_valid;       /* 电流零偏校准完成 */
    bool motor_calibrated;           /* 电阻/电感校准完成 */
    bool encoder_calibrated;         /* 编码器方向/offset 校准完成 */
} Axis0Context;

#ifdef __cplusplus
}
#endif

#endif /* AXIS0_TYPES_H */
