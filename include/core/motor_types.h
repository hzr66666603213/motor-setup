#ifndef MOTOR_TYPES_H
#define MOTOR_TYPES_H

/*
 * motor_types.h
 *
 * 本文件定义整个单轴 FOC 固件框架共享的核心数据结构。
 * 设计原则：
 * 1. 所有物理量字段名都带单位后缀，注释再次标明单位。
 * 2. 结构体只保存数据，不包含硬件外设句柄，保证算法层可移植。
 * 3. 这些结构体会被 20 kHz ISR、1 kHz 控制任务和后台通信任务共同访问；
 *    最终工程中跨上下文写读的字段应使用双缓冲、临界区或 volatile 对象保护。
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CONTROL_MODE_TORQUE = 0,     /* 力矩模式：输入力矩 Nm，换算为 iq 目标电流 */
    CONTROL_MODE_VELOCITY,       /* 速度模式：速度 PI 输出 iq 目标电流 */
    CONTROL_MODE_POSITION,       /* 位置模式：位置 P 输出速度目标，再进入速度 PI */
    CONTROL_MODE_TRAJECTORY      /* 轨迹模式：预留给后续轨迹规划器 */
} ControlMode;

typedef enum {
    AXIS_STATE_BOOT = 0,                       /* 上电初始化，输出保持安全 */
    AXIS_STATE_IDLE,                           /* 空闲，PWM 关闭，gate driver 关闭或安全保持 */
    AXIS_STATE_MOTOR_CALIBRATION,              /* 电流零偏、相电阻、相电感校准 */
    AXIS_STATE_ENCODER_OFFSET_CALIBRATION,     /* 编码器零位和方向校准 */
    AXIS_STATE_CLOSED_LOOP_CONTROL,            /* 闭环控制状态，允许电流环输出 PWM */
    AXIS_STATE_FAULT                           /* 故障状态，立即关 PWM 和 EN_GATE */
} AxisStateId;

typedef enum {
    ENCODER_TYPE_SPI = 0,  /* SPI 磁编码器，例如 AS5047/MA732 等 */
    ENCODER_TYPE_ABI,      /* ABI 增量编码器，需要计数器和 index 处理 */
    ENCODER_TYPE_HALL      /* Hall 传感器，低分辨率，常用于启动或简易 BLDC */
} EncoderType;

typedef enum {
    ENCODER_DIR_UNKNOWN = 0,  /* 未校准方向 */
    ENCODER_DIR_POSITIVE = 1, /* 开环正转时，编码器角度正向增加 */
    ENCODER_DIR_NEGATIVE = -1 /* 开环正转时，编码器角度反向变化 */
} EncoderDirection;

typedef enum {
    FAULT_NONE                    = 0u,        /* 无故障 */
    FAULT_BUS_UNDERVOLTAGE        = 1u << 0,  /* 母线欠压，通常禁止闭环使能 */
    FAULT_BUS_OVERVOLTAGE         = 1u << 1,  /* 母线过压，可能来自再生制动或电源异常 */
    FAULT_PHASE_OVERCURRENT       = 1u << 2,  /* 相电流超过快速保护阈值 */
    FAULT_MOS_OVERTEMPERATURE     = 1u << 3,  /* MOS/功率板温度过高 */
    FAULT_MOTOR_OVERTEMPERATURE   = 1u << 4,  /* 电机绕组或壳体温度过高 */
    FAULT_ENCODER_ERROR           = 1u << 5,  /* 编码器数据错误或校验失败 */
    FAULT_ENCODER_NO_RESPONSE     = 1u << 6,  /* 编码器无响应 */
    FAULT_ENCODER_ANGLE_JUMP      = 1u << 7,  /* 编码器角度跳变异常 */
    FAULT_GATE_DRIVER             = 1u << 8,  /* 栅极驱动 nFAULT 有效 */
    FAULT_ADC_ERROR               = 1u << 9,  /* ADC 采样无效、溢出或同步异常 */
    FAULT_PWM_ERROR               = 1u << 10, /* PWM 输出异常，预留给后端自检 */
    FAULT_CONTROL_SATURATION      = 1u << 11, /* 控制输出长时间饱和 */
    FAULT_CALIBRATION_FAILED      = 1u << 12  /* 校准失败或校准结果不可信 */
} FaultFlags;

typedef struct {
    /* 电机静态参数和安全限幅；通常来自出厂标定、校准流程或参数表写入。 */
    uint8_t pole_pairs;              /* 极对数，无单位 */
    float phase_resistance_ohm;      /* 相电阻，默认指相对中性点电阻，ohm */
    float phase_inductance_h;        /* 相电感，默认指相对中性点电感，H */
    float torque_constant_nm_per_a;  /* q 轴转矩常数，Nm/A */
    float current_limit_a;           /* 电流限幅，A */
    float velocity_limit_rad_s;      /* 机械速度限幅，rad/s */
    float position_min_rad;          /* 软件位置下限，rad */
    float position_max_rad;          /* 软件位置上限，rad */
} MotorConfig;

typedef struct {
    /* 实时状态；20 kHz ISR 和慢速任务都会更新其中不同字段。 */
    float mechanical_angle_rad;      /* 转子机械角，rad */
    float electrical_angle_rad;      /* 转子电角度，rad */
    float mechanical_velocity_rad_s; /* 转子机械速度，rad/s */
    float id_a;                      /* d 轴电流，A */
    float iq_a;                      /* q 轴电流，A */
    float bus_voltage_v;             /* 直流母线电压，V */
    float mos_temperature_c;         /* MOSFET/驱动板温度，degC */
    float motor_temperature_c;       /* 电机绕组或壳体温度，degC */
} MotorState;

typedef struct {
    /* FOC 与采样相关配置；这些值直接影响 20 kHz 电流环。 */
    float pwm_period_s;              /* PWM 周期，s */
    float current_kp;                /* 电流 PI 比例增益，V/A */
    float current_ki;                /* 电流 PI 积分增益，V/(A*s) */
    float max_voltage_v;             /* 电压矢量最大幅值，V */
    float current_sample_gain_a_per_count; /* 电流采样比例，A/count */
    float current_offset_u_count;    /* U 相 ADC 零偏，count */
    float current_offset_v_count;    /* V 相 ADC 零偏，count */
    float current_offset_w_count;    /* W 相 ADC 零偏，count */
} FocConfig;

typedef struct {
    /* FOC 中间量；主要由 20 kHz PWM ISR 读写，可用于调试观测。 */
    float ia_a;          /* U 相电流，A */
    float ib_a;          /* V 相电流，A */
    float ic_a;          /* W 相电流，A */
    float i_alpha_a;     /* alpha 轴电流，A */
    float i_beta_a;      /* beta 轴电流，A */
    float id_a;          /* d 轴电流，A */
    float iq_a;          /* q 轴电流，A */
    float vd_v;          /* d 轴电压指令，V */
    float vq_v;          /* q 轴电压指令，V */
    float v_alpha_v;     /* alpha 轴电压指令，V */
    float v_beta_v;      /* beta 轴电压指令，V */
    float duty_u;        /* U 相 PWM 占空比，0..1 */
    float duty_v;        /* V 相 PWM 占空比，0..1 */
    float duty_w;        /* W 相 PWM 占空比，0..1 */
} FocState;

typedef struct {
    /* 编码器状态；不同传感器后端应统一转换到机械角 rad 和速度 rad/s。 */
    EncoderType type;                /* 编码器类型 */
    uint32_t raw_count;              /* 原始编码器计数，count */
    float raw_angle_rad;             /* 原始传感器角度，rad */
    float mechanical_angle_rad;      /* 修正方向和零偏后的机械角，rad */
    float electrical_angle_rad;      /* 电角度，rad */
    float velocity_rad_s;            /* 机械速度，rad/s */
    EncoderDirection direction;      /* 编码器方向符号 */
    float offset_rad;                /* 编码器零偏，rad */
    bool is_ready;                   /* 编码器已初始化并可用于闭环 */
    bool has_error;                  /* 编码器后端报告错误 */
} EncoderState;

typedef struct {
    /* 外部输入指令；通常由 UART/CAN 参数表或上层轨迹规划器写入。 */
    float torque_nm;       /* 力矩指令，Nm */
    float velocity_rad_s;  /* 速度指令，rad/s */
    float position_rad;    /* 位置指令，rad */
} ControlInput;

typedef struct {
    /* 单轴运行状态；状态机只改变状态，硬件关断通过 HAL 抽象完成。 */
    AxisStateId current_state;    /* 当前状态 */
    AxisStateId requested_state;  /* 请求状态，由通信或调试代码写入 */
    ControlMode control_mode;     /* 当前控制模式 */
    uint32_t error;               /* 故障位掩码，使用 FaultFlags */
    bool calibration_valid;       /* 电机/编码器校准结果有效 */
    bool closed_loop_allowed;     /* 状态机允许闭环输出 */
} AxisState;

typedef struct {
    /*
     * 单轴聚合对象。
     * 第一版只考虑单电机单轴；多轴系统可为每个轴各维护一个 Axis 实例。
     */
    MotorConfig motor;            /* 电机参数 */
    MotorState motor_state;       /* 电机实时状态 */
    FocConfig foc_config;         /* FOC 配置 */
    FocState foc_state;           /* FOC 中间状态 */
    EncoderState encoder;         /* 编码器状态 */
    ControlInput input;           /* 外部输入指令 */
    AxisState axis;               /* 轴状态机状态 */
} Axis;

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_TYPES_H */
