#ifndef POSITION_CONTROLLER_H
#define POSITION_CONTROLLER_H

/*
 * position_controller.h
 *
 * 位置环控制器。
 * 运行频率：1 kHz 或 500 Hz。
 * 第一版采用 P 控制，输出速度目标 rad/s，再交给速度环。
 */

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;                    /* 位置比例增益，(rad/s)/rad */
    float velocity_limit_rad_s;  /* 位置环输出速度限幅，rad/s */
    float position_min_rad;      /* 位置软限位下限，rad */
    float position_max_rad;      /* 位置软限位上限，rad */
} PositionController;

/* 初始化位置 P 控制器和软件限位。 */
void position_controller_init(PositionController *controller,
                              float kp,
                              float velocity_limit_rad_s,
                              float position_min_rad,
                              float position_max_rad);
/* 在线更新位置环增益。 */
void position_controller_set_gain(PositionController *controller, float kp);

/* 执行一次位置环，返回速度目标 rad/s。 */
float position_controller_update(PositionController *controller,
                                 float position_target_rad,
                                 float position_measured_rad);

#ifdef __cplusplus
}
#endif

#endif /* POSITION_CONTROLLER_H */
