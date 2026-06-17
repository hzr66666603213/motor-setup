#include "hal/hal_pwm.h"

/*
 * hal_pwm_stm32f405.c
 *
 * ODrive v3.6 Axis0 TIM1 浜掕ˉ PWM 鐪熷疄鍚庣銆?
 *
 * 璧勬簮鍋囪锛?
 * - TIM1_CH1/2/3锛歅A8/PA9/PA10锛岄珮杈?PWM锛?
 * - TIM1_CH1N/2N/3N锛歅B13/PB14/PB15锛屼綆杈逛簰琛?PWM锛?
 * - TIM1_CH4锛氬缓璁綔涓?injected ADC 閲囨牱瑙﹀彂鐐癸紱
 * - CubeMX/LL 璐熻矗閰嶇疆涓績瀵归綈銆佹鍖恒€乥reak/off-state銆丳WM 棰戠巼 20 kHz銆?
 *
 * 瀹夊叏璇箟锛?
 * - hal_pwm_disable() 鍏抽棴 MOE锛屽苟鍋滄涓夌浉 CH/CHN锛?
 * - fault/idle 杩樺繀椤荤敱 board/DRV 灞傛媺浣?EN_GATE锛?
 * - hal_pwm_start_adc_trigger_only() 鍙繍琛?TIM1/CC4 瑙﹀彂 ADC锛屼笉鍚姩涓夌浉鍔熺巼閫氶亾銆?
 */

#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim1;

static bool s_pwm_enabled = false;
static HalPwmDiagnostics s_pwm_diagnostics;

static float clamp01(float x)
{
    if (x < 0.0f) {
        return 0.0f;
    }
    if (x > 1.0f) {
        return 1.0f;
    }
    return x;
}

static uint32_t duty_to_ccr(float duty)
{
    const uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
    return (uint32_t)(clamp01(duty) * (float)arr + 0.5f);
}

bool hal_pwm_init(void)
{
    hal_pwm_disable();
    hal_pwm_set_duty(0.5f, 0.5f, 0.5f);
    return true;
}

void hal_pwm_enable(void)
{
    /*
     * 鐪熸浣胯兘鍔熺巼绾?PWM銆?
     * 璋冪敤鍓?board 灞傚繀椤诲凡缁忕‘璁?fault銆乂BUS銆丄DC銆佺紪鐮佸櫒銆丏RV 鐘舵€佹弧瓒冲噯鍏ユ潯浠躲€?
     */
    hal_pwm_set_duty(0.5f, 0.5f, 0.5f);
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    (void)HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    (void)HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    (void)HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    __HAL_TIM_MOE_ENABLE(&htim1);
    s_pwm_enabled = true;
}

void hal_pwm_disable(void)
{
    /*
     * 鏁呴殰/IDLE 鐨勫叏鍏虫柇锛氬仠姝笁鐩稿姛鐜?PWM 鍜?ADC 瑙﹀彂銆?
     * current_offset_calibration 涓嶅簲璋冪敤杩欎釜鍑芥暟锛屽洜涓哄畠闇€瑕?TIM1 缁х画瑙﹀彂 ADC銆?
     */
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
    (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    (void)HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_4);
    (void)HAL_TIM_Base_Stop(&htim1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0u);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0u);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0u);
    s_pwm_enabled = false;
}

void hal_pwm_start_adc_trigger_only(void)
{
    /*
     * ADC trigger-only 妯″紡锛?
     * - 鍏ュ彛鍏堝叧闂?MOE锛?
     * - 涓嶅惎鍔?CH1/2/3 鍜?CH1N/2N/3N锛?
     * - 榛樿鍙惎鍔?TIM1 base + OC4锛岃 TIM1_CC4 瑙﹀彂 ADC injected conversion锛?
     * - 濡傛灉浣犵殑 CubeMX 浣跨敤 TIM1 TRGO/update 瑙﹀彂 ADC锛屽彲浠ヤ繚鐣?Base_Start 骞舵寜瀹炵墿閰嶇疆璋冩暣銆?
    */
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
    (void)HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_4);
    (void)HAL_TIM_Base_Stop(&htim1);

    __HAL_TIM_DISABLE(&htim1);
    __HAL_TIM_SET_COUNTER(&htim1, 0u);
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE | TIM_FLAG_CC1 |
                                  TIM_FLAG_CC2 | TIM_FLAG_CC3 |
                                  TIM_FLAG_CC4);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0u);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0u);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0u);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, duty_to_ccr(0.5f));

    s_pwm_diagnostics.oc4_start_status = (uint32_t)HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4);
    s_pwm_diagnostics.base_start_status = (uint32_t)HAL_TIM_Base_Start(&htim1);
    s_pwm_diagnostics.start_count++;

    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
    s_pwm_enabled = false;
}

void hal_pwm_set_duty(float duty_u, float duty_v, float duty_w)
{
    /*
     * 20 kHz ISR 鍐呰皟鐢紝蹇呴』鍙啓瀵勫瓨鍣紝涓嶉樆濉炪€?
     * TIM1 浜掕ˉ杈撳嚭鐢辩‖浠跺拰姝诲尯鍗曞厓鐢熸垚锛岃蒋浠跺彧鏇存柊 CCR1/2/3銆?
     */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_to_ccr(duty_u));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty_to_ccr(duty_v));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, duty_to_ccr(duty_w));
}

void hal_pwm_set_all_low(void)
{
    /*
     * 绗竴闃舵 bring-up 榛樿瀹夊叏鎬佹槸鈥淢OE=0 + EN_GATE=0鈥濓紝涓嶆妸浣庤竟鍏ㄥ紑褰撲綔瀹夊叏鎬併€?
     */
    hal_pwm_disable();
}

bool hal_pwm_is_enabled(void)
{
    return s_pwm_enabled;
}

void hal_pwm_get_diagnostics(HalPwmDiagnostics *diagnostics)
{
    if (diagnostics == 0) {
        return;
    }

    *diagnostics = s_pwm_diagnostics;
}
