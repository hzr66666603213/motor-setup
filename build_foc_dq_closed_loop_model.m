function build_foc_dq_closed_loop_model()
% 自动生成 FOC 电流环闭环 Simulink 模型
%
% 结构：
% iq_target Step
%      ↓
% foc_sim_step_wrapper()
%      ↓ vd/vq
% dq 轴 RL 电机模型
%      ↓ id/iq
% dq_to_abc(theta_e = 0)
%      ↓ ia/ib/ic
% 反馈到 foc_sim_step_wrapper()
%
% 前提：
% 1. 当前 MATLAB C 编译器已配置好：mex -setup C
% 2. 工程根目录包含：
%    src/sim/foc_sim.c
%    src/control/current_controller.c
%    src/control/velocity_controller.c
%    src/foc/foc_math.c
%    src/foc/svpwm.c
% 3. foc_sim.h 中已声明 foc_sim_step_wrapper()

clc;

%% 工程路径
proj = 'D:\Documents\motor control';
cd(proj);

mdl = 'foc_dq_closed_loop_autogen';

Ts = 5e-5;        % 20 kHz
StopTime = 0.05;  % 50 ms

% 简化 2804 电机初值，后面可根据实测修改
R_phase = 0.3;       % ohm
L_phase = 50e-6;     % H
inv_L = 1.0 / L_phase;

if bdIsLoaded(mdl)
    close_system(mdl, 0);
end

new_system(mdl);
open_system(mdl);

%% 仿真设置
set_param(mdl, ...
    'StopTime', num2str(StopTime), ...
    'SolverType', 'Fixed-step', ...
    'Solver', 'FixedStepDiscrete', ...
    'FixedStep', num2str(Ts));

%% Custom Code 配置
% 使用相对路径，避免 "motor control" 路径空格被拆开
set_param(mdl, 'SimCustomHeaderCode', ...
    '#include "sim/foc_sim.h"');

set_param(mdl, 'SimUserIncludeDirs', sprintf('%s\n%s\n%s\n%s\n%s', ...
    'include', ...
    'src', ...
    'src\sim', ...
    'src\control', ...
    'src\foc'));

set_param(mdl, 'SimUserSources', sprintf('%s\n%s\n%s\n%s\n%s', ...
    'src\sim\foc_sim.c', ...
    'src\control\current_controller.c', ...
    'src\control\velocity_controller.c', ...
    'src\foc\foc_math.c', ...
    'src\foc\svpwm.c'));

%% 添加 C Caller
caller = [mdl '/FOC_Caller'];
add_block('simulink/User-Defined Functions/C Caller', caller, ...
    'Position', [520 150 820 500]);

set_param(caller, ...
    'FunctionName', 'foc_sim_step_wrapper', ...
    'SampleTime', num2str(Ts));

try
    set_param(mdl, 'SimulationCommand', 'update');
catch ME
    warning("模型更新时遇到问题：%s", ME.message);
    disp("如果提示输出参数尺寸，请双击 FOC_Caller，把 id_a/iq_a/vd_v/vq_v/v_alpha_v/v_beta_v/duty_u/duty_v/duty_w 的 Size 设置为 1。");
end

%% 输入常量和 Step
addConst(mdl, 'mechanical_angle_0', '0',      [80 250 140 280]);
addConst(mdl, 'mechanical_speed_0', '0',      [80 290 140 320]);
addConst(mdl, 'vbus_12',            '12',     [80 330 140 360]);
addConst(mdl, 'id_ref_0',           '0',      [80 370 140 400]);
addStep (mdl, 'iq_ref_step',                  [70 410 150 450]);
addConst(mdl, 'dt_50us',            '5e-5',   [80 460 140 490]);
addConst(mdl, 'pole_pairs_7',       '7.0',    [80 500 140 530]);
addConst(mdl, 'offset_0',           '0',      [80 540 140 570]);

%% dq 轴 RL 电机模型
% id[k+1] = id[k] + Ts * (vd - R*id)/L
% iq[k+1] = iq[k] + Ts * (vq - R*iq)/L

% id state
add_block('simulink/Discrete/Unit Delay', [mdl '/id_delay'], ...
    'InitialCondition', '0', ...
    'SampleTime', num2str(Ts), ...
    'Position', [1110 170 1160 210]);

add_block('simulink/Math Operations/Gain', [mdl '/R_id'], ...
    'Gain', num2str(R_phase), ...
    'Position', [990 220 1050 250]);

add_block('simulink/Math Operations/Sum', [mdl '/sum_vd_minus_Rid'], ...
    'Inputs', '+-', ...
    'Position', [1070 250 1100 290]);

add_block('simulink/Math Operations/Gain', [mdl '/invL_id'], ...
    'Gain', num2str(inv_L), ...
    'Position', [1130 250 1190 290]);

add_block('simulink/Math Operations/Gain', [mdl '/Ts_id'], ...
    'Gain', num2str(Ts), ...
    'Position', [1220 250 1280 290]);

add_block('simulink/Math Operations/Sum', [mdl '/sum_id_next'], ...
    'Inputs', '++', ...
    'Position', [1310 205 1340 245]);

% iq state
add_block('simulink/Discrete/Unit Delay', [mdl '/iq_delay'], ...
    'InitialCondition', '0', ...
    'SampleTime', num2str(Ts), ...
    'Position', [1110 420 1160 460]);

add_block('simulink/Math Operations/Gain', [mdl '/R_iq'], ...
    'Gain', num2str(R_phase), ...
    'Position', [990 470 1050 500]);

add_block('simulink/Math Operations/Sum', [mdl '/sum_vq_minus_Riq'], ...
    'Inputs', '+-', ...
    'Position', [1070 500 1100 540]);

add_block('simulink/Math Operations/Gain', [mdl '/invL_iq'], ...
    'Gain', num2str(inv_L), ...
    'Position', [1130 500 1190 540]);

add_block('simulink/Math Operations/Gain', [mdl '/Ts_iq'], ...
    'Gain', num2str(Ts), ...
    'Position', [1220 500 1280 540]);

add_block('simulink/Math Operations/Sum', [mdl '/sum_iq_next'], ...
    'Inputs', '++', ...
    'Position', [1310 455 1340 495]);

%% dq -> abc，当前 theta_e = 0
% theta_e = 0 时：
% ia = id
% ib = -0.5*id + 0.8660254*iq
% ic = -0.5*id - 0.8660254*iq

add_block('simulink/Math Operations/Gain', [mdl '/gain_id_to_ib'], ...
    'Gain', '-0.5', ...
    'Position', [1230 80 1290 110]);

add_block('simulink/Math Operations/Gain', [mdl '/gain_iq_to_ib'], ...
    'Gain', '0.8660254037844386', ...
    'Position', [1230 120 1290 150]);

add_block('simulink/Math Operations/Sum', [mdl '/sum_ib'], ...
    'Inputs', '++', ...
    'Position', [1320 95 1350 135]);

add_block('simulink/Math Operations/Gain', [mdl '/gain_id_to_ic'], ...
    'Gain', '-0.5', ...
    'Position', [1230 600 1290 630]);

add_block('simulink/Math Operations/Gain', [mdl '/gain_iq_to_ic'], ...
    'Gain', '-0.8660254037844386', ...
    'Position', [1230 640 1290 670]);

add_block('simulink/Math Operations/Sum', [mdl '/sum_ic'], ...
    'Inputs', '++', ...
    'Position', [1320 615 1350 655]);

%% Scopes / Display
add_block('simulink/Sinks/Display', [mdl '/status_display'], ...
    'Position', [900 130 970 165]);

add_block('simulink/Signal Routing/Mux', [mdl '/duty_mux'], ...
    'Inputs', '3', ...
    'Position', [920 610 950 700]);

add_block('simulink/Sinks/Scope', [mdl '/duty_scope'], ...
    'Position', [1010 630 1070 690]);

add_block('simulink/Signal Routing/Mux', [mdl '/iq_compare_mux'], ...
    'Inputs', '2', ...
    'Position', [1510 380 1540 440]);

add_block('simulink/Sinks/Scope', [mdl '/iq_compare_scope'], ...
    'Position', [1600 390 1660 450]);

add_block('simulink/Signal Routing/Mux', [mdl '/id_iq_motor_mux'], ...
    'Inputs', '2', ...
    'Position', [1510 500 1540 560]);

add_block('simulink/Sinks/Scope', [mdl '/id_iq_motor_scope'], ...
    'Position', [1600 510 1660 570]);

%% C Caller 输入连接
% 输入端口：
% 1 ia_a
% 2 ib_a
% 3 ic_a
% 4 mechanical_angle_rad
% 5 mechanical_velocity_rad_s
% 6 vbus_v
% 7 id_target_a
% 8 iq_target_a
% 9 dt_s
% 10 pole_pairs_double
% 11 encoder_offset_rad

% ia = id_motor
add_line(mdl, 'id_delay/1', 'FOC_Caller/1', 'autorouting', 'on');

% ib / ic 来自逆 Clarke
add_line(mdl, 'sum_ib/1', 'FOC_Caller/2', 'autorouting', 'on');
add_line(mdl, 'sum_ic/1', 'FOC_Caller/3', 'autorouting', 'on');

add_line(mdl, 'mechanical_angle_0/1', 'FOC_Caller/4', 'autorouting', 'on');
add_line(mdl, 'mechanical_speed_0/1', 'FOC_Caller/5', 'autorouting', 'on');
add_line(mdl, 'vbus_12/1',            'FOC_Caller/6', 'autorouting', 'on');
add_line(mdl, 'id_ref_0/1',           'FOC_Caller/7', 'autorouting', 'on');
add_line(mdl, 'iq_ref_step/1',        'FOC_Caller/8', 'autorouting', 'on');
add_line(mdl, 'dt_50us/1',            'FOC_Caller/9', 'autorouting', 'on');
add_line(mdl, 'pole_pairs_7/1',       'FOC_Caller/10', 'autorouting', 'on');
add_line(mdl, 'offset_0/1',           'FOC_Caller/11', 'autorouting', 'on');

%% 电机模型连接
% FOC 输出端口：
% 1 out/status
% 2 id_a
% 3 iq_a
% 4 vd_v
% 5 vq_v
% 6 v_alpha_v
% 7 v_beta_v
% 8 duty_u
% 9 duty_v
% 10 duty_w

% vd path
add_line(mdl, 'FOC_Caller/4', 'sum_vd_minus_Rid/1', 'autorouting', 'on');
add_line(mdl, 'id_delay/1',   'R_id/1', 'autorouting', 'on');
add_line(mdl, 'R_id/1',       'sum_vd_minus_Rid/2', 'autorouting', 'on');
add_line(mdl, 'sum_vd_minus_Rid/1', 'invL_id/1', 'autorouting', 'on');
add_line(mdl, 'invL_id/1', 'Ts_id/1', 'autorouting', 'on');
add_line(mdl, 'Ts_id/1', 'sum_id_next/2', 'autorouting', 'on');
add_line(mdl, 'id_delay/1', 'sum_id_next/1', 'autorouting', 'on');
add_line(mdl, 'sum_id_next/1', 'id_delay/1', 'autorouting', 'on');

% vq path
add_line(mdl, 'FOC_Caller/5', 'sum_vq_minus_Riq/1', 'autorouting', 'on');
add_line(mdl, 'iq_delay/1',   'R_iq/1', 'autorouting', 'on');
add_line(mdl, 'R_iq/1',       'sum_vq_minus_Riq/2', 'autorouting', 'on');
add_line(mdl, 'sum_vq_minus_Riq/1', 'invL_iq/1', 'autorouting', 'on');
add_line(mdl, 'invL_iq/1', 'Ts_iq/1', 'autorouting', 'on');
add_line(mdl, 'Ts_iq/1', 'sum_iq_next/2', 'autorouting', 'on');
add_line(mdl, 'iq_delay/1', 'sum_iq_next/1', 'autorouting', 'on');
add_line(mdl, 'sum_iq_next/1', 'iq_delay/1', 'autorouting', 'on');

%% dq -> abc 连接
% ib
add_line(mdl, 'id_delay/1', 'gain_id_to_ib/1', 'autorouting', 'on');
add_line(mdl, 'iq_delay/1', 'gain_iq_to_ib/1', 'autorouting', 'on');
add_line(mdl, 'gain_id_to_ib/1', 'sum_ib/1', 'autorouting', 'on');
add_line(mdl, 'gain_iq_to_ib/1', 'sum_ib/2', 'autorouting', 'on');

% ic
add_line(mdl, 'id_delay/1', 'gain_id_to_ic/1', 'autorouting', 'on');
add_line(mdl, 'iq_delay/1', 'gain_iq_to_ic/1', 'autorouting', 'on');
add_line(mdl, 'gain_id_to_ic/1', 'sum_ic/1', 'autorouting', 'on');
add_line(mdl, 'gain_iq_to_ic/1', 'sum_ic/2', 'autorouting', 'on');

%% 输出观察
add_line(mdl, 'FOC_Caller/1', 'status_display/1', 'autorouting', 'on');

add_line(mdl, 'FOC_Caller/8',  'duty_mux/1', 'autorouting', 'on');
add_line(mdl, 'FOC_Caller/9',  'duty_mux/2', 'autorouting', 'on');
add_line(mdl, 'FOC_Caller/10', 'duty_mux/3', 'autorouting', 'on');
add_line(mdl, 'duty_mux/1', 'duty_scope/1', 'autorouting', 'on');

% iq_target vs iq_motor
add_line(mdl, 'iq_ref_step/1', 'iq_compare_mux/1', 'autorouting', 'on');
add_line(mdl, 'iq_delay/1',    'iq_compare_mux/2', 'autorouting', 'on');
add_line(mdl, 'iq_compare_mux/1', 'iq_compare_scope/1', 'autorouting', 'on');

% id_motor / iq_motor
add_line(mdl, 'id_delay/1', 'id_iq_motor_mux/1', 'autorouting', 'on');
add_line(mdl, 'iq_delay/1', 'id_iq_motor_mux/2', 'autorouting', 'on');
add_line(mdl, 'id_iq_motor_mux/1', 'id_iq_motor_scope/1', 'autorouting', 'on');

%% 未使用输出接 Terminator
unusedPorts = [2 3 6 7]; % C Caller measured id/iq, v_alpha/v_beta
for i = 1:numel(unusedPorts)
    termName = sprintf('term_out_%d', unusedPorts(i));
    add_block('simulink/Sinks/Terminator', [mdl '/' termName], ...
        'Position', [900 720 + 35*i 940 735 + 35*i]);
    add_line(mdl, ['FOC_Caller/' num2str(unusedPorts(i))], ...
        [termName '/1'], 'autorouting', 'on');
end

%% 保存
save_system(mdl);

disp("已生成闭环 dq 电流模型：");
disp(mdl);
disp("运行：");
disp("sim('foc_dq_closed_loop_autogen')");

end

function addConst(mdl, name, value, pos)
blk = [mdl '/' name];
add_block('simulink/Sources/Constant', blk, ...
    'Value', value, ...
    'SampleTime', '5e-5', ...
    'Position', pos);
end

function addStep(mdl, name, pos)
blk = [mdl '/' name];
add_block('simulink/Sources/Step', blk, ...
    'Time', '0.005', ...
    'Before', '0', ...
    'After', '0.5', ...
    'SampleTime', '5e-5', ...
    'Position', pos);
end
