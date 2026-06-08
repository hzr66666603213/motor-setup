function build_foc_caller_smoke_model()
% 自动生成 foc_sim_step_wrapper 的 Simulink smoke test 模型
%
% 目标：
% Step iq_target -> C Caller -> duty_u/v/w Scope
%
% 前提：
% 1. MATLAB 已经配置好 mex C 编译器
% 2. 当前工程里已经有 src/sim/foc_sim.c
% 3. foc_sim.h 中已经声明 foc_sim_step_wrapper()

clc;

proj = 'D:\Documents\motor control';
cd(proj);

mdl = 'foc_caller_smoke_autogen';

if bdIsLoaded(mdl)
    close_system(mdl, 0);
end

new_system(mdl);
open_system(mdl);

%% 基本仿真设置
set_param(mdl, ...
    'StopTime', '0.02', ...
    'SolverType', 'Fixed-step', ...
    'Solver', 'FixedStepDiscrete', ...
    'FixedStep', '5e-5');

%% 配置 Custom Code
% 注意：这里用相对路径，避免 "motor control" 中的空格导致路径被拆开
set_param(mdl, 'SimCustomHeaderCode', ...
    '#include "sim/foc_sim.h"');

set_param(mdl, 'SimUserIncludeDirs', sprintf('%s\n%s\n%s\n%s\n%s', ...
    'include', ...
    'src', ...
    'src\sim', ...
    'src\control', ...
    'src\foc'));

set_param(mdl, 'SimUserSources', sprintf('%s\n%s\n%s\n%s', ...
    'src\sim\foc_sim.c', ...
    'src\control\current_controller.c', ...
    'src\foc\foc_math.c', ...
    'src\foc\svpwm.c'));

%% 添加 C Caller
caller = [mdl '/FOC_Caller'];
add_block('simulink/User-Defined Functions/C Caller', caller, ...
    'Position', [420 120 700 420]);

set_param(caller, ...
    'FunctionName', 'foc_sim_step_wrapper', ...
    'SampleTime', '5e-5');

% 更新一次模型，让 C Caller 解析函数端口
try
    set_param(mdl, 'SimulationCommand', 'update');
catch ME
    warning("模型更新时遇到问题：%s", ME.message);
    disp("请双击 C Caller，确认 Function name = foc_sim_step_wrapper，并把输出指针端口 Size 设置为 1。");
end

%% 输入模块
% C Caller 输入端口顺序：
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

addConst(mdl, 'ia_0',        '0',     [80 125 130 155]);
addConst(mdl, 'ib_0',        '0',     [80 160 130 190]);
addConst(mdl, 'ic_0',        '0',     [80 195 130 225]);
addConst(mdl, 'angle_0',     '0',     [80 230 130 260]);
addConst(mdl, 'speed_5',     '5',     [80 265 130 295]);
addConst(mdl, 'vbus_12',     '12',    [80 300 130 330]);
addConst(mdl, 'id_ref_0',    '0',     [80 335 130 365]);
addStep(mdl,  'iq_step',             [65 370 145 410]);
addConst(mdl, 'dt_50us',     '5e-5',  [80 425 130 455]);
addConst(mdl, 'pole_pairs',  '7.0',   [80 460 130 490]);
addConst(mdl, 'offset_0',    '0',     [80 495 130 525]);

inputBlocks = { ...
    'ia_0', ...
    'ib_0', ...
    'ic_0', ...
    'angle_0', ...
    'speed_5', ...
    'vbus_12', ...
    'id_ref_0', ...
    'iq_step', ...
    'dt_50us', ...
    'pole_pairs', ...
    'offset_0'};

for k = 1:numel(inputBlocks)
    add_line(mdl, [inputBlocks{k} '/1'], ['FOC_Caller/' num2str(k)], ...
        'autorouting', 'on');
end

%% 输出模块
% C Caller 输出端口顺序通常为：
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

add_block('simulink/Sinks/Display', [mdl '/status_display'], ...
    'Position', [780 120 850 155]);

add_block('simulink/Sinks/Display', [mdl '/iq_display'], ...
    'Position', [780 190 850 225]);

add_block('simulink/Signal Routing/Mux', [mdl '/duty_mux'], ...
    'Inputs', '3', ...
    'Position', [790 285 820 365]);

add_block('simulink/Sinks/Scope', [mdl '/duty_scope'], ...
    'Position', [900 300 960 360]);

add_line(mdl, 'FOC_Caller/1', 'status_display/1', 'autorouting', 'on');
add_line(mdl, 'FOC_Caller/3', 'iq_display/1', 'autorouting', 'on');

add_line(mdl, 'FOC_Caller/8', 'duty_mux/1', 'autorouting', 'on');
add_line(mdl, 'FOC_Caller/9', 'duty_mux/2', 'autorouting', 'on');
add_line(mdl, 'FOC_Caller/10', 'duty_mux/3', 'autorouting', 'on');
add_line(mdl, 'duty_mux/1', 'duty_scope/1', 'autorouting', 'on');

%% 未使用输出接 Terminator，避免警告
unusedPorts = [2 4 5 6 7]; % id, vd, vq, v_alpha, v_beta
for i = 1:numel(unusedPorts)
    termName = sprintf('term_out_%d', unusedPorts(i));
    add_block('simulink/Sinks/Terminator', [mdl '/' termName], ...
        'Position', [780 420 + 35*i 820 435 + 35*i]);
    add_line(mdl, ['FOC_Caller/' num2str(unusedPorts(i))], ...
        [termName '/1'], 'autorouting', 'on');
end

%% 保存模型
save_system(mdl);

disp("模型已生成：");
disp(mdl);
disp("现在可以点击运行，或执行：");
disp("sim('foc_caller_smoke_autogen')");

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