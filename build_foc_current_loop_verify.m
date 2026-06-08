function build_foc_current_loop_verify()
% build_foc_current_loop_verify.m
%
% 重新生成 FOC 电流环 Simulink 波形验证模型。
%
% 适用目录：
%   D:\Documents\motor_control
%
% 本脚本会：
% 1. 删除旧 foc_current_loop_verify.slx；
% 2. 删除 slprj 缓存；
% 3. 生成 simulink_foc_entry.h；
% 4. 新建 Simulink 模型；
% 5. 配置 Custom Code；
% 6. 配置 C Caller 调用 foc_sim_step_wrapper；
% 7. 自动连接 20 个输入端口、10 个输出端口；
% 8. 搭建一阶 q 轴电流对象：
%       diq/dt = (vq - R*iq)/L
%
% C Caller 端口说明：
% foc_sim_step_wrapper() 有：
%   11 个普通输入
%   9 个 double* 输出指针
%
% Simulink C Caller 会把 double* 指针同时生成为输入端口和输出端口：
%   输入端口 = 11 + 9 = 20
%   输出端口 = status + 9 = 10
%
% 输出端口顺序通常为：
%   1 status
%   2 id_a
%   3 iq_a
%   4 vd_v
%   5 vq_v
%   6 v_alpha_v
%   7 v_beta_v
%   8 duty_u
%   9 duty_v
%   10 duty_w

clc;

%% ========== 基础参数 ==========
model = 'foc_current_loop_verify';

Ts = 5e-5;              % 20 kHz 电流环
stop_time = 0.05;       % 50 ms

R_phase = 0.5;          % ohm
L_phase = 1e-3;         % H

vbus = 12.0;
pole_pairs = 7.0;
encoder_offset = 0.0;

id_ref = 0.0;
iq_step_time = 0.01;
iq_initial = 0.0;
iq_final = 0.5;

%% ========== 强制使用当前脚本所在目录 ==========
script_fullpath = mfilename('fullpath');
if isempty(script_fullpath)
    repo_root = pwd;
else
    repo_root = fileparts(script_fullpath);
end

repo_root = char(java.io.File(repo_root).getCanonicalPath());
cd(repo_root);

fprintf('\n当前仓库路径：%s\n', repo_root);

if contains(repo_root, ' ')
    warning(['当前路径包含空格：%s\n' ...
             'Simulink C Caller 对带空格路径不稳定，建议使用 D:\\Documents\\motor_control。'], repo_root);
end

%% ========== 清理旧模型和缓存 ==========
try
    bdclose all;
catch
end

try
    clear mex;
    clear functions;
catch
end

slprj_dir = fullfile(repo_root, 'slprj');
if exist(slprj_dir, 'dir')
    try
        rmdir(slprj_dir, 's');
        fprintf('已删除 slprj 缓存。\n');
    catch ME
        warning('slprj 删除失败：%s', ME.message);
    end
end

model_file = fullfile(repo_root, [model '.slx']);
if isfile(model_file)
    delete(model_file);
    fprintf('已删除旧模型：%s\n', model_file);
end

%% ========== 生成 Simulink 专用入口头文件 ==========
entry_header = fullfile(repo_root, 'simulink_foc_entry.h');

fid = fopen(entry_header, 'w');
if fid < 0
    error('无法创建入口头文件：%s', entry_header);
end

fprintf(fid, '#ifndef SIMULINK_FOC_ENTRY_H\n');
fprintf(fid, '#define SIMULINK_FOC_ENTRY_H\n\n');

fprintf(fid, 'int foc_sim_step_wrapper(double ia_a,\n');
fprintf(fid, '                         double ib_a,\n');
fprintf(fid, '                         double ic_a,\n');
fprintf(fid, '                         double mechanical_angle_rad,\n');
fprintf(fid, '                         double mechanical_velocity_rad_s,\n');
fprintf(fid, '                         double vbus_v,\n');
fprintf(fid, '                         double id_target_a,\n');
fprintf(fid, '                         double iq_target_a,\n');
fprintf(fid, '                         double dt_s,\n');
fprintf(fid, '                         double pole_pairs_double,\n');
fprintf(fid, '                         double encoder_offset_rad,\n');
fprintf(fid, '                         double *id_a,\n');
fprintf(fid, '                         double *iq_a,\n');
fprintf(fid, '                         double *vd_v,\n');
fprintf(fid, '                         double *vq_v,\n');
fprintf(fid, '                         double *v_alpha_v,\n');
fprintf(fid, '                         double *v_beta_v,\n');
fprintf(fid, '                         double *duty_u,\n');
fprintf(fid, '                         double *duty_v,\n');
fprintf(fid, '                         double *duty_w);\n\n');

fprintf(fid, '#endif\n');

fclose(fid);

fprintf('已生成入口头文件：%s\n', entry_header);

%% ========== 检查必要文件 ==========
required_files = {
    fullfile(repo_root, 'src', 'sim', 'foc_sim.c')
    fullfile(repo_root, 'src', 'control', 'current_controller.c')
    fullfile(repo_root, 'src', 'control', 'velocity_controller.c')
    fullfile(repo_root, 'src', 'foc', 'foc_math.c')
    fullfile(repo_root, 'src', 'foc', 'svpwm.c')
    fullfile(repo_root, 'include', 'sim', 'foc_sim.h')
    fullfile(repo_root, 'include', 'control', 'current_controller.h')
    fullfile(repo_root, 'include', 'control', 'velocity_controller.h')
    entry_header
};

for i = 1:numel(required_files)
    if ~isfile(required_files{i})
        error('找不到必要文件：%s', required_files{i});
    end
end

fprintf('必要文件检查通过。\n');

%% ========== 新建模型 ==========
new_system(model);
open_system(model);

%% ========== 模型求解器设置 ==========
set_param(model, 'SolverType', 'Fixed-step');
set_param(model, 'Solver', 'FixedStepDiscrete');
set_param(model, 'FixedStep', num2str(Ts));
set_param(model, 'StopTime', num2str(stop_time));
set_param(model, 'SimulationMode', 'normal');

%% ========== Custom Code 设置 ==========
include_dirs = {
    repo_root
    fullfile(repo_root, 'include')
    fullfile(repo_root, 'src')
    fullfile(repo_root, 'src', 'foc')
    fullfile(repo_root, 'src', 'control')
    fullfile(repo_root, 'src', 'sim')
};

source_files = {
    fullfile(repo_root, 'src', 'sim', 'foc_sim.c')
    fullfile(repo_root, 'src', 'control', 'current_controller.c')
    fullfile(repo_root, 'src', 'control', 'velocity_controller.c')
    fullfile(repo_root, 'src', 'foc', 'foc_math.c')
    fullfile(repo_root, 'src', 'foc', 'svpwm.c')
};

set_param(model, 'SimCustomHeaderCode', '#include "simulink_foc_entry.h"');
set_param(model, 'SimUserIncludeDirs', strjoin(include_dirs, newline));
set_param(model, 'SimUserSources', strjoin(source_files, newline));

% 清空可能残留的库、宏、编译/链接标志
try
    set_param(model, 'SimUserLibraries', '');
catch
end

try
    set_param(model, 'SimUserDefines', '');
catch
end

try
    set_param(model, 'SimCustomSourceCode', '');
catch
end

fprintf('\nCustom Code include dirs:\n%s\n', get_param(model, 'SimUserIncludeDirs'));
fprintf('\nCustom Code sources:\n%s\n', get_param(model, 'SimUserSources'));

%% ========== 常用坐标 ==========
x0 = 40;
y0 = 40;
dx = 150;
dy = 45;

%% ========== 输入源 ==========
add_block('simulink/Sources/Constant', [model '/ia_zero'], ...
    'Value', '0', ...
    'Position', [x0 y0 x0+80 y0+25]);

add_block('simulink/Math Operations/Gain', [model '/ib_from_iq'], ...
    'Gain', 'sqrt(3)/2', ...
    'Position', [x0 y0+dy x0+100 y0+dy+25]);

add_block('simulink/Math Operations/Gain', [model '/ic_from_ib'], ...
    'Gain', '-1', ...
    'Position', [x0+130 y0+dy x0+210 y0+dy+25]);

add_block('simulink/Sources/Constant', [model '/mechanical_angle_rad'], ...
    'Value', '0', ...
    'Position', [x0 y0+2*dy x0+130 y0+2*dy+25]);

add_block('simulink/Sources/Constant', [model '/mechanical_velocity_rad_s'], ...
    'Value', '0', ...
    'Position', [x0 y0+3*dy x0+150 y0+3*dy+25]);

add_block('simulink/Sources/Constant', [model '/vbus_v'], ...
    'Value', num2str(vbus), ...
    'Position', [x0 y0+4*dy x0+80 y0+4*dy+25]);

add_block('simulink/Sources/Constant', [model '/id_target_a'], ...
    'Value', num2str(id_ref), ...
    'Position', [x0 y0+5*dy x0+90 y0+5*dy+25]);

add_block('simulink/Sources/Step', [model '/iq_target_step'], ...
    'Time', num2str(iq_step_time), ...
    'Before', num2str(iq_initial), ...
    'After', num2str(iq_final), ...
    'SampleTime', num2str(Ts), ...
    'Position', [x0 y0+6*dy x0+110 y0+6*dy+30]);

add_block('simulink/Sources/Constant', [model '/dt_s'], ...
    'Value', num2str(Ts), ...
    'Position', [x0 y0+7*dy x0+80 y0+7*dy+25]);

add_block('simulink/Sources/Constant', [model '/pole_pairs'], ...
    'Value', num2str(pole_pairs), ...
    'Position', [x0 y0+8*dy x0+90 y0+8*dy+25]);

add_block('simulink/Sources/Constant', [model '/encoder_offset_rad'], ...
    'Value', num2str(encoder_offset), ...
    'Position', [x0 y0+9*dy x0+140 y0+9*dy+25]);

%% ========== C Caller ==========
c_caller = [model '/FOC_C_Caller'];

add_block('simulink/User-Defined Functions/C Caller', c_caller, ...
    'Position', [x0+2*dx y0+2*dy x0+2*dx+250 y0+2*dy+310]);

try
    set_param(c_caller, 'FunctionName', 'foc_sim_step_wrapper');
catch ME
    warning('设置 C Caller FunctionName 失败：%s', ME.message);
end

%% ========== 强制 update，让 C Caller 解析端口 ==========
try
    set_param(model, 'SimulationCommand', 'update');
catch ME
    warning(['模型 update 失败：%s\n\n' ...
             '请双击 FOC_C_Caller，Refresh Custom Code Parser，选择 foc_sim_step_wrapper。\n' ...
             '如果仍失败，请检查 Custom Code 路径是否全部是 D:\\Documents\\motor_control。\n'], ME.message);
end

ph = get_param(c_caller, 'PortHandles');

expected_inports = 20;
expected_outports = 10;

if numel(ph.Inport) ~= expected_inports || numel(ph.Outport) ~= expected_outports
    warning(['C Caller 端口数量不符合预期。\n\n' ...
             '当前输入端口数：%d\n' ...
             '当前输出端口数：%d\n\n' ...
             '预期：20 输入、10 输出。\n\n' ...
             '原因：9 个 double* 输出指针会被 C Caller 同时生成为输入端口和输出端口。\n\n' ...
             '处理办法：\n' ...
             '1. 双击 FOC_C_Caller。\n' ...
             '2. 点击 Refresh Custom Code Parser。\n' ...
             '3. 选择 foc_sim_step_wrapper。\n' ...
             '4. 确认端口为 20 输入、10 输出后，重新运行本脚本。\n'], ...
             numel(ph.Inport), numel(ph.Outport));
    save_system(model);
    open_system(model);
    return;
end

fprintf('\nC Caller 端口解析成功：20 输入，10 输出。\n');

%% ========== 连接 C Caller 前 11 个普通输入 ==========
% 输入顺序：
% 1  ia_a
% 2  ib_a
% 3  ic_a
% 4  mechanical_angle_rad
% 5  mechanical_velocity_rad_s
% 6  vbus_v
% 7  id_target_a
% 8  iq_target_a
% 9  dt_s
% 10 pole_pairs_double
% 11 encoder_offset_rad
% 12~20 double* 输出指针输入占位

add_line(model, 'ia_zero/1', 'FOC_C_Caller/1', 'autorouting', 'on');

add_line(model, 'ib_from_iq/1', 'FOC_C_Caller/2', 'autorouting', 'on');

add_line(model, 'ib_from_iq/1', 'ic_from_ib/1', 'autorouting', 'on');
add_line(model, 'ic_from_ib/1', 'FOC_C_Caller/3', 'autorouting', 'on');

add_line(model, 'mechanical_angle_rad/1', 'FOC_C_Caller/4', 'autorouting', 'on');
add_line(model, 'mechanical_velocity_rad_s/1', 'FOC_C_Caller/5', 'autorouting', 'on');
add_line(model, 'vbus_v/1', 'FOC_C_Caller/6', 'autorouting', 'on');
add_line(model, 'id_target_a/1', 'FOC_C_Caller/7', 'autorouting', 'on');
add_line(model, 'iq_target_step/1', 'FOC_C_Caller/8', 'autorouting', 'on');
add_line(model, 'dt_s/1', 'FOC_C_Caller/9', 'autorouting', 'on');
add_line(model, 'pole_pairs/1', 'FOC_C_Caller/10', 'autorouting', 'on');
add_line(model, 'encoder_offset_rad/1', 'FOC_C_Caller/11', 'autorouting', 'on');

%% ========== 给 double* 输出指针输入端口接 dummy zero ==========
add_block('simulink/Sources/Constant', [model '/dummy_pointer_zero'], ...
    'Value', '0', ...
    'Position', [x0+dx y0+10*dy x0+dx+110 y0+10*dy+25]);

for k = 12:20
    add_line(model, 'dummy_pointer_zero/1', ...
        sprintf('FOC_C_Caller/%d', k), ...
        'autorouting', 'on');
end

%% ========== q 轴一阶对象：diq/dt = (vq - R*iq)/L ==========
add_block('simulink/Math Operations/Gain', [model '/R_iq'], ...
    'Gain', num2str(R_phase), ...
    'Position', [x0+4*dx y0+5*dy x0+4*dx+80 y0+5*dy+25]);

add_block('simulink/Math Operations/Sum', [model '/vq_minus_Riq'], ...
    'Inputs', '+-', ...
    'Position', [x0+4*dx y0+3*dy x0+4*dx+45 y0+3*dy+40]);

add_block('simulink/Math Operations/Gain', [model '/one_over_L'], ...
    'Gain', num2str(1/L_phase), ...
    'Position', [x0+4*dx+95 y0+3*dy x0+4*dx+180 y0+3*dy+25]);

add_block('simulink/Discrete/Discrete-Time Integrator', [model '/iq_plant_integrator'], ...
    'gainval', '1', ...
    'InitialCondition', '0', ...
    'SampleTime', num2str(Ts), ...
    'Position', [x0+4*dx+230 y0+3*dy x0+4*dx+340 y0+3*dy+40]);

% C Caller 输出顺序：
% 1 status
% 2 id_a
% 3 iq_a
% 4 vd_v
% 5 vq_v
% 6 v_alpha_v
% 7 v_beta_v
% 8 duty_u
% 9 duty_v
% 10 duty_w

add_line(model, 'FOC_C_Caller/5', 'vq_minus_Riq/1', 'autorouting', 'on');

add_line(model, 'iq_plant_integrator/1', 'R_iq/1', 'autorouting', 'on');
add_line(model, 'R_iq/1', 'vq_minus_Riq/2', 'autorouting', 'on');

add_line(model, 'vq_minus_Riq/1', 'one_over_L/1', 'autorouting', 'on');
add_line(model, 'one_over_L/1', 'iq_plant_integrator/1', 'autorouting', 'on');

% iq -> ib/ic 映射
add_line(model, 'iq_plant_integrator/1', 'ib_from_iq/1', 'autorouting', 'on');

%% ========== 电压幅值 sqrt(vd^2 + vq^2) ==========
add_block('simulink/Math Operations/Math Function', [model '/vd_square'], ...
    'Operator', 'square', ...
    'Position', [x0+4*dx y0 x0+4*dx+80 y0+30]);

add_block('simulink/Math Operations/Math Function', [model '/vq_square'], ...
    'Operator', 'square', ...
    'Position', [x0+4*dx y0+dy x0+4*dx+80 y0+dy+30]);

add_block('simulink/Math Operations/Sum', [model '/vd2_plus_vq2'], ...
    'Inputs', '++', ...
    'Position', [x0+4*dx+120 y0+20 x0+4*dx+165 y0+60]);

add_block('simulink/Math Operations/Sqrt', [model '/voltage_mag'], ...
    'Position', [x0+4*dx+210 y0+25 x0+4*dx+270 y0+55]);

add_line(model, 'FOC_C_Caller/4', 'vd_square/1', 'autorouting', 'on');
add_line(model, 'FOC_C_Caller/5', 'vq_square/1', 'autorouting', 'on');
add_line(model, 'vd_square/1', 'vd2_plus_vq2/1', 'autorouting', 'on');
add_line(model, 'vq_square/1', 'vd2_plus_vq2/2', 'autorouting', 'on');
add_line(model, 'vd2_plus_vq2/1', 'voltage_mag/1', 'autorouting', 'on');

%% ========== Scope 1：iq_ref vs iq_meas ==========
add_block('simulink/Signal Routing/Mux', [model '/mux_iq'], ...
    'Inputs', '2', ...
    'Position', [x0+6*dx y0+2*dy x0+6*dx+40 y0+2*dy+55]);

add_block('simulink/Sinks/Scope', [model '/Scope_iq_ref_vs_iq_meas'], ...
    'Position', [x0+7*dx y0+2*dy x0+7*dx+190 y0+2*dy+100]);

add_line(model, 'iq_target_step/1', 'mux_iq/1', 'autorouting', 'on');
add_line(model, 'iq_plant_integrator/1', 'mux_iq/2', 'autorouting', 'on');
add_line(model, 'mux_iq/1', 'Scope_iq_ref_vs_iq_meas/1', 'autorouting', 'on');

%% ========== Scope 2：vd/vq/voltage_mag ==========
add_block('simulink/Signal Routing/Mux', [model '/mux_voltage'], ...
    'Inputs', '3', ...
    'Position', [x0+6*dx y0+4*dy x0+6*dx+40 y0+4*dy+75]);

add_block('simulink/Sinks/Scope', [model '/Scope_vd_vq_voltage_mag'], ...
    'Position', [x0+7*dx y0+4*dy x0+7*dx+190 y0+4*dy+110]);

add_line(model, 'FOC_C_Caller/4', 'mux_voltage/1', 'autorouting', 'on');
add_line(model, 'FOC_C_Caller/5', 'mux_voltage/2', 'autorouting', 'on');
add_line(model, 'voltage_mag/1', 'mux_voltage/3', 'autorouting', 'on');
add_line(model, 'mux_voltage/1', 'Scope_vd_vq_voltage_mag/1', 'autorouting', 'on');

%% ========== Scope 3：duty_u/v/w ==========
add_block('simulink/Signal Routing/Mux', [model '/mux_duty'], ...
    'Inputs', '3', ...
    'Position', [x0+6*dx y0+6*dy x0+6*dx+40 y0+6*dy+75]);

add_block('simulink/Sinks/Scope', [model '/Scope_duty_uvw'], ...
    'Position', [x0+7*dx y0+6*dy x0+7*dx+190 y0+6*dy+110]);

add_line(model, 'FOC_C_Caller/8', 'mux_duty/1', 'autorouting', 'on');
add_line(model, 'FOC_C_Caller/9', 'mux_duty/2', 'autorouting', 'on');
add_line(model, 'FOC_C_Caller/10', 'mux_duty/3', 'autorouting', 'on');
add_line(model, 'mux_duty/1', 'Scope_duty_uvw/1', 'autorouting', 'on');

%% ========== Scope 4：status ==========
add_block('simulink/Sinks/Scope', [model '/Scope_status'], ...
    'Position', [x0+7*dx y0+8*dy x0+7*dx+190 y0+8*dy+80]);

add_line(model, 'FOC_C_Caller/1', 'Scope_status/1', 'autorouting', 'on');

%% ========== To Workspace ==========
add_block('simulink/Sinks/To Workspace', [model '/to_ws_iq_ref'], ...
    'VariableName', 'iq_ref_log', ...
    'SaveFormat', 'Timeseries', ...
    'Position', [x0+6*dx y0+9*dy x0+6*dx+130 y0+9*dy+25]);

add_block('simulink/Sinks/To Workspace', [model '/to_ws_iq_meas'], ...
    'VariableName', 'iq_meas_log', ...
    'SaveFormat', 'Timeseries', ...
    'Position', [x0+6*dx y0+10*dy x0+6*dx+130 y0+10*dy+25]);

add_block('simulink/Sinks/To Workspace', [model '/to_ws_vq'], ...
    'VariableName', 'vq_log', ...
    'SaveFormat', 'Timeseries', ...
    'Position', [x0+6*dx y0+11*dy x0+6*dx+130 y0+11*dy+25]);

add_block('simulink/Sinks/To Workspace', [model '/to_ws_voltage_mag'], ...
    'VariableName', 'voltage_mag_log', ...
    'SaveFormat', 'Timeseries', ...
    'Position', [x0+6*dx y0+12*dy x0+6*dx+160 y0+12*dy+25]);

add_line(model, 'iq_target_step/1', 'to_ws_iq_ref/1', 'autorouting', 'on');
add_line(model, 'iq_plant_integrator/1', 'to_ws_iq_meas/1', 'autorouting', 'on');
add_line(model, 'FOC_C_Caller/5', 'to_ws_vq/1', 'autorouting', 'on');
add_line(model, 'voltage_mag/1', 'to_ws_voltage_mag/1', 'autorouting', 'on');

%% ========== 整理和保存 ==========
try
    Simulink.BlockDiagram.arrangeSystem(model);
catch
end

save_system(model);

fprintf('\n已生成模型：%s\n', model_file);
fprintf('\n运行仿真：\n');
fprintf('  sim(''%s'')\n', model);

fprintf('\n正常结果：\n');
fprintf('1. iq_ref 在 %.3f s 从 %.2f A 跳到 %.2f A。\n', iq_step_time, iq_initial, iq_final);
fprintf('2. iq_meas 应跟随到 %.2f A 附近。\n', iq_final);
fprintf('3. voltage_mag 不应超过 3 V。\n');
fprintf('4. duty_u/v/w 应保持在 0~1。\n');
fprintf('5. status 应一直为 0。\n');

fprintf('\n可直接画图：\n');
fprintf('  sim(''%s'');\n', model);
fprintf('  figure; plot(iq_ref_log.Time, iq_ref_log.Data); hold on; plot(iq_meas_log.Time, iq_meas_log.Data); grid on; legend(''iq ref'',''iq meas'');\n');
fprintf('  figure; plot(vq_log.Time, vq_log.Data); hold on; plot(voltage_mag_log.Time, voltage_mag_log.Data); grid on; legend(''vq'',''voltage mag'');\n');

end