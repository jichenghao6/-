%% ==================== 数控机床进给伺服系统建模与分析 ====================
% 第三章：时域、根轨迹、频域综合分析
% 说明：本代码建立系统模型，绘制响应曲线，并自动计算关键性能指标

clear; clc; close all;

%% 1. 系统参数定义（根据实际设备查手册，此处为示例值）
Km = 10;        % 电机传递系数 (rad/(V·s))
Tm = 0.1;       % 电机机电时间常数 (s)
Kt = 1;         % 转角到位移的转换系数 (m/rad)，与丝杠导程相关
K0 = 5;         % 综合伺服调节增益（含PID等）

K = K0 * Km * Kt;   % 系统开环总增益

%% 2. 建立传递函数模型
s = tf('s');
G_open = K / (s^2 * (Tm * s + 1));  % 开环传递函数
G_closed = feedback(G_open, 1);      % 闭环传递函数

fprintf('========== 系统传递函数 ==========\n');
fprintf('开环传递函数 G_open(s) = \n');
disp(G_open);
fprintf('闭环传递函数 G_closed(s) = \n');
disp(G_closed);

%% 3. 时域分析：单位阶跃响应及性能指标
figure(1);
[y, t] = step(G_closed);   % 获取阶跃响应数据
step(G_closed);            % 绘制阶跃响应曲线
grid on;
title('进给伺服系统单位阶跃响应');
xlabel('时间 (秒)');
ylabel('工作台位移 (m)');

% 计算性能指标
% 稳态值（终值）
C = dcgain(G_closed);
% 峰值时间与超调量
[Y_max, idx_max] = max(y);
peak_time = t(idx_max);
overshoot = (Y_max - C) / C * 100;  % 百分比

% 上升时间：从10%到90%稳态值的时间
% 使用 stepinfo 更方便，但我们手动计算以更灵活
t_10 = interp1(y, t, 0.1*C, 'pchip');
t_90 = interp1(y, t, 0.9*C, 'pchip');
rise_time = t_90 - t_10;

% 调节时间：进入±2%误差带的最短时间
error_band = 0.02 * C;
idx_settle = find(abs(y - C) < error_band, 1, 'last');
if isempty(idx_settle)
    settling_time = NaN;
else
    settling_time = t(idx_settle);
end

% 稳态误差（阶跃输入下，对于I型系统，稳态误差为0）
ess_step = 1 - C;   % 输入为1

% 输出结果
fprintf('\n========== 时域性能指标 ==========\n');
fprintf('稳态值（终值）         = %.4f m\n', C);
fprintf('峰值时间 tp            = %.4f s\n', peak_time);
fprintf('超调量 σ%%             = %.2f %%\n', overshoot);
fprintf('上升时间 tr (10%%~90%%) = %.4f s\n', rise_time);
fprintf('调节时间 ts (±2%%)      = %.4f s\n', settling_time);
fprintf('阶跃输入稳态误差 ess   = %.4f (理论应为0)\n', ess_step);

%% 4. 根轨迹分析
figure(2);
rlocus(G_open);
grid on;
title('进给伺服系统根轨迹图');
% 可额外计算临界增益（根轨迹与虚轴交点）
% 根据特征方程 1+G_open=0 => s^2*(Tm*s+1)+K=0 => Tm*s^3 + s^2 + K = 0
% 利用 rlocfind 交互式获取，也可以直接求解，这里仅作图。

%% 5. 频域分析
% 5.1 Bode图及稳定裕度
figure(3);
bode(G_open);
grid on;
title('进给伺服系统开环Bode图');
[Gm, Pm, Wcg, Wcp] = margin(G_open);

fprintf('\n========== 频域稳定裕度 ==========\n');
fprintf('幅值裕度 Gm = %.2f dB\n', 20*log10(Gm));
fprintf('相角裕度 Pm = %.2f deg\n', Pm);
fprintf('相角穿越频率 Wcg = %.4f rad/s\n', Wcg);
fprintf('幅值穿越频率 Wcp = %.4f rad/s\n', Wcp);

% 5.2 Nyquist图（补充，用于奈氏判据）
figure(4);
nyquist(G_open);
grid on;
title('进给伺服系统Nyquist图');
axis equal;

% 判断闭环稳定性：开环无右半平面极点（根据开环传递函数分母，极点为0（二重）和 -1/Tm，都在左半平面或虚轴但无正实部，故P=0），
% 若Nyquist曲线不包围(-1,j0)，则闭环稳定。
% 从稳定裕度看，Gm>0且Pm>0，系统稳定。
