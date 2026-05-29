%% =========================================================================
%  Intent-Biased Informed RRT* + TA-HDI-FMP (Refine Version)
%  - Lightweight post-first refinement for hybrid via points
%  - Intent Consistency Score (ICS) added for evaluation
% =========================================================================
clear; clc; close all;
script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir)
    script_dir = pwd;
end
addpath(fullfile(script_dir, 'func'), '-begin');

%% Unified parameters (all tunable)
cfg = struct();

% demo / intent
cfg.demo.demoLen = 150;      % 示教轨迹离散点数（越大时间分辨率越高，但计算更慢）
cfg.demo.demo_dt = 0.1;      % 示教轨迹采样周期，单位秒（总时长约 = demoLen * demo_dt）
cfg.demo.alpha = 0.1;        % FMP 时间特征指数权重（越大越强调后段时间差异）

% FMP / interpolation
cfg.fmp.N_C = 20;            % FMP 模糊聚类中心数（越大拟合能力更强，但更易过拟合且更慢）
cfg.fmp.dt = 0.05;           % 在线轨迹时间步长，单位秒（越小轨迹更细，数据量更大）
cfg.fmp.interp_dist = 0.5;   % 局部路径插值间距（越小 via 点更密、更平滑但更慢）
cfg.fmp.via_trim = 0.05;     % 去除分段首尾 via 点的时间边距，单位秒（避免拼接端点硬拐）
cfg.fmp.transition_ratio = 0.10; % Soft Boundary Blending 过渡区占总时长比例（可调 0.05/0.10）

cfg.fmp.transition_gamma = 1.0;  % boundary region preference to demo trajectory (>1 means stronger preference)

% robust defaults for backward compatibility
if ~isfield(cfg.fmp, 'transition_ratio')
    cfg.fmp.transition_ratio = 0.10;
end
if ~isfield(cfg.fmp, 'transition_gamma')
    cfg.fmp.transition_gamma = 1.0;
end

% environment
cfg.env.num_obs_target = 40; % 随机障碍物目标数量（越大场景更拥挤）
cfg.env.min_gap = 1.0;       % 障碍物最小间距（越大障碍越分散）
cfg.env.rng_seed = 2025;     % 随机种子（固定后结果可复现）
cfg.env.max_tries = 20000;   % 障碍物采样最大尝试次数（防止生成阶段死循环）
cfg.env.rrt_inflation = 0.5; % RRT* 碰撞膨胀半径（越大越保守更安全，但可行空间更小）
cfg.env.safe_margin = 0.2;   % 示教轨迹危险区判定安全边距（越大越容易触发局部重规划）

% rrt core
cfg.rrt.step_size = 1.5;       % RRT* 单步扩展长度（越大搜索更快但路径更折）
cfg.rrt.r_neighbor = 4.0;      % rewiring 邻域半径（越大重连更充分但更耗时）
cfg.rrt.max_iter = 6000;       % 单次 RRT* 最大迭代上限（防止最坏情况耗时过大）
cfg.rrt.edge_sample_step = 0.5;% 线段碰撞检测采样步长（越小检测更细更安全）
cfg.rrt.goal_bias_pre = 0.15;  % baseline 首解前目标偏置概率（越大越快连终点，但多样性下降）

% convergence stop (for baseline RRT*)
cfg.conv.W = 200;            % 收敛判定滑窗长度（迭代步）
cfg.conv.epsilon_rel = 1e-3; % 相对改进阈值（越小收敛判定越严格）
cfg.conv.M = 3;              % 连续命中收敛阈值次数（越大停机更稳但更慢）
cfg.conv.lambda_k = 0.15;    % 路径代价中的转角惩罚权重（越大越偏好平滑）

% lightweight refine
cfg.refine.enable_refine = true;                     % 是否启用首解后轻量优化
cfg.refine.budget_candidates = [25 50 100 150 200]; % 自动选预算候选（单位：首解后成功扩展次数）
cfg.refine.chosen_budget = 100;                      % 当前生效预算（用于日志和统计）
cfg.refine.fixed_budget = 100;                       % 固定预算值（非 NaN 时跳过自动搜索）

% intent bias sampling
cfg.intent_bias.enable = true;              % 是否启用意图偏置采样（hybrid 链路）
cfg.intent_bias.p_intent_pre = 0.55;        % 首解前意图采样概率（越大越贴示教）
cfg.intent_bias.p_goal_pre = 0.15;          % 首解前目标点采样概率（越大越偏向尽快连终点）
cfg.intent_bias.p_uniform_pre = 0.30;       % 首解前均匀采样概率（用于保持探索性）
cfg.intent_bias.p_intent_post = 0.65;       % 首解后意图采样概率（越大越保持示教趋势）
cfg.intent_bias.p_informed_post = 0.30;     % 首解后 informed 椭圆采样概率（越大收敛更快）
cfg.intent_bias.p_goal_post = 0.05;         % 首解后目标点采样概率（微量拉向终点）
cfg.intent_bias.sigma_intent = 1.2;         % 意图采样高斯扰动标准差（越大探索更散）
cfg.intent_bias.post_intent_max_retry = 10; % 首解后意图采样不满足约束时最大重试次数

% metrics
cfg.metrics.ics_mode = 'score01'; % ICS 评分模式（当前为 0~1，越大越像示教）
cfg.metrics.ics_eps = 1e-9;       % ICS 数值稳定项（防止除零）

%% 1. Offline intent learning
disp('1. Training offline FMP model...');
demoLen = cfg.demo.demoLen;
demo_dt = cfg.demo.demo_dt;
alpha = cfg.demo.alpha;
N_C = cfg.fmp.N_C;
dt = cfg.fmp.dt;

x_line = linspace(0, 100, demoLen);
y_line = 50 + 30 * sin(x_line * pi / 50);
my_demos{1}.pos = [x_line; y_line];

[Data, demo_dura] = demo_processing(my_demos, demoLen, demo_dt, alpha);

[C_x, pInvCov_x, p1_u_x] = fuzzymodellingCandGK(Data(1:3,:), demoLen, N_C, 30, 30);
[C_y, pInvCov_y, p1_u_y] = fuzzymodellingCandGK(Data([1 2 4],:), demoLen, N_C, 30, 30);

timeinput = dt:dt:demoLen*demo_dt;
N_Data = length(timeinput);

%% 2. Environment generation
disp('2. Generating random obstacle field...');
rng(cfg.env.rng_seed, 'twister');
num_obs_target = cfg.env.num_obs_target;
obstacles = zeros(num_obs_target, 6); % [x, y, s1, s2, theta, type]
count = 0;
tries = 0;

while count < num_obs_target && tries < cfg.env.max_tries
    tries = tries + 1;
    obs_x = 10 + 80 * rand();
    obs_y = 10 + 80 * rand();
    obs_type = randi([1, 2]);

    if obs_type == 1
        obs_r = 2 + 3 * rand();
        current_radius = obs_r;
        new_obs = [obs_x, obs_y, obs_r, 0, 0, 1];
    else
        obs_w = 4 + 6 * rand();
        obs_h = 2 + 5 * rand();
        obs_theta = rand() * pi;
        current_radius = sqrt((obs_w/2)^2 + (obs_h/2)^2);
        new_obs = [obs_x, obs_y, obs_w, obs_h, obs_theta, 2];
    end

    overlap = false;
    for j = 1:count
        if obstacles(j, 6) == 1
            r_exist = obstacles(j, 3);
        else
            r_exist = sqrt((obstacles(j,3)/2)^2 + (obstacles(j,4)/2)^2);
        end
        if norm([obs_x, obs_y] - obstacles(j, 1:2)) < (current_radius + r_exist + cfg.env.min_gap)
            overlap = true;
            break;
        end
    end

    if ~overlap
        count = count + 1;
        obstacles(count, :) = new_obs;
    end
end

num_obs = count;

%% 3. Baseline (global, fully independent) + Hybrid (segmented)
disp('3. Running independent global baseline + segmented hybrid...');

% ---- baseline: fully independent global Informed RRT* (no segmentation) ----
global_start = [x_line(1), y_line(1)];
global_goal = [x_line(end), y_line(end)];
baseline_seed = cfg.env.rng_seed + 11;
base_plan_global = get_baseline_informed_rrt_paths(global_start, global_goal, obstacles, cfg, baseline_seed);

if isempty(base_plan_global.path_conv)
    baseline_traj_x = x_line;
    baseline_traj_y = y_line;
    T_explore = 0;
    T_rrt_opt = 0;
else
    baseline_traj_x = base_plan_global.path_conv(1,:);
    baseline_traj_y = base_plan_global.path_conv(2,:);
    T_explore = base_plan_global.t_first;
    T_rrt_opt = max(0, base_plan_global.t_conv - base_plan_global.t_first);
end

% ---- hybrid path still uses segmented local replanning to generate vias ----
danger_indices = [];
for k = 1:num_obs
    type = obstacles(k, 6);
    cx = obstacles(k, 1);
    cy = obstacles(k, 2);
    for j = 1:demoLen
        px = x_line(j);
        py = y_line(j);
        if type == 1
            if norm([px-cx, py-cy]) < (obstacles(k,3) + cfg.env.safe_margin)
                danger_indices = [danger_indices, j];
            end
        else
            theta = obstacles(k,5);
            w = obstacles(k,3)/2 + cfg.env.safe_margin;
            h = obstacles(k,4)/2 + cfg.env.safe_margin;
            dx = px - cx;
            dy = py - cy;
            lx = dx * cos(-theta) - dy * sin(-theta);
            ly = dx * sin(-theta) + dy * cos(-theta);
            if abs(lx) < w && abs(ly) < h
                danger_indices = [danger_indices, j];
            end
        end
    end
end
danger_indices = unique(sort(danger_indices));

all_via_points = [];
all_via_times = [];
total_t_refine = 0;
segment_mask = false(1, demoLen);   % true: obstacle-replanning zone
boundary_idx = [];

if ~isempty(danger_indices)
    segments = {};
    curr_seg = danger_indices(1);
    for i = 2:length(danger_indices)
        if danger_indices(i) - danger_indices(i-1) > 10
            segments{end+1} = curr_seg;
            curr_seg = danger_indices(i);
        else
            curr_seg = [curr_seg, danger_indices(i)];
        end
    end
    segments{end+1} = curr_seg;

    budget_locked = false;
    for s = 1:length(segments)
        seg = segments{s};
        idx_start = max(1, seg(1) - 4);
        idx_goal = min(demoLen, seg(end) + 4);
        segment_mask(idx_start:idx_goal) = true;
        boundary_idx = [boundary_idx, idx_start, idx_goal];
        local_start = [x_line(idx_start), y_line(idx_start)];
        local_goal = [x_line(idx_goal), y_line(idx_goal)];
        intent_local = [x_line(idx_start:idx_goal); y_line(idx_start:idx_goal)];

        segment_seed = cfg.env.rng_seed + s * 1000;
        hybrid_seed = segment_seed + 2;

        if cfg.refine.enable_refine && ~budget_locked
            if isfield(cfg.refine, 'fixed_budget') && ~isnan(cfg.refine.fixed_budget)
                cfg.refine.chosen_budget = cfg.refine.fixed_budget;
                budget_locked = true;
                fprintf('   Fixed refine budget: %d\n', cfg.refine.chosen_budget);
            else
                [cfg.refine.chosen_budget, sweep_log] = auto_select_refine_budget(local_start, local_goal, intent_local, obstacles, cfg, hybrid_seed);
                budget_locked = true;
                disp('   Budget auto-selection (Pareto+knee):');
                disp(sweep_log);
                fprintf('   Chosen refine budget: %d\n', cfg.refine.chosen_budget);
            end
        end
        if ~cfg.refine.enable_refine
            cfg.refine.chosen_budget = 0;
        end

        hy_plan = get_intent_biased_rrt_paths(local_start, local_goal, intent_local, obstacles, cfg, cfg.refine.chosen_budget, hybrid_seed);
        if isempty(hy_plan.path_conv)
            continue;
        end
        total_t_refine = total_t_refine + max(0, hy_plan.t_refine - hy_plan.t_first);

        if isempty(hy_plan.path_refine)
            path_for_via = hy_plan.path_first;
        else
            path_for_via = hy_plan.path_refine;
        end

        dense_path = [];
        for p = 1:(size(path_for_via,2)-1)
            pt1 = path_for_via(:, p);
            pt2 = path_for_via(:, p+1);
            num_interp = max(2, ceil(norm(pt2 - pt1) / cfg.fmp.interp_dist));
            xx = linspace(pt1(1), pt2(1), num_interp);
            yy = linspace(pt1(2), pt2(2), num_interp);
            if p < size(path_for_via,2)-1
                dense_path = [dense_path, [xx(1:end-1); yy(1:end-1)]];
            else
                dense_path = [dense_path, [xx; yy]];
            end
        end

        t_start = idx_start * demo_dt;
        t_goal = idx_goal * demo_dt;
        dist_accum = [0, cumsum(sqrt(sum(diff(dense_path, 1, 2).^2, 1)))];
        t_local = t_start + (dist_accum / max(eps, dist_accum(end))) * (t_goal - t_start);

        valid_t_mask = (t_local > t_start + cfg.fmp.via_trim) & (t_local < t_goal - cfg.fmp.via_trim);
        all_via_points = [all_via_points, dense_path(:, valid_t_mask)];
        all_via_times = [all_via_times, t_local(valid_t_mask)];
    end
end

%% 4. FMP modulation
disp('4. Running algebraic FMP smooth modulation...');
t_start_fmp = tic;
if ~isempty(all_via_points)
    [all_via_times, unique_idx] = unique(all_via_times);
    all_via_points = all_via_points(:, unique_idx);

    via_time_scaled = (demoLen*demo_dt*demo_dura) * (all_via_times / (demoLen*demo_dt));
    y_mod_x = call_fmp_modulation(timeinput, demo_dura, alpha, N_Data, N_C, C_x, pInvCov_x, p1_u_x, [1 2], [3], via_time_scaled, all_via_points(1,:), 1, cfg.fmp.transition_ratio, cfg.fmp.transition_gamma);
    y_mod_y = call_fmp_modulation(timeinput, demo_dura, alpha, N_Data, N_C, C_y, pInvCov_y, p1_u_y, [1 2], [3], via_time_scaled, all_via_points(2,:), 1, cfg.fmp.transition_ratio, cfg.fmp.transition_gamma);
else
    y_mod_x = x_line;
    y_mod_y = y_line;
end
t_fmp_optimize = toc(t_start_fmp);

%% 4.5 局部折角检测与精准平滑 (Local Corner Smoothing)
disp('4.5 Running local corner detection and smoothing...');

% 1. 计算轨迹的航向角 (Heading / Yaw)
dx = gradient(y_mod_x);
dy = gradient(y_mod_y);
headings = atan2(dy, dx);

% 2. 计算相邻点航向角的绝对变化率 (检测折线)
d_headings = abs(diff(headings));
% 处理角度在 -pi 到 pi 之间的翻转问题
d_headings = min(d_headings, 2*pi - d_headings);
d_headings = [0, d_headings]; % 补齐数组长度

% ------------------- 核心可调参数 -------------------
% angle_threshold: 判定为“折线”的角度突变阈值。0.05 弧度约等于 2.8 度
angle_threshold = 0.01;  
% local_window: 发现折线点后，向前向后抹平多大范围（越大倒角越圆，越小越贴近原路径）
local_window = 0.1;       
% --------------------------------------------------

% 3. 找出所有超过阈值的“折线突变点”
sharp_indices = find(d_headings > angle_threshold);

if ~isempty(sharp_indices)
    fprintf('   Detected %d sharp points. Applying local filleting...\n', length(sharp_indices));
    
    % 4. 创建一个逻辑掩码(Mask)，只标记需要平滑的“一小段”区域
    smooth_mask = false(size(y_mod_x));
    for i = 1:length(sharp_indices)
        idx = sharp_indices(i);
        % 限制窗口边界，防止索引越界或修改起点/终点
        start_idx = max(2, idx - local_window);       
        end_idx = min(length(y_mod_x)-1, idx + local_window); 
        smooth_mask(start_idx:end_idx) = true;
    end
    
    % 5. 提取出所有被标记的独立“折角区域”
    regions = bwconncomp(smooth_mask);
    
    % 6. 只针对这些被挑出来的区域执行高斯平滑
    for r = 1:regions.NumObjects
        idx_range = regions.PixelIdxList{r}';
        smooth_len = length(idx_range);
        
        % 'gaussian' 滤波能够形成非常完美的 S 型软过渡，且绝不影响非标记区
        y_mod_x(idx_range) = smoothdata(y_mod_x(idx_range), 'gaussian', smooth_len);
        y_mod_y(idx_range) = smoothdata(y_mod_y(idx_range), 'gaussian', smooth_len);
    end
else
    disp('   No sharp corners detected. Trajectory is already globally smooth.');
end



%% 5. Quantitative report: time + jerk + ICS
disp('5. Computing quantitative report...');
T_hybrid_opt = total_t_refine + t_fmp_optimize;
Speedup_opt = T_rrt_opt / max(T_hybrid_opt, eps);

baseline_xy = [baseline_traj_x; baseline_traj_y];
hybrid_xy = [y_mod_x; y_mod_y];
intent_xy = [x_line; y_line];

% jerk
dist_base = [0, cumsum(sqrt(diff(baseline_traj_x).^2 + diff(baseline_traj_y).^2))];
dist_query = linspace(0, dist_base(end), N_Data);
base_x_eval = interp1(dist_base, baseline_traj_x, dist_query, 'linear');
base_y_eval = interp1(dist_base, baseline_traj_y, dist_query, 'linear');
j_rrt = sum(sum(diff(diff(diff([base_x_eval; base_y_eval], 1, 2)/dt, 1, 2)/dt, 1, 2).^2, 1)) * dt;
j_hybrid = sum(sum(diff(diff(diff(hybrid_xy, 1, 2)/dt, 1, 2)/dt, 1, 2).^2, 1)) * dt;

% ICS
d_ref = norm([max(intent_xy(1,:)) - min(intent_xy(1,:)), max(intent_xy(2,:)) - min(intent_xy(2,:))]);
ICS_rrt = compute_intent_consistency_score(baseline_xy, intent_xy, N_Data, d_ref, cfg.metrics.ics_eps);
ICS_hybrid = compute_intent_consistency_score(hybrid_xy, intent_xy, N_Data, d_ref, cfg.metrics.ics_eps);
ICS_gain = ICS_hybrid - ICS_rrt;

if ICS_hybrid < ICS_rrt
    warning('ICS_hybrid (%.4f) < ICS_rrt (%.4f), hybrid intent similarity did not improve in this run.', ICS_hybrid, ICS_rrt);
end

disp('================================================================');
disp(' Baseline (No-Intent Bias) vs Intent-Biased Hybrid');
disp('================================================================');
fprintf('[Time Decomposition]\n');
fprintf(' - T_explore: %.1f ms\n', T_explore * 1000);
fprintf(' - T_rrt_opt: %.1f ms\n', T_rrt_opt * 1000);
fprintf(' - T_refine_hybrid: %.1f ms\n', total_t_refine * 1000);
fprintf(' - T_fmp_opt: %.1f ms\n', t_fmp_optimize * 1000);
fprintf(' - T_hybrid_opt = T_refine_hybrid + T_fmp_opt: %.1f ms\n', T_hybrid_opt * 1000);
fprintf(' - Speedup_opt = T_rrt_opt / T_hybrid_opt: %.2f x\n\n', Speedup_opt);

fprintf('[Jerk]\n');
fprintf(' - Jerk_rrt (No-Intent Bias baseline): %.2e\n', j_rrt);
fprintf(' - Jerk_hybrid: %.2e\n', j_hybrid);
fprintf(' - Smoothness gain (J_rrt / J_hybrid): %.2f x\n\n', j_rrt / max(j_hybrid, eps));

fprintf('[Intent Consistency Score, ICS]\n');
fprintf(' - ICS_rrt: %.4f\n', ICS_rrt);
fprintf(' - ICS_hybrid: %.4f\n', ICS_hybrid);
fprintf(' - ICS_gain = ICS_hybrid - ICS_rrt: %.4f\n', ICS_gain);
if ~isnan(cfg.refine.chosen_budget)
    fprintf(' - Chosen refine budget: %d\n', cfg.refine.chosen_budget);
end
disp('================================================================');

%% 6. Visualization
figure('Name', 'Trajectory Comparison', 'Position', [100, 100, 820, 620], 'Color', 'w');
hold on; grid on; axis equal;
for k = 1:num_obs
    fc = [1 0.8 0.8];
    ec = [0.8 0.5 0.5];
    if obstacles(k, 6) == 1
        rectangle('Position', [obstacles(k,1)-obstacles(k,3), obstacles(k,2)-obstacles(k,3), obstacles(k,3)*2, obstacles(k,3)*2], 'Curvature', 1, 'FaceColor', fc, 'EdgeColor', ec);
    else
        w = obstacles(k,3);
        h = obstacles(k,4);
        theta = obstacles(k,5);
        R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
        c = [-w/2 -w/2 w/2 w/2; -h/2 h/2 h/2 -h/2];
        rc = R * c;
        patch(rc(1,:)+obstacles(k,1), rc(2,:)+obstacles(k,2), fc, 'EdgeColor', ec);
    end
end
h1 = plot(x_line, y_line, 'k--', 'LineWidth', 1.5);
h2 = plot(baseline_traj_x, baseline_traj_y, 'b-.', 'LineWidth', 1.2);
h3 = plot(y_mod_x, y_mod_y, 'g-', 'LineWidth', 2.5);
if ~isempty(all_via_points)
    h4 = plot(all_via_points(1,:), all_via_points(2,:), 'r.', 'MarkerSize', 8);
    legend([h1 h2 h3 h4], {'Nominal Intent', 'Informed RRT* (No-Intent Bias, Convergence-Stop)', 'Intent-Biased Informed RRT* + TA-HDI-FMP', 'Hybrid Via Points'}, 'Location', 'best');
else
    legend([h1 h2 h3], {'Nominal Intent', 'Informed RRT* (No-Intent Bias, Convergence-Stop)', 'Intent-Biased Informed RRT* + TA-HDI-FMP'}, 'Location', 'best');
end
title('Obstacle Avoidance Trajectory Comparison');

figure('Name', 'Time Decomposition', 'Position', [950, 100, 620, 420], 'Color', 'w');
bar_data = [T_explore*1000, T_rrt_opt*1000; T_explore*1000, T_hybrid_opt*1000];
bar(bar_data, 'stacked');
set(gca, 'XTickLabel', {'Informed RRT* (No-Intent Bias, Conv-Stop)', 'Intent-Biased Informed RRT* + TA-HDI-FMP'});
ylabel('Planning Latency (ms)');
title('Time Decomposition: Exploration vs Optimization');
legend({'Exploration', 'Optimization'}, 'Location', 'northwest');
grid on;

figure('Name', 'Intent Consistency Score (ICS)', 'Position', [950, 560, 420, 300], 'Color', 'w');
bar([ICS_rrt, ICS_hybrid]);
set(gca, 'XTickLabel', {'RRT* Baseline', 'Intent-Biased Hybrid'});
ylabel('ICS (higher is better)');
title('Intent Similarity Comparison');
grid on;

%% ============================ Helper functions ============================
function [best_budget, sweep_log] = auto_select_refine_budget(local_start, local_goal, intent_local, obstacles, cfg, base_seed)
    budgets = cfg.refine.budget_candidates;
    n = numel(budgets);
    gains = nan(1, n);
    deltas = nan(1, n);

    for i = 1:n
        plan_i = get_intent_biased_rrt_paths(local_start, local_goal, intent_local, obstacles, cfg, budgets(i), base_seed);
        if isempty(plan_i.path_first) || isempty(plan_i.path_refine)
            continue;
        end
        j_first = path_objective(plan_i.path_first, cfg.conv.lambda_k);
        j_refine = path_objective(plan_i.path_refine, cfg.conv.lambda_k);
        gains(i) = (j_first - j_refine) / max(j_first, eps);
        deltas(i) = max(0, plan_i.t_refine - plan_i.t_first);
    end

    valid = find(~isnan(gains) & ~isnan(deltas));
    if isempty(valid)
        best_budget = budgets(min(find(budgets == 100, 1), numel(budgets)));
        sweep_log = table(budgets(:), gains(:), deltas(:), 'VariableNames', {'budget', 'gain', 'delta_t'});
        return;
    end

    b = budgets(valid);
    g = gains(valid);
    t = deltas(valid);

    pick_local = choose_refine_budget_pareto(b, g, t);
    best_budget = b(pick_local);

    sweep_log = table(budgets(:), gains(:), deltas(:), 'VariableNames', {'budget', 'gain', 'delta_t'});
end

function pick_idx = choose_refine_budget_pareto(budgets, gains, deltas)
    n = numel(budgets);
    is_pareto = true(1, n);

    for i = 1:n
        for j = 1:n
            if i == j
                continue;
            end
            better_or_equal = (gains(j) >= gains(i)) && (deltas(j) <= deltas(i));
            strictly_better = (gains(j) > gains(i)) || (deltas(j) < deltas(i));
            if better_or_equal && strictly_better
                is_pareto(i) = false;
                break;
            end
        end
    end

    p_idx = find(is_pareto);
    pb = budgets(p_idx);
    pg = gains(p_idx);
    pt = deltas(p_idx);

    [pt, order] = sort(pt, 'ascend');
    pb = pb(order);
    pg = pg(order);

    if numel(pb) == 1
        pick_idx = p_idx(order(1));
        return;
    end

    pt_n = (pt - min(pt)) / max(max(pt) - min(pt), eps);
    pg_n = (pg - min(pg)) / max(max(pg) - min(pg), eps);

    p1 = [pt_n(1), pg_n(1)];
    p2 = [pt_n(end), pg_n(end)];
    v = p2 - p1;

    if norm(v) < eps
        [~, k] = max(pg);
        target_budget = pb(k);
        pick_idx = find(budgets == target_budget, 1, 'first');
        return;
    end

    dist = zeros(1, numel(pb));
    for i = 1:numel(pb)
        p = [pt_n(i), pg_n(i)];
        dist(i) = abs(det([v; p - p1])) / norm(v);
    end

    [max_dist, k] = max(dist);
    if max_dist < 1e-4
        gain_thr = 0.9 * max(pg);
        idx_ok = find(pg >= gain_thr);
        [~, k2] = min(pt(idx_ok));
        target_budget = pb(idx_ok(k2));
    else
        target_budget = pb(k);
    end

    pick_idx = find(budgets == target_budget, 1, 'first');
end

function plan = get_baseline_informed_rrt_paths(start_pos, goal_pos, obstacles, cfg, seed)
    rng(seed, 'twister');

    step_size = cfg.rrt.step_size;
    r_neighbor = cfg.rrt.r_neighbor;
    max_iter = cfg.rrt.max_iter;

    W = cfg.conv.W;
    epsilon_rel = cfg.conv.epsilon_rel;
    M = cfg.conv.M;
    lambda_k = cfg.conv.lambda_k;

    num_obs = size(obstacles, 1); %#ok<NASGU>
    c_best = inf;
    c_min = norm(goal_pos - start_pos);

    x_center = (start_pos + goal_pos) / 2;
    dir = (goal_pos - start_pos) / max(c_min, eps);
    angle = atan2(dir(2), dir(1));
    C_mat = [cos(angle) -sin(angle); sin(angle) cos(angle)];

    tree = [start_pos(1), start_pos(2), 0, 0]; % [x, y, parent, cost]
    x_min = min(start_pos(1), goal_pos(1)) - 20;
    x_max = max(start_pos(1), goal_pos(1)) + 20;
    y_min = min(start_pos(2), goal_pos(2)) - 20;
    y_max = max(start_pos(2), goal_pos(2)) + 20;

    t_start_rrt = tic;
    goal_idx = -1;
    found_first = false;
    t_first = NaN;
    path_first = [];

    best_J_hist = inf(1, max_iter);
    conv_hits = 0;
    stop_reason = 'max_iter_guard';
    iter_end = max_iter;

    for iter = 1:max_iter
        if c_best < inf
            rand_node = sample_informed_node(x_center, C_mat, c_best, c_min);
        else
            if rand < cfg.rrt.goal_bias_pre
                rand_node = goal_pos;
            else
                rand_node = [x_min + rand()*(x_max - x_min), y_min + rand()*(y_max - y_min)];
            end
        end

        dist = sqrt((tree(:,1)-rand_node(1)).^2 + (tree(:,2)-rand_node(2)).^2);
        [~, nearest_idx] = min(dist);
        nearest_node = tree(nearest_idx, 1:2);

        theta_step = atan2(rand_node(2)-nearest_node(2), rand_node(1)-nearest_node(1));
        new_node = nearest_node + [step_size*cos(theta_step), step_size*sin(theta_step)];

        if ~is_point_collision_free(new_node, obstacles, cfg.env.rrt_inflation) || ...
           ~is_edge_collision_free(nearest_node, new_node, obstacles, cfg.env.rrt_inflation, cfg.rrt.edge_sample_step)
            if iter > 1
                best_J_hist(iter) = best_J_hist(iter-1);
            end
            continue;
        end

        dist_to_all = sqrt((tree(:,1)-new_node(1)).^2 + (tree(:,2)-new_node(2)).^2);
        neighbor_indices = find(dist_to_all <= r_neighbor);

        best_parent_idx = nearest_idx;
        min_cost = tree(nearest_idx, 4) + dist_to_all(nearest_idx);
        for i = 1:length(neighbor_indices)
            idx = neighbor_indices(i);
            cost_temp = tree(idx, 4) + dist_to_all(idx);
            if cost_temp < min_cost && is_edge_collision_free(tree(idx,1:2), new_node, obstacles, cfg.env.rrt_inflation, cfg.rrt.edge_sample_step)
                best_parent_idx = idx;
                min_cost = cost_temp;
            end
        end

        new_idx = size(tree, 1) + 1;
        tree(new_idx, :) = [new_node, best_parent_idx, min_cost];

        for i = 1:length(neighbor_indices)
            idx = neighbor_indices(i);
            if idx == best_parent_idx
                continue;
            end
            cost_via_new = min_cost + norm(tree(idx,1:2) - new_node);
            if cost_via_new < tree(idx, 4) && is_edge_collision_free(new_node, tree(idx,1:2), obstacles, cfg.env.rrt_inflation, cfg.rrt.edge_sample_step)
                tree(idx, 3) = new_idx;
                tree(idx, 4) = cost_via_new;
            end
        end

        if norm(new_node - goal_pos) <= step_size && is_edge_collision_free(new_node, goal_pos, obstacles, cfg.env.rrt_inflation, cfg.rrt.edge_sample_step)
            cost_to_goal = min_cost + norm(new_node - goal_pos);
            if cost_to_goal < c_best
                c_best = cost_to_goal;
                goal_idx = new_idx;
                if ~found_first
                    found_first = true;
                    t_first = toc(t_start_rrt);
                    path_first = backtrack_path(tree, goal_idx, goal_pos);
                end
            end
        end

        if found_first
            best_path_now = backtrack_path(tree, goal_idx, goal_pos);
            best_J_hist(iter) = path_objective(best_path_now, lambda_k);

            if mod(iter, W) == 0 && iter >= 2*W
                J_prev = best_J_hist(iter - W);
                J_curr = best_J_hist(iter);
                rel_improve = (J_prev - J_curr) / max(J_prev, eps);
                if rel_improve < epsilon_rel
                    conv_hits = conv_hits + 1;
                else
                    conv_hits = 0;
                end
                if conv_hits >= M
                    stop_reason = 'converged';
                    iter_end = iter;
                    break;
                end
            end
        elseif iter > 1
            best_J_hist(iter) = best_J_hist(iter-1);
        end
    end

    t_conv = toc(t_start_rrt);
    if ~found_first
        plan = struct('path_first', [], 'path_conv', [], 't_first', NaN, 't_conv', t_conv, 'stop_reason', 'failed');
        fprintf('[Baseline RRT*] stop=max_iter_guard, iter=%d, no feasible path found.\n', max_iter);
        return;
    end

    path_conv = backtrack_path(tree, goal_idx, goal_pos);
    if strcmp(stop_reason, 'converged')
        fprintf('[Baseline RRT*] stop=converged, iter=%d, t_first=%.1fms, t_conv=%.1fms\n', iter_end, t_first*1000, t_conv*1000);
    else
        fprintf('[Baseline RRT*] stop=max_iter_guard, iter=%d, t_first=%.1fms, t_conv=%.1fms\n', max_iter, t_first*1000, t_conv*1000);
    end

    plan = struct('path_first', path_first, 'path_conv', path_conv, 't_first', t_first, 't_conv', t_conv, 'stop_reason', stop_reason);
end

function plan = get_intent_biased_rrt_paths(start_pos, goal_pos, intent_path, obstacles, cfg, refine_budget, seed)
    rng(seed, 'twister');

    step_size = cfg.rrt.step_size;
    r_neighbor = cfg.rrt.r_neighbor;
    max_iter = cfg.rrt.max_iter;

    W = cfg.conv.W;
    epsilon_rel = cfg.conv.epsilon_rel;
    M = cfg.conv.M;
    lambda_k = cfg.conv.lambda_k;

    num_obs = size(obstacles, 1);
    c_best = inf;
    c_min = norm(goal_pos - start_pos);

    x_center = (start_pos + goal_pos) / 2;
    dir = (goal_pos - start_pos) / max(c_min, eps);
    angle = atan2(dir(2), dir(1));
    C_mat = [cos(angle) -sin(angle); sin(angle) cos(angle)];

    tree = [start_pos(1), start_pos(2), 0, 0]; % [x, y, parent, cost]

    x_min = min(start_pos(1), goal_pos(1)) - 20;
    x_max = max(start_pos(1), goal_pos(1)) + 20;
    y_min = min(start_pos(2), goal_pos(2)) - 20;
    y_max = max(start_pos(2), goal_pos(2)) + 20;

    t_start_rrt = tic;
    goal_idx = -1;
    found_first = false;
    t_first = NaN;
    path_first = [];

    path_refine = [];
    t_refine = NaN;
    refine_hit = false;
    refine_expand_count = 0;

    best_J_hist = inf(1, max_iter);
    conv_hits = 0;
    stop_reason = 'max_iter_guard';
    iter_end = max_iter;

    for iter = 1:max_iter
        [rand_node, sampled_informed] = sample_intent_biased_node(found_first, c_best, start_pos, goal_pos, intent_path, x_center, C_mat, c_min, x_min, x_max, y_min, y_max, cfg);
        if ~sampled_informed && found_first && isfinite(c_best) && (norm(rand_node - start_pos) + norm(rand_node - goal_pos) > c_best)
            rand_node = sample_informed_node(x_center, C_mat, c_best, c_min);
        end

        dist = sqrt((tree(:,1)-rand_node(1)).^2 + (tree(:,2)-rand_node(2)).^2);
        [~, nearest_idx] = min(dist);
        nearest_node = tree(nearest_idx, 1:2);

        theta_step = atan2(rand_node(2)-nearest_node(2), rand_node(1)-nearest_node(1));
        new_node = nearest_node + [step_size*cos(theta_step), step_size*sin(theta_step)];

        if ~is_point_collision_free(new_node, obstacles, cfg.env.rrt_inflation) || ...
           ~is_edge_collision_free(nearest_node, new_node, obstacles, cfg.env.rrt_inflation, cfg.rrt.edge_sample_step)
            if iter > 1
                best_J_hist(iter) = best_J_hist(iter-1);
            end
            continue;
        end

        dist_to_all = sqrt((tree(:,1)-new_node(1)).^2 + (tree(:,2)-new_node(2)).^2);
        neighbor_indices = find(dist_to_all <= r_neighbor);

        best_parent_idx = nearest_idx;
        min_cost = tree(nearest_idx, 4) + dist_to_all(nearest_idx);
        for i = 1:length(neighbor_indices)
            idx = neighbor_indices(i);
            cost_temp = tree(idx, 4) + dist_to_all(idx);
            if cost_temp < min_cost && is_edge_collision_free(tree(idx,1:2), new_node, obstacles, cfg.env.rrt_inflation, cfg.rrt.edge_sample_step)
                best_parent_idx = idx;
                min_cost = cost_temp;
            end
        end

        new_idx = size(tree, 1) + 1;
        tree(new_idx, :) = [new_node, best_parent_idx, min_cost];

        for i = 1:length(neighbor_indices)
            idx = neighbor_indices(i);
            if idx == best_parent_idx
                continue;
            end
            cost_via_new = min_cost + norm(tree(idx,1:2) - new_node);
            if cost_via_new < tree(idx, 4) && is_edge_collision_free(new_node, tree(idx,1:2), obstacles, cfg.env.rrt_inflation, cfg.rrt.edge_sample_step)
                tree(idx, 3) = new_idx;
                tree(idx, 4) = cost_via_new;
            end
        end

        if found_first
            refine_expand_count = refine_expand_count + 1;
            if cfg.refine.enable_refine && ~refine_hit && refine_expand_count >= max(refine_budget, 0)
                path_refine = backtrack_path(tree, goal_idx, goal_pos);
                t_refine = toc(t_start_rrt);
                refine_hit = true;
                stop_reason = 'budget_hit';
            end
        end

        if norm(new_node - goal_pos) <= step_size && is_edge_collision_free(new_node, goal_pos, obstacles, cfg.env.rrt_inflation, cfg.rrt.edge_sample_step)
            cost_to_goal = min_cost + norm(new_node - goal_pos);
            if cost_to_goal < c_best
                c_best = cost_to_goal;
                goal_idx = new_idx;
                if ~found_first
                    found_first = true;
                    t_first = toc(t_start_rrt);
                    path_first = backtrack_path(tree, goal_idx, goal_pos);
                    if ~cfg.refine.enable_refine || refine_budget == 0
                        path_refine = path_first;
                        t_refine = t_first;
                        refine_hit = true;
                    end
                end
            end
        end

        if found_first
            best_path_now = backtrack_path(tree, goal_idx, goal_pos);
            best_J_hist(iter) = path_objective(best_path_now, lambda_k);

            if mod(iter, W) == 0 && iter >= 2*W
                J_prev = best_J_hist(iter - W);
                J_curr = best_J_hist(iter);
                rel_improve = (J_prev - J_curr) / max(J_prev, eps);

                if rel_improve < epsilon_rel
                    conv_hits = conv_hits + 1;
                else
                    conv_hits = 0;
                end

                if conv_hits >= M
                    stop_reason = 'converged';
                    iter_end = iter;
                    break;
                end
            end
        elseif iter > 1
            best_J_hist(iter) = best_J_hist(iter-1);
        end
    end

    t_conv = toc(t_start_rrt);

    if ~found_first
        plan = struct('path_first', [], 'path_refine', [], 'path_conv', [], 't_first', NaN, 't_refine', NaN, 't_conv', t_conv, 'stop_reason', 'failed');
        fprintf('[RRT* monitor] stop=max_iter_guard, iter=%d, no feasible path found.\n', max_iter);
        return;
    end

    path_conv = backtrack_path(tree, goal_idx, goal_pos);

    if isempty(path_refine)
        path_refine = path_conv;
        t_refine = t_conv;
    end

    if isnan(t_refine)
        t_refine = t_conv;
    end

    if t_refine < t_first
        t_refine = t_first;
    end
    if t_refine > t_conv
        t_refine = t_conv;
    end

    if strcmp(stop_reason, 'converged')
        fprintf('[RRT* monitor] stop=converged, iter=%d, t_first=%.1fms, t_refine=%.1fms, t_conv=%.1fms\n', iter_end, t_first*1000, t_refine*1000, t_conv*1000);
    elseif strcmp(stop_reason, 'budget_hit')
        fprintf('[RRT* monitor] stop=budget_hit, iter=%d, t_first=%.1fms, t_refine=%.1fms, t_conv=%.1fms\n', iter, t_first*1000, t_refine*1000, t_conv*1000);
    else
        fprintf('[RRT* monitor] stop=max_iter_guard, iter=%d, t_first=%.1fms, t_refine=%.1fms, t_conv=%.1fms\n', max_iter, t_first*1000, t_refine*1000, t_conv*1000);
    end

    plan = struct('path_first', path_first, 'path_refine', path_refine, 'path_conv', path_conv, ...
                  't_first', t_first, 't_refine', t_refine, 't_conv', t_conv, ...
                  'stop_reason', stop_reason);
end

function [node, is_informed] = sample_intent_biased_node(found_first, c_best, start_pos, goal_pos, intent_path, x_center, C_mat, c_min, x_min, x_max, y_min, y_max, cfg)
    is_informed = false;
    use_intent = cfg.intent_bias.enable && ~isempty(intent_path);

    if ~found_first
        r = rand();
        p_intent = cfg.intent_bias.p_intent_pre;
        p_goal = cfg.intent_bias.p_goal_pre;

        if use_intent && (r < p_intent)
            idx = randi(size(intent_path, 2));
            node = intent_path(:, idx)' + cfg.intent_bias.sigma_intent * randn(1,2);
        elseif r < (p_intent + p_goal)
            node = goal_pos;
        else
            node = [x_min + rand()*(x_max - x_min), y_min + rand()*(y_max - y_min)];
        end
    else
        r = rand();
        p_intent = cfg.intent_bias.p_intent_post;
        p_informed = cfg.intent_bias.p_informed_post;
        p_goal = cfg.intent_bias.p_goal_post;

        if use_intent && (r < p_intent)
            ok = false;
            for t = 1:cfg.intent_bias.post_intent_max_retry
                idx = randi(size(intent_path, 2));
                cand = intent_path(:, idx)' + cfg.intent_bias.sigma_intent * randn(1,2);
                if ~isfinite(c_best) || is_inside_informed(cand, start_pos, goal_pos, c_best)
                    node = cand;
                    ok = true;
                    break;
                end
            end
            if ~ok
                node = sample_informed_node(x_center, C_mat, c_best, c_min);
                is_informed = true;
            end
        elseif r < (p_intent + p_informed)
            node = sample_informed_node(x_center, C_mat, c_best, c_min);
            is_informed = true;
        elseif r < (p_intent + p_informed + p_goal)
            node = goal_pos;
        else
            node = [x_min + rand()*(x_max - x_min), y_min + rand()*(y_max - y_min)];
        end
    end
end

function node = sample_informed_node(x_center, C_mat, c_best, c_min)
    if ~isfinite(c_best)
        node = x_center;
        return;
    end
    r1 = c_best / 2;
    r2 = sqrt(max(c_best^2 - c_min^2, eps)) / 2;
    L_mat = [r1 0; 0 r2];
    rr = sqrt(rand());
    tt = 2*pi*rand();
    x_ball = [rr*cos(tt); rr*sin(tt)];
    node = (C_mat * L_mat * x_ball + x_center')';
end

function inside = is_inside_informed(pt, start_pos, goal_pos, c_best)
    inside = (norm(pt - start_pos) + norm(pt - goal_pos)) <= (c_best + 1e-9);
end

function path = backtrack_path(tree, goal_idx, goal_pos)
    curr_idx = goal_idx;
    path = goal_pos;
    while curr_idx > 0
        path = [tree(curr_idx, 1:2); path];
        curr_idx = tree(curr_idx, 3);
    end
    path = path';
end

function J = path_objective(path_xy, lambda_k)
    if size(path_xy, 2) < 2
        J = inf;
        return;
    end
    seg = diff(path_xy, 1, 2);
    seg_len = sqrt(sum(seg.^2, 1));
    L = sum(seg_len);

    if size(seg, 2) < 2
        J = L;
        return;
    end

    theta = atan2(seg(2,:), seg(1,:));
    dtheta = diff(theta);
    dtheta = atan2(sin(dtheta), cos(dtheta));
    J = L + lambda_k * sum(abs(dtheta));
end

function ok = is_point_collision_free(pt, obstacles, rrt_inflation)
    ok = true;
    for k = 1:size(obstacles, 1)
        type = obstacles(k, 6);
        cx = obstacles(k, 1);
        cy = obstacles(k, 2);

        if type == 1
            if norm(pt - [cx, cy]) <= (obstacles(k,3) + rrt_inflation)
                ok = false;
                return;
            end
        else
            theta_obb = obstacles(k,5);
            hw = obstacles(k,3)/2 + rrt_inflation;
            hh = obstacles(k,4)/2 + rrt_inflation;
            dx = pt(1) - cx;
            dy = pt(2) - cy;
            lx = dx * cos(-theta_obb) - dy * sin(-theta_obb);
            ly = dx * sin(-theta_obb) + dy * cos(-theta_obb);
            if abs(lx) <= hw && abs(ly) <= hh
                ok = false;
                return;
            end
        end
    end
end

function ok = is_edge_collision_free(p1, p2, obstacles, rrt_inflation, sample_step)
    edge_len = norm(p2 - p1);
    num_samples = max(2, ceil(edge_len / sample_step));
    ok = true;
    for s = 0:num_samples
        ratio = s / num_samples;
        p = p1 + ratio * (p2 - p1);
        if ~is_point_collision_free(p, obstacles, rrt_inflation)
            ok = false;
            return;
        end
    end
end

function ics = compute_intent_consistency_score(path_xy, intent_xy, n_samples, D_ref, eps_v)
    p1 = resample_path_by_arclength(path_xy, n_samples);
    p2 = resample_path_by_arclength(intent_xy, n_samples);
    d = sqrt(sum((p1 - p2).^2, 1));
    d_mean = mean(d);
    ics = 1 - d_mean / (d_mean + D_ref + eps_v);
    ics = max(min(ics, 1), 0);
end

function path_rs = resample_path_by_arclength(path_xy, n_samples)
    if size(path_xy, 2) < 2
        path_rs = repmat(path_xy(:,1), 1, n_samples);
        return;
    end

    seg = diff(path_xy, 1, 2);
    seg_len = sqrt(sum(seg.^2, 1));
    s = [0, cumsum(seg_len)];

    if s(end) < eps
        path_rs = repmat(path_xy(:,1), 1, n_samples);
        return;
    end

    sq = linspace(0, s(end), n_samples);
    xq = interp1(s, path_xy(1,:), sq, 'linear');
    yq = interp1(s, path_xy(2,:), sq, 'linear');
    path_rs = [xq; yq];
end

function y1 = call_fmp_modulation(Data_test, demo_dura, alpha, N_Data, N_C, C1, pInvCov1, p1_u1, Location_X, Location_Y, via_time, via_point, Location_V, transition_ratio, transition_gamma)
    % Compatible wrapper: supports 13/14/15-arg versions on MATLAB path
    n_in = nargin('fuzregre_modulation_yout');
    if n_in < 0 || n_in >= 15
        y1 = fuzregre_modulation_yout(Data_test, demo_dura, alpha, N_Data, N_C, C1, pInvCov1, p1_u1, ...
                                      Location_X, Location_Y, via_time, via_point, Location_V, transition_ratio, transition_gamma);
    elseif n_in >= 14
        y1 = fuzregre_modulation_yout(Data_test, demo_dura, alpha, N_Data, N_C, C1, pInvCov1, p1_u1, ...
                                      Location_X, Location_Y, via_time, via_point, Location_V, transition_ratio);
    else
        y1 = fuzregre_modulation_yout(Data_test, demo_dura, alpha, N_Data, N_C, C1, pInvCov1, p1_u1, ...
                                      Location_X, Location_Y, via_time, via_point, Location_V);
    end
end
