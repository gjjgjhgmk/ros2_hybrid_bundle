function [y1] = fuzregre_modulation_yout(Data_test,demo_dura,alpha,N_Data,N_C, C1, pInvCov1, p1_u1, Location_X, Location_Y, via_time, via_point,Location_V, transition_ratio, transition_gamma)

if nargin < 14 || isempty(transition_ratio)
    transition_ratio = 0.10;
end
transition_ratio = max(0, min(0.5, transition_ratio));
if nargin < 15 || isempty(transition_gamma)
    transition_gamma = 1.0;
end
transition_gamma = max(1.0, transition_gamma);

D_y = size(Location_Y, 2);
D_C = size(C1, 1);
N_via = numel(via_time);
% Data_test = [demo_dura*Data_test;(demo_dura*Data_test).^2];
Data_test = [demo_dura*Data_test;exp(alpha*Data_test/(Data_test(end)))];%修改
% Initialize variables
MD2 = zeros(N_C, N_via);
C = C1;
p1_u = p1_u1;
N_C1 = N_C;

% Compute MD2
for i = 1:N_C
    % dist = [via_time; via_time.^2; via_point(Location_V, :)] - C1(:, i);
    dist = [via_time; exp(alpha*via_time/(Data_test(1,end))); via_point(Location_V, :)] - C1(:, i);%修改
    pInvCov_i = pInvCov1(:, :, i);
    MD2(i, :) = sum(dist .* (pInvCov_i * dist), 1);
end

temp = 1./MD2;
U_via = temp ./ sum(temp, 1);
[~, jl] = max(U_via);

%% classify switch and add
via_time1=via_time(1);
via_time2=[];
via_point1=via_point(:,1);
via_point2=[];
N_via1 = 1;
N_via2 = 0;
U_via2 = [];
jl1 = jl(1);
jl2 = [];

for i=2:N_via
   if any(jl(i) == jl(1:i-1))
       via_time2 = [via_time2 via_time(i)];
       via_point2 = [via_point2 via_point(:,i)];
       N_via2 = N_via2 + 1;
       U_via2 = [U_via2 U_via(:,i)];
       jl2 = [jl2 jl(i)];
   else
       via_time1 = [via_time1 via_time(i)];
       via_point1 = [via_point1 via_point(:,i)];
       N_via1 = N_via1 + 1; 
       jl1 = [jl1 jl(i)];
   end
end

%% switch
for i = 1:N_via1
   [~, idx] = min(abs(Data_test(1,:) - via_time1(i)));
   C_index = jl1(i);
   C(1:end-1,C_index) = Data_test(:,idx)+1e-7;
   datax1 = Data_test(Location_X(1:end),idx);
   p1_u_delete = p1_u(:,:,C_index);
   p1_u(1,:,C_index) = (via_point1(Location_V,i)-([datax1']*p1_u_delete(2:end,:))');

end

%% add
d_pI1 = size(pInvCov1,1);
d_pI2 = size(pInvCov1,2);

d_pu1 = size(p1_u,1);
d_pu2 = size(p1_u,2);


pInvCov2=zeros(d_pI1,d_pI2,25);
pInvCov2(:,:,1:N_C1)=pInvCov1;
p1_u2=zeros(d_pu1,d_pu2,25);
p1_u2(:,:,1:N_C1)=p1_u;
C2=C;

if N_via2 > 0
    C2 = [C2, zeros(D_C, N_via2)];
    pInvCov2 = cat(3, pInvCov2, zeros(d_pI1, d_pI2, N_via2));
    p1_u2 = cat(3, p1_u2, zeros(d_pu1, d_pu2, N_via2));
    
    for i = 1:N_via2
        [~, idx] = min(abs(Data_test(1, :) - via_time2(i)));

        C2(:, end - N_via2 + i) = C1(:,jl2(i));
        C2(1:end-1, end - N_via2 + i) = Data_test(:, idx) + 1e-7;

        U_via2_current = reshape(U_via2(:, i), [1, 1, N_C]);
        pInvCov_temp = bsxfun(@times, pInvCov1, U_via2_current);
        pInvCov2(:, :, N_C1 + i) = sum(pInvCov_temp, 3);

        p1_u2(:, :, N_C1 + i) = p1_u1(:, :, jl2(i));

        p1_u_delete = p1_u2(:, :, N_C1 + i);
        p1_u_delete(1, :) = [];
        p1_u2(1, :, N_C1 + i) = (via_point2(Location_V, i) - (Data_test(Location_X, idx)' * p1_u_delete)')';
    end
    N_C1 = N_C1 + N_via2;
end


MD2 = zeros(N_C1,N_Data);
y1=zeros(D_y,N_Data);
for i = 1:N_C1
    dist = Data_test([Location_X],:) - C2([Location_X],i);
    pInvCov_i = pInvCov2([Location_X],[Location_X],i);
    MD2(i,:) = sum(dist.*(pInvCov_i*dist), 1);
end

temp = 1./MD2;

% Soft Boundary Blending for added avoidance clusters
if N_C1 > N_C && ~isempty(via_time)
    t = Data_test(1, :);
    core_start = min(via_time);
    core_end = max(via_time);
    total_time = max(eps, t(end) - t(1));
    transition_len = transition_ratio * total_time; % transition zone length
    eps_mask = 1e-6;

    decay_mask = eps_mask * ones(1, N_Data);

    core_idx = (t >= core_start) & (t <= core_end);
    decay_mask(core_idx) = 1;

    if transition_len > 0
        % Left transition: eps -> 1
        left_start = core_start - transition_len;
        left_idx = (t >= left_start) & (t < core_start);
        if any(left_idx)
            s = (t(left_idx) - left_start) / transition_len; % [0,1]
            blend = 0.5 - 0.5 * cos(pi * s);
            blend = blend .^ transition_gamma;
            decay_mask(left_idx) = eps_mask + (1 - eps_mask) * blend;
        end

        % Right transition: 1 -> eps
        right_end = core_end + transition_len;
        right_idx = (t > core_end) & (t <= right_end);
        if any(right_idx)
            s = (t(right_idx) - core_end) / transition_len; % [0,1]
            blend = 0.5 + 0.5 * cos(pi * s);
            blend = blend .^ transition_gamma;
            decay_mask(right_idx) = eps_mask + (1 - eps_mask) * blend;
        end
    end

    temp(N_C+1:N_C1, :) = temp(N_C+1:N_C1, :) .* decay_mask;
end

U = temp ./ sum(temp, 1);

for j=1:N_C1
  y1=y1+([ones(N_Data,1) Data_test(Location_X,:)']*p1_u2(:,:,j))'.*U(j,:);
end 

end
