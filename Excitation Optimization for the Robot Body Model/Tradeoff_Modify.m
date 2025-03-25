function [Coefficient_ExTra] = Tradeoff_Modify( Coefficient_ExTra,wf,q_max,q_min,dq_max,ddq_max )

Tf = 2*pi/wf;

t = 0:0.01:Tf;


% 速度和加速度轨迹上特别设计的偏移量
ddq_ini = Coefficient_ExTra(2)*(wf) + Coefficient_ExTra(4)*(2*(wf)) + Coefficient_ExTra(6)*(3*(wf)) + Coefficient_ExTra(8)*(4*(wf)); 
dq_ini = Coefficient_ExTra(1) + Coefficient_ExTra(3) + Coefficient_ExTra(5) + Coefficient_ExTra(7);

% Coefficient_ExTra [a1 b1 a2 b2 a3 b3 a4 b4 q_k0 dq_k0 ddq_k0]
Coefficient_ExTra(1,10:11) = [dq_ini, ddq_ini];

% 加入偏移轨迹后的傅里叶级数激励轨迹
[q,dq,ddq] = Exciting_Trajectory(Coefficient_ExTra,t,wf);

% 激励轨迹的特征量定义
Qmax = max(q);
Qmin = min(q);
dQmax = max(abs(dq));
ddQmax = max(abs(ddq));

%% ------------ 缩放------------------

% 速度缩放因子
Gama_2 = dq_max/dQmax;

% 加速度缩放因子
Gama_3 = ddq_max/ddQmax;
 
% 位置缩放因子 
Dq_limit = q_max - q_min;
Dq_current = Qmax - Qmin;
Gama_1 = Dq_limit/Dq_current;

% 综合缩放因子
Gama = min([Gama_1,Gama_2,Gama_3]);

% 对当前提供的傅里叶系数执行缩放操作
Coefficient_ExTra = Coefficient_ExTra * Gama;

%% ------------ 中心平移 -----------------------

% 关节位置可行区间的中心点
q_middle= (q_max + q_min)/2; 

% 位置轨迹的中心点
q_center = (Qmax + Qmin)/2;

% 缩放后的位置轨迹偏移量
q_ini = Coefficient_ExTra(9);

% 缩放后的位置轨迹的中心点
q_center = q_center*Gama;

% 对位置轨迹上的偏移量进行平移操作
q_ini = q_ini - (q_center - q_middle);

% 更新中心平移后的位置轨迹上的偏移量
Coefficient_ExTra(1,9) = q_ini;


end

