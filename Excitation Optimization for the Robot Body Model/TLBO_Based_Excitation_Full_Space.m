clear all;
clc;

Q1_set = [-160 160;-160 160;-160 160];
Q2_set = [-160 -30;-160 -30;-160 -30];
Q3_set = [-20 220;-20 220;-20 220];
Q4_set = [-170 170;-170 170;-170 170];
Q5_set = [-115 115; -115 115; -115 115];
Q6_set = [-170 170;-170 170;-170 170];

Exciting_Time = 6;
Sampling_Time = 0.004;
Max_Init_SampTime = 0.008;
Max_Interval_SampTime = 0.008;
Calculate_Init = ceil(Max_Init_SampTime / Sampling_Time);
Calculate_Interval = floor(Max_Interval_SampTime / Sampling_Time);

% 总的采样点数
Calculate_Num = floor(((Exciting_Time / Sampling_Time + 1) - Calculate_Init)/Calculate_Interval + 1);

DOF = 6;

Tf = Exciting_Time;

% 傅里叶级数基频
wf = 2*pi/Tf;       

% 种群数量-TLBO成员数量
Population = 15;  

% 最大迭代次数
Iteration = 20;   

% 每个自由度的设计变量数量 [a1 b1 a2 b2 a3 b3 a4 b4 q_k0 dq_k0 ddq_k0]
Num_Design_Variate_OneDof = 11;  

% 总的设计变量数量 （对于6轴机械臂 = 6 * 11）
Num_Design_Variate = DOF*Num_Design_Variate_OneDof;     

TYPE = 1;
UIO = [randperm(TYPE); randperm(TYPE); randperm(TYPE); randperm(TYPE); randperm(TYPE); randperm(TYPE)];

profile clear;
profile off;
profile on;

for MU = 1:TYPE
    
    % 关节限位
    q_max = [Q1_set(UIO(1,MU),2) Q2_set(UIO(2,MU),2) Q3_set(UIO(3,MU),2) Q4_set(UIO(4,MU),2) Q5_set(UIO(5,MU),2) Q6_set(UIO(6,MU),2)]/180*pi;
    q_min = [Q1_set(UIO(1,MU),1) Q2_set(UIO(2,MU),1) Q3_set(UIO(3,MU),1) Q4_set(UIO(4,MU),1) Q5_set(UIO(5,MU),1) Q6_set(UIO(6,MU),1)]/180*pi;
    
    % 速度和加速度限制
    dq_max = [110 110 230 230 230 230]/180*pi;
    ddq_max = [380; 380; 780; 780; 780; 780]/180*pi;
    
    % 随机生成当前种群的设计变量系数，范围在[-3, 3]之间  为什么要限制成这个范围？
    Coefficient_ExTra_Current = -3 + 6*rand(Population,Num_Design_Variate);
    
    % 对傅里叶级数系数进行缩放+中心平移处理
    for i = 1:Population
        for Joint = 1:DOF
            XI = Coefficient_ExTra_Current(i,(Num_Design_Variate_OneDof*(Joint-1)+1):Num_Design_Variate_OneDof*Joint);
            [XI] = Tradeoff_Modify( XI,wf,q_max(Joint),q_min(Joint),dq_max(Joint),ddq_max(Joint));
            Coefficient_ExTra_Current(i,(Num_Design_Variate_OneDof*(Joint-1)+1):Num_Design_Variate_OneDof*Joint) = XI;
        end
    end
    

    for i = 1:Population
        Value(i) = Objective_BattleMyself_FullModel( Coefficient_ExTra_Current(i,:),wf,Calculate_Num,Calculate_Interval,Calculate_Init,Sampling_Time,DOF,Num_Design_Variate_OneDof );
    end
    
    % 从所有种群中找出最小的条件数以及对应的索引
    [Y_total_kbese_i(1), i_best] = min(Value);
    
    for k = 1:Iteration

        %knowledge transfer
        for j = 1:Num_Design_Variate

            % 计算所有成员的平均学习方法
            Mean_Result(1,j) = sum(Coefficient_ExTra_Current(:,j))/Population;
        end
        
        % 随机选择的标量，取值1或2
        TF = round(1 + rand);

        % r_is 是迭代步骤中每个对象相关的向量，其元素是在[0,1]上均匀分布的随机数
        r_is = rand(1,Num_Design_Variate);

        % 将使得目标函数值 f_d 最小的成员选做教师，其学习方法记 X_tch 
        X_tch = Coefficient_ExTra_Current(i_best,1:Num_Design_Variate);

        % 教师对学生的影响
        Difference_Mean_i = r_is.*(X_tch - TF * Mean_Result(1,1:Num_Design_Variate));

        for i = 1:Population

            % 根据教师影响Difference_Mean_i，每个学生更新其潜在学习方法
            Coefficient_ExTra_Current_New(i,1:Num_Design_Variate) = Coefficient_ExTra_Current(i,1:Num_Design_Variate) + Difference_Mean_i;
            
            % 对傅里叶级数系数进行缩放+中心平移处理
            for Joint = 1:DOF
                XI = Coefficient_ExTra_Current_New(i,(Num_Design_Variate_OneDof*(Joint-1)+1):Num_Design_Variate_OneDof*Joint);
                [XI] = Tradeoff_Modify( XI,wf,q_max(Joint),q_min(Joint),dq_max(Joint),ddq_max(Joint));
                Coefficient_ExTra_Current_New(i,(Num_Design_Variate_OneDof*(Joint-1)+1):Num_Design_Variate_OneDof*Joint) = XI;
            end
        end
        
        %Optimization Selected
        for i = 1:Population

            % 修正后的Xd_new计算目标函数值fd_new
            Value_new(i) = Objective_BattleMyself_FullModel( Coefficient_ExTra_Current_New(i,:),wf,Calculate_Num,Calculate_Interval,Calculate_Init,Sampling_Time,DOF,Num_Design_Variate_OneDof );
            
            % 如果fd_new小于fd，成员d接受Xd_new作为新的学习方法
            if Value_new(i) < Value(i)
                Coefficient_ExTra_Current(i,1:Num_Design_Variate) = Coefficient_ExTra_Current_New(i,1:Num_Design_Variate);
                Value(i) = Value_new(i);
            end
        end
        
        
        %% 学习者之间讨论交流来增加各自的知识
        
        for i = 1:Population
            Rand_Subject(1,1:Num_Design_Variate) = rand(1,Num_Design_Variate);

            % 成员i随机选择另外一个非本身的成员进行讨论
            if i == 1
                Order = 2:Population;
            elseif i == Population
                Order = 1:(Population-1);
            else
                Order1 = 1:(i-1);
                Order2 = (i+1):Population;
                Order = [Order1 Order2];
            end
            h = Order(ceil(rand*(Population-1)));   %Learner
            
            % 比较学员i和学员h的目标函数值，形成一个潜在学习方法
            if Value(i) <= Value(h)
                Coefficient_ExTra_Current_New(i,1:Num_Design_Variate) = Coefficient_ExTra_Current(i,1:Num_Design_Variate) + Rand_Subject.*(Coefficient_ExTra_Current(i,1:Num_Design_Variate) - Coefficient_ExTra_Current(h,1:Num_Design_Variate));
            else
                Coefficient_ExTra_Current_New(i,1:Num_Design_Variate) = Coefficient_ExTra_Current(i,1:Num_Design_Variate) + Rand_Subject.*(Coefficient_ExTra_Current(h,1:Num_Design_Variate) - Coefficient_ExTra_Current(i,1:Num_Design_Variate));
            end
            
             % 对傅里叶级数系数进行缩放+中心平移处理
            for Joint = 1:DOF
                XI = Coefficient_ExTra_Current_New(i,(Num_Design_Variate_OneDof*(Joint-1)+1):Num_Design_Variate_OneDof*Joint);
                [XI] = Tradeoff_Modify( XI,wf,q_max(Joint),q_min(Joint),dq_max(Joint),ddq_max(Joint));
                Coefficient_ExTra_Current_New(i,(Num_Design_Variate_OneDof*(Joint-1)+1):Num_Design_Variate_OneDof*Joint) = XI;
            end
            
            % 再次计算新的目标函数值
            Value_new(i) = Objective_BattleMyself_FullModel( Coefficient_ExTra_Current_New(i,:),wf,Calculate_Num,Calculate_Interval,Calculate_Init,Sampling_Time,DOF,Num_Design_Variate_OneDof );
            
            % 如果fd_new < fd，表明成员i通过新的学习方法可以取得更高的分数，因此接受新的学习方法
            if Value_new(i) < Value(i)
                Coefficient_ExTra_Current(i,1:Num_Design_Variate) = Coefficient_ExTra_Current_New(i,1:Num_Design_Variate);
                Value(i) = Value_new(i);
            end      
        end
        
        % 总共迭代 Iteration+1 次，将每次的最小目标值都存储在Y_total_kbese_i
        [Y_total_kbese_i(k+1), i_best] = min(Value);
        
    end
    
    Best_Sollution_Cycle(MU,1:Num_Design_Variate) = Coefficient_ExTra_Current(i_best,1:Num_Design_Variate);
    
    Y_total_kbese_i(end)
    
end

profile viewer;
profile off;

figure(MU);
X = 0:1:Iteration;
plot(X,Y_total_kbese_i,'d:b','linewidth',1.5);
xlabel('Iteration');
ylabel('Condition number');
title('My Method');

for Joint = 1:DOF
    Coefficient_ExTra(Joint,1:Num_Design_Variate_OneDof) = Best_Sollution_Cycle(1,(Num_Design_Variate_OneDof*(Joint-1)+1):Num_Design_Variate_OneDof*Joint);  
end

