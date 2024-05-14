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
Calculate_Num = floor(((Exciting_Time / Sampling_Time + 1) - Calculate_Init)/Calculate_Interval + 1);

DOF = 6;

Tf = Exciting_Time;
wf = 2*pi/Tf;

Population = 15;  

Iteration = 100;   

Num_Design_Variate_OneDof = 11;     
Num_Design_Variate = DOF*Num_Design_Variate_OneDof;     

TYPE = 1;
UIO = [randperm(TYPE); randperm(TYPE); randperm(TYPE); randperm(TYPE); randperm(TYPE); randperm(TYPE)];

profile clear;
profile off;
profile on;

for MU = 1:TYPE
    
    q_max = [Q1_set(UIO(1,MU),2) Q2_set(UIO(2,MU),2) Q3_set(UIO(3,MU),2) Q4_set(UIO(4,MU),2) Q5_set(UIO(5,MU),2) Q6_set(UIO(6,MU),2)]/180*pi;
    q_min = [Q1_set(UIO(1,MU),1) Q2_set(UIO(2,MU),1) Q3_set(UIO(3,MU),1) Q4_set(UIO(4,MU),1) Q5_set(UIO(5,MU),1) Q6_set(UIO(6,MU),1)]/180*pi;
    
    dq_max = [110 110 230 230 230 230]/180*pi;
    ddq_max = [380; 380; 780; 780; 780; 780]/180*pi;
    
    Coefficient_ExTra_Current = -3 + 6*rand(Population,Num_Design_Variate);
        
    for i = 1:Population
        Value(i) = Objective_BattleMyself_FullModel( Coefficient_ExTra_Current(i,:),wf,Calculate_Num,Calculate_Interval,Calculate_Init,Sampling_Time,DOF,Num_Design_Variate_OneDof );
        %计算约束违法值
        Value_Cons_Viola(i) = Constraints_Violation( Coefficient_ExTra_Current(i,:),wf,q_max,q_min,dq_max,ddq_max );
        if Value_Cons_Viola(i) > 0
            Value(i) = Value(i) + 10000*Value_Cons_Viola(i);
        end
    end
    
    [Y_total_kbese_i(1), i_best] = min(Value);
    if Value_Cons_Viola(i_best) > 0
        Label_Viola(1) = 1;
    else
        Label_Viola(1) = 0;
    end
    
    for k = 1:Iteration

        %knowledge transfer
        for j = 1:Num_Design_Variate
            Mean_Result(1,j) = sum(Coefficient_ExTra_Current(:,j))/Population;
        end
        
        TF = round(1 + rand);
        Difference_Mean_i = rand(1,Num_Design_Variate).*(Coefficient_ExTra_Current(i_best,1:Num_Design_Variate) - TF*Mean_Result(1,1:Num_Design_Variate));
        
        for i = 1:Population
            Coefficient_ExTra_Current_New(i,1:Num_Design_Variate) = Coefficient_ExTra_Current(i,1:Num_Design_Variate) + Difference_Mean_i;
        end
        
        %Optimization Selected
        for i = 1:Population
            Value_new(i) = Objective_BattleMyself_FullModel( Coefficient_ExTra_Current_New(i,:),wf,Calculate_Num,Calculate_Interval,Calculate_Init,Sampling_Time,DOF,Num_Design_Variate_OneDof );
            %计算约束违法值
            Value_Cons_Viola(i) = Constraints_Violation( Coefficient_ExTra_Current_New(i,:),wf,q_max,q_min,dq_max,ddq_max );
            if Value_Cons_Viola(i) > 0
                Value_new(i) = Value_new(i) + 10000*Value_Cons_Viola(i);
            end
            
            if Value_new(i) < Value(i)
                Coefficient_ExTra_Current(i,1:Num_Design_Variate) = Coefficient_ExTra_Current_New(i,1:Num_Design_Variate);
                Value(i) = Value_new(i);
            end
        end
        
        
        %learners increase their knowledge by interacting among themselves
        
        for i = 1:Population
            Rand_Subject(1,1:Num_Design_Variate) = rand(1,Num_Design_Variate);
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
            
            %Compare i and h
            if Value(i) <= Value(h)
                Coefficient_ExTra_Current_New(i,1:Num_Design_Variate) = Coefficient_ExTra_Current(i,1:Num_Design_Variate) + Rand_Subject.*(Coefficient_ExTra_Current(i,1:Num_Design_Variate) - Coefficient_ExTra_Current(h,1:Num_Design_Variate));
            else
                Coefficient_ExTra_Current_New(i,1:Num_Design_Variate) = Coefficient_ExTra_Current(i,1:Num_Design_Variate) + Rand_Subject.*(Coefficient_ExTra_Current(h,1:Num_Design_Variate) - Coefficient_ExTra_Current(i,1:Num_Design_Variate));
            end
            
            Value_new(i) = Objective_BattleMyself_FullModel( Coefficient_ExTra_Current_New(i,:),wf,Calculate_Num,Calculate_Interval,Calculate_Init,Sampling_Time,DOF,Num_Design_Variate_OneDof );
            %计算约束违法值
            Value_Cons_Viola(i) = Constraints_Violation( Coefficient_ExTra_Current_New(i,:),wf,q_max,q_min,dq_max,ddq_max );
            if Value_Cons_Viola(i) > 0
                Value_new(i) = Value_new(i) + 10000*Value_Cons_Viola(i);
            end
            
            if Value_new(i) < Value(i)
                Coefficient_ExTra_Current(i,1:Num_Design_Variate) = Coefficient_ExTra_Current_New(i,1:Num_Design_Variate);
                Value(i) = Value_new(i);
            end
            
        end
        
        [Y_total_kbese_i(k+1), i_best] = min(Value);
        if Value_Cons_Viola(i_best) > 0
            Label_Viola(k+1) = 1;
        else
            Label_Viola(k+1) = 0;
        end
    end
    
    Best_Sollution_Cycle(MU,1:Num_Design_Variate) = Coefficient_ExTra_Current(i_best,1:Num_Design_Variate);
    
    Y_total_kbese_i(end)
    
end

profile viewer;
profile off;

figure(MU);
hold on;
for X = 0:1:Iteration
    if Label_Viola(X+1) == 1
        plot(X,Y_total_kbese_i(X+1),'ro','linewidth',1.5);
    else
        plot(X,Y_total_kbese_i(X+1),'bo','linewidth',1.5);
    end
end
set(gca,'FontName','Times New Roman','fontsize',12);
xlabel('Iterations');
ylabel('Pseudo Objective Function Value');
legend('Infeasible solution');
title('TLBO with Penalty');

for Joint = 1:DOF
    Coefficient_ExTra(Joint,1:Num_Design_Variate_OneDof) = Best_Sollution_Cycle(1,(Num_Design_Variate_OneDof*(Joint-1)+1):Num_Design_Variate_OneDof*Joint);  %最优解
end

%% 分析用
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%求基于最优解，各关节的约束违法值
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t = 0:0.01:Tf;
for Joint = 1:DOF
    [q,dq,ddq] = Exciting_Trajectory(Coefficient_ExTra(Joint,1:Num_Design_Variate_OneDof),t,wf);
    Qmax = max(q);
    Qmin = min(q);
    dQmax = max(dq);
    dQmin = min(dq);
    ddQmax = max(ddq);
    ddQmin = min(ddq);
    dQstart = dq(1);
    ddQstart = ddq(1);
    %位置
    q_UM(Joint) = q_max(Joint) - Qmax;
    q_LM(Joint) = Qmin - q_min(Joint);
    
    %速度
    dq_UM(Joint) = dq_max(Joint) - dQmax;
    dq_LM(Joint) = dQmin - (-dq_max(Joint));
    dq_initial(Joint) = dQstart;
    
    %加速度
    ddq_UM(Joint) = ddq_max(Joint) - ddQmax;
    ddq_LM(Joint) = ddQmin - (-ddq_max(Joint));
    ddq_initial(Joint) = ddQstart;
end

Time_Num = length(t);

%画各关节的激励轨迹
Fig = tiledlayout(6,3);
Fig.Padding = 'compact';
Fig.TileSpacing = 'compact';
for Joint = 1:DOF
    [q,dq,ddq] = Exciting_Trajectory(Coefficient_ExTra(Joint,1:Num_Design_Variate_OneDof),t,wf);
    %画位置
    %subplot(6,3,(Joint-1)*3+1);
    nexttile
    plot(t,q,'linewidth',1.5,'color',[0.5 0.5 0.5]);
    hold on;
    plot(t,q_max(Joint).*ones(1,Time_Num),'r--','linewidth',1.5);
    plot(t,q_min(Joint).*ones(1,Time_Num),'b--','linewidth',1.5);
    set(gca,'FontName','Times New Roman','fontsize',8);
    xlabel('Time (s)');
    ylabel('Pos. (\circ)');
    %画速度
    nexttile
    %subplot(6,3,(Joint-1)*3+2);
    plot(t,dq,'linewidth',1.5,'color',[0.5 0.5 0.5]);
    hold on;
    plot(t,dq_max(Joint).*ones(1,Time_Num),'r--','linewidth',1.5);
    plot(t,-dq_max(Joint).*ones(1,Time_Num),'b--','linewidth',1.5);
    set(gca,'FontName','Times New Roman','fontsize',8);
    xlabel('Time (s)');
    ylabel('Vel. (\circ/s)');
    %画加速度
    nexttile
    %subplot(6,3,(Joint-1)*3+3);
    plot(t,ddq,'linewidth',1.5,'color',[0.5 0.5 0.5]);
    hold on;
    plot(t,ddq_max(Joint).*ones(1,Time_Num),'r--','linewidth',1.5);
    plot(t,-ddq_max(Joint).*ones(1,Time_Num),'b--','linewidth',1.5);
    set(gca,'FontName','Times New Roman','fontsize',8);
    xlabel('Time (s)');
    ylabel('Acc. (\circ/s^2)');
end


