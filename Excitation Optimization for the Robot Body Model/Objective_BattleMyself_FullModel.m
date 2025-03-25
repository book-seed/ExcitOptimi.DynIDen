function Cond_H = Objective_BattleMyself_FullModel( Coefficient,wf,Max,Interval,Ini,Sampling_Time,DOF,Num_Design_Variate_OneDof )

% Max      ：总采样点数
% Interval ：采样间隔内点数
% Ini      : 初始采样点数量

k = 1:Max;
t_sample = (Interval.*(k-1)+Ini)*Sampling_Time;

% Extra_Coe ：[DOF * 11]
for Joint = 1:DOF
    Extra_Coe(Joint,1:Num_Design_Variate_OneDof) = Coefficient(1,(Num_Design_Variate_OneDof*(Joint-1)+1):Num_Design_Variate_OneDof*Joint);  
end

% Qn : [DOF * MAX]
[ Qn ,dQn ,ddQn ] = Exciting_Trajectory( Extra_Coe,t_sample,wf );

for k = 1:Max
    H_Cc_Base(DOF*(k-1)+1:DOF*k,:) = Base_Dynamics_Para_CO605_Cc_Neg(Qn(:,k),dQn(:,k),ddQn(:,k));
end

Cond_H = cond(H_Cc_Base); 


end




