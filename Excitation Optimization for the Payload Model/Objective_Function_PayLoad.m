function fitness_value = Objective_Function_PayLoad( XI,DOF,Calculate_Num,Calculate_Interval,Calculate_Init,Sampling_Time,wf,Num_Coefficient,Q_NonMotion,DOF_Start,DOF_Active )

k = 1:Calculate_Num;
t_sample = (Calculate_Interval.*(k-1)+Calculate_Init)*Sampling_Time;

for Joint = 1:(DOF_Start-1)
    Qn(Joint,1:Calculate_Num) = Q_NonMotion(Joint);
    dQn(Joint,1:Calculate_Num) = 0;
    ddQn(Joint,1:Calculate_Num) = 0;
end

for Joint = DOF_Start:DOF
    Extra_Coe(Joint-DOF_Start+1,1:Num_Coefficient) = XI(1,(Num_Coefficient*(Joint-DOF_Start)+1):Num_Coefficient*(Joint-DOF_Start+1));  
end

[ Qnx ,dQnx ,ddQnx ] = Exciting_Trajectory( Extra_Coe,t_sample,wf );

Qn(DOF_Start:DOF,1:Calculate_Num) = Qnx;
dQn(DOF_Start:DOF,1:Calculate_Num) = dQnx;
ddQn(DOF_Start:DOF,1:Calculate_Num) = ddQnx;

for k = 1:Calculate_Num
    Heq(DOF_Active*(k-1)+1:DOF_Active*k,:) = CO605_PayLoadRegressor(Qn(:,k),dQn(:,k),ddQn(:,k));
end

fitness_value = cond(Heq);

end
