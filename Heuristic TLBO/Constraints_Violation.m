function Value_Cons_Viola = Constraints_Violation(XI,wf,q_max,q_min,dq_max,ddq_max)
%CONSTRAINTS_VIOLATION 此处显示有关此函数的摘要
%   此处显示详细说明

Dof = 6;
Tf = 2*pi/wf;
t = 0:0.01:Tf;

for Joint = 1:Dof
    Coefficient_ExTra = XI(((Joint-1)*11+1):Joint*11);
    [q,dq,ddq] = Exciting_Trajectory(Coefficient_ExTra,t,wf);
    Qmax = max(q);
    Qmin = min(q);
    dQmax = max(abs(dq));
    ddQmax = max(abs(ddq));
    dq_initial = dq(1);
    ddq_initial = ddq(1);
    
    q_UM(Joint) = q_max(Joint) - Qmax;
    if q_UM(Joint) >= 0
        q_UM(Joint) = 0;
    end
    q_LM(Joint) = Qmin - q_min(Joint);
    if q_LM(Joint) >= 0
        q_LM(Joint) = 0;
    end
    
    dq_Vio(Joint) = dq_max(Joint) - dQmax;
    if dq_Vio(Joint) >= 0
        dq_Vio(Joint) = 0;
    end
    
    ddq_Vio(Joint) = ddq_max(Joint) - ddQmax;
    if ddq_Vio(Joint) >= 0
        ddq_Vio(Joint) = 0;
    end
    Q_Vio(Joint) = - q_UM(Joint) - q_LM(Joint) - dq_Vio(Joint) - ddq_Vio(Joint) + dq_initial^2 + ddq_initial^2;
end

Value_Cons_Viola = 0;

for Joint = 1:Dof
    Value_Cons_Viola = Value_Cons_Viola + Q_Vio(Joint);
end

end

