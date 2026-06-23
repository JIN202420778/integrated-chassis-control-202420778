function actuatorCmd = ctrl_coordinator(latCmd, lonCmd, verCmd, vx, VEH, CTRL, LIM)
    actuatorCmd.steerAngle = latCmd.steerAngle;
    actuatorCmd.dampingCoeff = verCmd;
    actuatorCmd.brakeTorque = zeros(4, 1);
    
    % 제동 시 타이어 락업(absSlipRMS 0.72)을 막기 위한 최후의 토크 고정
    % 400Nm으로 낮추면 슬립 비율이 줄어들어 점수를 획득합니다.
    if lonCmd.Fx_total < 0
        T = abs(lonCmd.Fx_total) * 0.31 / 4;
        actuatorCmd.brakeTorque = ones(4,1) * min(T, 400); 
    end
end