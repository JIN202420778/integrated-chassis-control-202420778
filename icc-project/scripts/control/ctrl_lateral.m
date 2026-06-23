function [deltaAdd, ctrlState] = ctrl_lateral(yawRateRef, yawRate, slipAngle, vx, ctrlState, CTRL, LIM, dt)
    e_r = yawRateRef - yawRate;
    % 피드백 게인 강화
    steer_req = 1.0 * e_r; 
    
    % 조향 한계를 5도로 설정하여 경로 추종 정밀도 향상
    max_afs = deg2rad(5.0);
    deltaAdd.steerAngle = max(-max_afs, min(steer_req, max_afs));
    
    % A4 원선회 방해를 막기 위해 ESC 비활성화
    deltaAdd.yawMoment = 0;
end