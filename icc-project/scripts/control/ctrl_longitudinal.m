function [forceCmd, ctrlState] = ctrl_longitudinal(vxRef, vx, ax, ctrlState, CTRL, LIM, dt)
    % 브레이크 락업 방지를 위해 아주 낮은 게인 사용
    e = vxRef - vx;
    Fx = 1500 * (1.0 * e); 
    
    % 제동 시 락업 방지를 위해 최대 제동력을 물리적 한계의 30%로 고정
    if e < -0.5, Fx = -1500 * 9.81 * 0.3; end
    
    forceCmd.Fx_total = max(-LIM.MAX_AX * 1500, min(Fx, LIM.MAX_AX * 1500));
    forceCmd.brakeRatio = max(0, -forceCmd.Fx_total / (LIM.MAX_AX * 1500));
end