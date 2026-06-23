function [dampingCmd, ctrlState] = ctrl_vertical(suspState, ctrlState, CTRL, dt)
    dampingCmd = zeros(4, 1);
    for i = 1:4
        zs_d = suspState.zs_dot(i);
        zu_d = suspState.zu_dot(i);
        v_rel = zs_d - zu_d;
        
        if (zs_d * v_rel) > 0
            c_req = CTRL.VER.skyGain * abs(zs_d) / max(abs(v_rel), 0.01);
            dampingCmd(i) = max(CTRL.VER.cMin, min(c_req, CTRL.VER.cMax));
        else
            dampingCmd(i) = CTRL.VER.cMin;
        end
    end
end