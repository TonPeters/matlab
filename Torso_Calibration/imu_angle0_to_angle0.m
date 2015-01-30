function out = imu_angle0_to_angle0(angle)

    run torso_measures_NX
    
    offset_angle = an(RL_top-RL_bottom,[0;0;0],G-A);
    
    out = angle-offset_angle;
    
end