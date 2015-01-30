function out = imu_angle2_to_angle2(imu_angle,angle0)

    run torso_measures_NX
    
    % offset from torso back plate to torso back joint orientation
    offset_angle = an(TB_top-TB_bottom,[0;0;0],L-J);
    
    % angle 1 dependant of angle0
    angle1 = angle0_to_angle1(angle0);
    
    % angle0 - angle1 + angle2 = imu_angle - offset_angle 
    % so angle2 is:
    out = imu_angle-offset_angle-angle0+angle1;
    
end