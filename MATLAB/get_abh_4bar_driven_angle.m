function out = get_abh_4bar_driven_angle(in)

    in = in - 4.84*pi/180;  %take out the offset introduced by our kinematic representation of the ability hand

    L0 = 9.5;
    L1 = 38.6104;
    L2 = 36.875;
    L3 = 9.1241;
    p3 = [L0,0,0];

    cq1 = cos(in);
    sq1 = sin(in);
    p1 = [L1*cq1, L1*sq1, 0];
    
    [sol0, sol1] = get_intersection_circles(p3,L2,p1,L3);
    
    %copy_vect3(&p2, &sols[1]);
    p2 = sol1;
    
    % calculate the linkage intermediate angle!
    q2pq1 = atan2(p2(2)-L1*sq1, p2(1)-L1*cq1);
    q2 = q2pq1-in;
    out = mod(q2+pi, 2*pi)-pi;
end