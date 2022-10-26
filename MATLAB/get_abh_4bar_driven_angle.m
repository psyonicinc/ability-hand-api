function out = get_abh_4bar_driven_angle(in)
	%
	%	Ascii Art Depction of the 4bar linkage mechanism:
	%
	%                              L3
	%                              |
	%                       OOOOOOOO
	%                       \    q2/
	%                        \    /
	%                         \  /
	%                          \/
	%                          /\
	%                   L1<---/  \
	%                        /    \
	%                       /      \--->L2
	%                      /q1      \
	%                     ############
	%                            |
	%                           L0
	
	in = in + 0.084474;	%factor in offset imposed by our choice of link frame attachments

    %L0 = 9.5;
    L1 = 38.6104;
    L2 = 36.875;
    L3 = 9.1241;
    p3 = [9.47966, -0.62133, 0];	%if X of the base frame was coincident with L3, p3 = [9.5, 0 0]. However, our frame choices are different to make the 0 references for the fingers nice, so this location is a little less convenient.

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