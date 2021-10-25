function [sol0, sol1] = get_intersection_circles(o0, r0, o1, r1)
    d = sqrt( sum( (o0-o1).^2) );
    
    sol0 = zeros(1,2);
    sol1 = zeros(1,2);
    
     r0_sq = r0*r0;
     r1_sq = r1*r1;
     d_sq = d*d;

    % solve for a
     a = (r0_sq - r1_sq + d_sq)/(2*d);

    % solve for h
     h_sq = r0_sq - a*a;
     h = sqrt(h_sq);

    % find p2
    p2 = o0 + a*(o1-o0)/d;
        
    t1 = h*(o1(2)-o0(2))/d;
    t2 = h*(o1(1)-o0(1))/d;

    sol0(1) = p2(1) + t1;
    sol0(2) = p2(2) - t2;
    
    sol1(1) = p2(1) - t1;
    sol1(2) = p2(2) + t2;

end