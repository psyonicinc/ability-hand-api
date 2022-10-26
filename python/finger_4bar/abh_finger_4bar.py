import numpy as np
#
#   Ascii Art Depction of the 4bar linkage mechanism:
#
#                              L3
#                              |
#                       OOOOOOOO
#                       \    q2/
#                        \    /
#                         \  /
#                          \/
#                          /\
#                   L1<---/  \
#                        /    \
#                       /      \--->L2
#                      /q1      \
#                     ############
#                            |
#                           L0



"""
	Main function used to determine the relationship of the Ability Hand 4bar linkage driven angle q2.
	This is an exact calculation of the mechanism. The output q2 angle is a function of the q1 angle
	in the URDF model of the Ability Hand, and is valid for the geometry defined in those files.
	
	INPUT: q1 (URDF q1 of any of the finger digits)
	OUTPUT: q2 (URDF q2 for that finger digit)
"""
def get_abh_4bar_driven_angle(q1):
	q1 = q1 + 0.084474	#factor in offset imposed by our choice of link frame attachments

	#L0 = 9.5
	L1 = 38.6104
	L2 = 36.875
	L3 = 9.1241
	p3 = np.array([9.47966, -0.62133, 0])	#if X of the base frame was coincident with L3, p3 = [9.5, 0 0]. However, our frame choices are different to make the 0 references for the fingers nice, so this location is a little less convenient.

	cq1 = np.cos(q1)
	sq1 = np.sin(q1)
	p1 = np.array([L1*cq1, L1*sq1, 0])
	
	sol0, sol1 = get_intersection_circles(p3,L2,p1,L3)
	
	#copy_vect3(&p2, &sols[1])
	p2 = sol1
	
	# calculate the linkage intermediate angle!
	q2pq1 = np.arctan2(p2[1]-L1*sq1, p2[0]-L1*cq1)
	q2 = q2pq1-q1
	q2 = np.mod(q2+np.pi, 2*np.pi)-np.pi
	return q2

"""
	Helper function for the above function. 
	Solves for the intersection of two circles.
	
	Behavior of this function for circles that intersect at only one point, or 
	circles that do not intersect, is not defined.
	
	INPUTS: 
		o0: origin of circle 0
		r0: radius of circle 0
		o1: origin of circle 1
		r1: origin of circle 1
	OUTPUS:
		sol0: 2d position of the first intersection point
		sol1: 2d position of the second intersection point
"""
def get_intersection_circles(o0, r0, o1, r1):
	
	d = np.sqrt(np.sum((o0 - o1)**2))
	
	
	sol0 = np.zeros(2)
	sol1 = np.zeros(2)
	
	r0_sq = r0*r0
	r1_sq = r1*r1
	d_sq = d*d
	
	# solve for a
	a = (r0_sq - r1_sq + d_sq)/(2*d)

	# solve for h
	h_sq = r0_sq - a*a
	h = np.sqrt(h_sq)

	# find p2
	p2 = o0 + a*(o1-o0)/d
	
	t1 = h*(o1[1]-o0[1])/d
	t2 = h*(o1[0]-o0[0])/d

	sol0[0] = p2[0] + t1
	sol0[1] = p2[1] - t2
	
	sol1[0] = p2[0] - t1
	sol1[1] = p2[1] + t2
	
	return sol0, sol1