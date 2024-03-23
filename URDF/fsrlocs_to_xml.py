import numpy as np


sizetype = "small"
thumbtype = "left"

if(sizetype == "small"):
    locs = np.array([
        [30.95037374e-3, -15.54446733e-3, 0.00000000],        #toptip
        [25.10622815e-3, -11.57958764e-3, 0.00000000],        #bottomtip
        [27.02797318e-3, -14.95400505e-3, 4.00000000e-3],        #left top
        [12.84956345e-3, -9.67364605e-3, 4.00000000e-3],        #left bottom
        [27.02797318e-3, -14.95400505e-3, -4.00000000e-3],        #right top 
        [27.02797318e-3, -14.95400505e-3, -4.00000000e-3],        #right bottom
    ])
    normref_toptip = np.array([30.00616438e-3,		-13.25623041e-3, 	3.25000000e-3])
    normref_bottomtip = np.array([29.33151346e-3,	-12.66935683e-3, 	3.25000000e-3])
    normref_left = np.array([0,0,4])
    normref_right = np.array([0,0,-4])

elif(sizetype == "large"):
    locs = np.array([
        [36.78129329e-3, -19.66347638e-3, 0.00000000],        #toptip
        [30.87755457e-3, -15.31625126e-3, 0.00000000],        #bottomtip
        [31.18106868e-3, -17.75288355e-3, 4.00000000e-3],        #left top
        [16.80497397e-3, -13.08242585e-3, 4.00000000e-3],        #left bottom
        [31.18106868e-3, -17.75288355e-3, -4.00000000e-3],        #right top
        [16.80497397e-3, -13.08242585e-3, -4.00000000e-3],        #right bottom
    ])
    normref_toptip = np.array([35.41686795e-3,		-16.85118633e-3, 	3.25000000e-3])
    normref_bottomtip = np.array([34.76690904e-3,	-16.31937847e-3, 	3.25000000e-3])
    normref_left = np.array([0,0,4])
    normref_right = np.array([0,0,-4])


locreflist = np.array([
    normref_toptip,
    normref_bottomtip,
    normref_left,
    normref_left,
    normref_right,
    normref_right     
])


"""
	get xyz/rpy from a 4x4 homogeneous transformation matrix
"""
def get_rpy(R):
	yaw = np.arctan2(R[1][0],R[0][0])
	pitch = np.arctan2( -R[2][0], np.sqrt(R[2][1]**2+R[2][2]**2))
	roll = np.arctan2(R[2][1],R[2][2])
	rpy = np.array([roll,pitch,yaw])
	return rpy

"""
	<sensor name="fsr1" >
		<parent link="index_L2"/>
		<origin xyz = "0 0 0" rpy = "0 0 0"/>
	</sensor>
"""
name=["index", "middle", "ring", "pinky"]
for idx in range(0,(30-6)):
    blk = "<sensor name=\"fsr"+str(idx+1)+"\">\r\n"
    blk = blk+"    <parent link = \""+name[int(idx/6)]+"_L2\"/>\r\n"
    
    locnorm = (locreflist[idx % 6] - locs[idx % 6])
    locnorm = locnorm / np.sqrt(locnorm.dot(locnorm))
    zbasis = np.array([0,0,1])
    zn = np.cross(zbasis, locnorm)
    yn = np.cross(zn, zbasis)
    xn = np.cross(yn, zn)
    R = np.array([xn,yn,zn]).reshape((3,3))
    blk = blk+"    <origin xyz = \""+str(locs[idx % 6])+"\" rpy = \""+str(get_rpy(R))+"\"/>\r\n"
    blk = blk+"</sensor>\r\n"
    print(blk)


del locs
del normref_toptip
del normref_bottomtip
del normref_left
del normref_right




thumblocs = np.array([
    [55.422e-3, 23.513e-3, 5.055e-3],        #toptip
    [47.909e-3,	23.120e-3,	5.728e-3],        #bottomtip
    [48.227e-3, 22.352e-3, -0.805e-3],        #left top
    [36.868e-3, 17.111e-3, -1.369e-3],        #left bottom
    [49.435e-3, 19.437e-3, 10.970e-3],        #right top 
    [38.158e-3, 13.997e-3, 11.211e-3],        #right bottom
])
if(thumbtype == 'left):
	for v in thumblocs:
		v[2] = -v[2]
thumb_normref_toptip = np.array([0,0,1])
thumb_normref_bottomtip = np.array([0,0,1])
thumb_normref_left = np.array([36.868e-3, 17.111e-3, -1.369e-3])
thumb_normref_right = np.array([38.158e-3, 13.997e-3, 11.211e-3])
thumblocreflist = np.array([
    thumb_normref_toptip,
    thumb_normref_bottomtip,
    thumb_normref_left,
    thumb_normref_left,
    thumb_normref_right,
    thumb_normref_right     
])

for idx in range(30-6, 30):
    blk = "<sensor name=\"fsr"+str(idx+1)+"\">\r\n"
    blk = blk+"    <parent link = \"thumb_L2\"/>\r\n"
    locnorm = thumblocreflist[idx % 6] - thumblocs[idx % 6]
    locnorm = locnorm / np.sqrt(locnorm.dot(locnorm))
    zbasis = np.array([0,0,1])
    zn = np.cross(zbasis, locnorm)
    yn = np.cross(zn, zbasis)
    xn = np.cross(yn, zn)
    R = np.array([xn,yn,zn]).reshape((3,3))
    blk = blk+"    <origin xyz = \""+str(thumblocs[idx % 6])+"\" rpy = \""+str(get_rpy(R))+"\"/>\r\n"
    blk = blk+"</sensor>\r\n"
    print(blk)
