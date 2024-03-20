import numpy as np

locs = np.array([
    [1,2,3],        #toptip
    [4,5,6],        #bottomtip
    [7,8,9],        #left top
    [10,11,12],        #left bottom
    [13,14,15],        #right top 
    [16,17,18],        #right bottom
])

normref_toptip = np.array([0,1,0])
normref_bottomtip = np.array([0,1,0])
normref_left = np.array([0,1,0])
normref_right = np.array([0,1,0])
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
    [1,2,3],        #toptip
    [4,5,6],        #bottomtip
    [7,8,9],        #left top
    [10,11,12],        #left bottom
    [13,14,15],        #right top 
    [16,17,18],        #right bottom
])
thumb_normref_toptip = np.array([0,1,0])
thumb_normref_bottomtip = np.array([0,1,0])
thumb_normref_left = np.array([0,1,0])
thumb_normref_right = np.array([0,1,0])
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
