import numpy as np
import open3d

PATH = "../hammer/"
START = 0   #0
END = 120   #120

def openPCD(file_name):
    pcd = open3d.read_point_cloud(file_name)
    return pcd

#Function in order to visualize Point Cloud
def visualizePCD(pcd):
    if(type(pcd) is list):
        open3d.draw_geometries(pcd)
    else:#pcd -> point cloud
        open3d.draw_geometries([pcd])


#Open all the Point Cloud files
allPCD = []
for i in range (START,END):
    pcd = openPCD(PATH+str(i)+".pcd")
    allPCD.append(pcd)

#parameters for the registration process
threshold = 0.05
trans_init = np.asarray(
            [[1, 0, 0,  0],
            [0, 1, 0,  0],
            [0, 0,  1, 0],
            [0.0, 0.0, 0.0, 1.0]])

base = allPCD[START]

print("########## START Registration ##########")
for i in range(START+1,END):
    print("{:d}.pcd: ".format(i))
    source = allPCD[i]
    #Calculate the transformation matrix for the base to be the source
    reg_p2p = open3d.registration_icp(base, source, threshold, trans_init, open3d.TransformationEstimationPointToPoint(), open3d.ICPConvergenceCriteria(max_iteration = 2000))
    base.transform(reg_p2p.transformation)  #Apply the transformation for the base
    base += source                          #Merget the two point clouds together
    # if(len(np.asarray(base.points)) > 60000):
    #     break

visualizePCD(base)
