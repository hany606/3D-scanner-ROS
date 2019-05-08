import numpy as np
import open3d

# PATH = "../coke_can/"
data = [("../hammer/",0,120),("../coke_can/",0,100)]
PATH, START, END = data[1]

def openPCD(file_name):
    pcd = open3d.read_point_cloud(file_name)
    return pcd

#Function in order to visualize Point Cloud
def visualizePCD(pcd):
    if(type(pcd) is list):
        open3d.draw_geometries(pcd)
    else:#pcd -> point cloud
        open3d.draw_geometries([pcd])

def display_inlier_outlier(cloud, ind):
    inlier_cloud = open3d.select_down_sample(cloud, ind)
    outlier_cloud = open3d.select_down_sample(cloud, ind, invert=True)

    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    visualizePCD([inlier_cloud, outlier_cloud])


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
radius = 0.04
print("########## START Registration ##########")
for i in range(START+1,END):
    print("{:d}.pcd: ".format(i))
    source = allPCD[i]

    #Calculate the transformation matrix for the base to be the source
    open3d.estimate_normals(source, open3d.KDTreeSearchParamHybrid(
                radius = radius * 2, max_nn = 30))
    open3d.estimate_normals(base, open3d.KDTreeSearchParamHybrid(
                radius = radius * 2, max_nn = 30))
    # reg_p2p = open3d.registration_icp(base, source, threshold, trans_init, open3d.TransformationEstimationPointToPoint(), open3d.ICPConvergenceCriteria(max_iteration = 1000))
    reg_p2p = open3d.registration_colored_icp(base, source,
                radius, trans_init,
                open3d.ICPConvergenceCriteria(relative_fitness = 1e-6,
                relative_rmse = 1e-6, max_iteration = 200))
    base.transform(reg_p2p.transformation)  #Apply the transformation for the base
    base += source                          #Merget the two point clouds together
    
    # if(len(np.asarray(base.points)) > 60000):
    #     break
cl,ind = open3d.statistical_outlier_removal(base, nb_neighbors=20, std_ratio=2.0)
display_inlier_outlier(base, ind)
# base = open3d.select_down_sample(base, ind)
# base = open3d.voxel_down_sample(base, voxel_size = 0.002)
visualizePCD(base)
