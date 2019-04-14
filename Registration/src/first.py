import numpy as np
import open3d

PATH = "../hammer/"
START = 0   #0
END = 120   #120

def openPCD(file_name):
    pcd = open3d.read_point_cloud(file_name)
    return pcd
def visualizePCD(pcd):
    if(type(pcd) is list):
        open3d.draw_geometries(pcd)
    else:
        open3d.draw_geometries([pcd])

if __name__ == "__main__":
    print("Hello to the main file")

    # pcd = openPCD(PATH+'14.pcd')
    # downpcd = open3d.voxel_down_sample(pcd, voxel_size = 0.05)
    # open3d.estimate_normals(downpcd, search_param = open3d.KDTreeSearchParamHybrid(radius = 0.1, max_nn = 30))
    # print(np.asarray(pcd.points))
    # open3d.draw_geometries([pcd])
    # visualizePCD(pcd)
    # # Open all the Point Cloud files
    allPCD = []
    for i in range (START,END):
        pcd = openPCD(PATH+str(i)+".pcd")
        allPCD.append(pcd)
        print("{:d}.pcd: ".format(i),end="")
        print(pcd)
    # visualizePCD(allPCD)
    # pcd.paint_uniform_color([0.5, 0.5, 0.5])
    # pcd_tree = open3d.KDTreeFlann(pcd)
    # pcd.colors[1000] = [1, 0, 0]
    
    # [k, idx, _] = pcd_tree.search_knn_vector_3d(pcd.points[1000], 10)
    # np.asarray(pcd.colors)[idx[1:], :] = [0, 0, 1]
    
    # [k, idx, _] = pcd_tree.search_radius_vector_3d(pcd.points[1000], 0.2)
    # np.asarray(pcd.colors)[idx[1:], :] = [0, 1, 0]
    
    # visualizePCD(pcd)


    threshold = 0.02
    trans_init = np.asarray(
                [[0.862, 0.011, -0.507,  0.5],
                [-0.139, 0.967, -0.215,  0.7],
                [0.487, 0.255,  0.835, -1.4],
                [0.0, 0.0, 0.0, 1.0]])
    source = allPCD[0]
    for i in range(START+1,END):
        print("{:d}.pcd: ".format(i))
        target = allPCD[i]
        evaluation = open3d.evaluate_registration(source, target,threshold, trans_init)
        reg_p2p = open3d.registration_icp(source, target, threshold, trans_init,open3d.TransformationEstimationPointToPoint())
        source.transform(reg_p2p.transformation)
    visualizePCD(source)