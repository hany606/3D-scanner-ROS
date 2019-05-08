import numpy as np
import open3d
import copy

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

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    # visualizePCD([source_temp,target_temp])
    visualizePCD(source_temp)

if __name__ == "__main__":
    print("Hello to the main file")

    # pcd = openPCD(PATH+'14.pcd')
    # downpcd = open3d.voxel_down_sample(pcd, voxel_size = 0.0001)
    # open3d.estimate_normals(downpcd, search_param = open3d.KDTreeSearchParamHybrid(radius = 0.1, max_nn = 30))
    # print(np.asarray(pcd.points))
    # visualizePCD(pcd)


    
    # # Open all the Point Cloud files
    allPCD = []
    r = 0
    g = 0
    b = 0
    for i in range (START,END):
        pcd = openPCD(PATH+str(i)+".pcd")
        pcd.paint_uniform_color([r, g, b])
        r += 0.008
        g += 0.005
        b += 0.003
        print(r)
        allPCD.append(pcd)
        print("{:d}.pcd: ".format(i),end="")
        print(pcd)
    visualizePCD(allPCD)
    # # pcd.paint_uniform_color([0.5, 0.5, 0.5])
    # # pcd_tree = open3d.KDTreeFlann(pcd)
    # # pcd.colors[1000] = [1, 0, 0]
    
    # # [k, idx, _] = pcd_tree.search_knn_vector_3d(pcd.points[1000], 10)
    # # np.asarray(pcd.colors)[idx[1:], :] = [0, 0, 1]
    
    # # [k, idx, _] = pcd_tree.search_radius_vector_3d(pcd.points[1000], 0.2)
    # # np.asarray(pcd.colors)[idx[1:], :] = [0, 1, 0]
    
    # # visualizePCD(pcd)

    r = 0
    g = 0.5
    b = 0.5

    threshold = 0.05
    trans_init = np.asarray(
                [[1, 0, 0,  0],
                [0, 1, 0,  0],
                [0, 0,  1, 0],
                [0.0, 0.0, 0.0, 1.0]])
    # source = allPCD[0]
    # finalPCD = [source,source]
    # for i in range(35+1,50):
    #     print("{:d}.pcd: ".format(i))
    #     target = allPCD[i]
        
    #     evaluation = open3d.evaluate_registration(source, target,threshold, trans_init)
    #     reg_p2p = open3d.registration_icp(source, target, threshold, trans_init,open3d.TransformationEstimationPointToPoint())
    #     target.paint_uniform_color([r,g,b])
    #     r += 0.008
    #     finalPCD.append(target)
    #     source.transform(reg_p2p.transformation)
    # finalPCD[0] = source
    # for i in range(START, 50+1):
    #     visualizePCD(finalPCD[i])

    # evaluation = open3d.evaluate_registration(source, target,threshold, trans_init)
    # reg_p2p = open3d.registration_icp(source, target, threshold, trans_init,open3d.TransformationEstimationPointToPoint())
    # target.paint_uniform_color([r,g,b])
    # r += 0.008
    # finalPCD.append(target)
    # source.transform(reg_p2p.transformation)
    # visualizePCD(allPCD[0])
    # visualizePCD(allPCD[40])
    source = allPCD[0]
    source_cpy = copy.deepcopy(source)
    # draw_registration_result(source,allPCD[40],trans_init)
    print("##########START##########")
    prev_transform = trans_init
    for i in range(1,END):
        print("{:d}.pcd: ".format(i))
        target = allPCD[i]
        
        evaluation = open3d.evaluate_registration(source, target,threshold, prev_transform)
        reg_p2p = open3d.registration_icp(source, target, threshold, prev_transform, open3d.TransformationEstimationPointToPoint())
        target.paint_uniform_color([r,g,b])
        r += 0.008
        # finalPCD.append(target)
        prev_transform = reg_p2p.transformation
        source.transform(reg_p2p.transformation)

    # draw_registration_result(source,source_cpy,trans_init)
    visualizePCD([source_cpy,source])