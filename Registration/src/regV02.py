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
    # source_temp.transform(transformation)
    visualizePCD([source_temp,target_temp])
    # visualizePCD(source_temp)

if __name__ == "__main__":
    print("Hello to the main file")

    
    # # Open all the Point Cloud files
    allPCD = []
    r = 0
    g = 0
    b = 0
    for i in range (START,END):
        pcd = openPCD(PATH+str(i)+".pcd")
        # pcd.paint_uniform_color([r, g, b])
        # r += 0.008
        # g += 0.005
        # b += 0.003
        # print(r)
        allPCD.append(pcd)
        # print("{:d}.pcd: ".format(i),end="")
        # print(pcd)
    # visualizePCD(allPCD)

    r = 0
    g = 0.5
    b = 0.5

    target_num = 3
    threshold = 0.05
    trans_init = np.asarray(
                [[1, 0, 0,  0],
                [0, 1, 0,  0],
                [0, 0,  1, 0],
                [0.0, 0.0, 0.0, 1.0]])
    prev_transform = copy.deepcopy(trans_init)

    source = allPCD[START]
    source_cpy = copy.deepcopy(source)
    # draw_registration_result(source,allPCD[40],trans_init)
    print("##########START Merging##########")
    print("{:d}.pcd: ".format(target_num))
    target = allPCD[target_num]

    evaluation = open3d.evaluate_registration(source, target,threshold, trans_init)
    print(evaluation)

    reg_p2p = open3d.registration_icp(source, target, threshold, trans_init, open3d.TransformationEstimationPointToPoint(), open3d.ICPConvergenceCriteria(max_iteration = 100))
    print(reg_p2p)
    print("Transformation is:")
    print(reg_p2p.transformation)
    print("")
    
    target.paint_uniform_color([r,g,b])
    r += 0.008
    # finalPCD.append(target)
    source.transform(reg_p2p.transformation)
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1,0,0.5])        #after transformation-Red
    target_temp.paint_uniform_color([0, 0.651, 0.929])  #Target-Blue
    source_cpy.paint_uniform_color([1, 0.706, 0])       #Source-Orange
    # source_temp.transform(transformation)
    #then we need to merge the two point clouds and make them as one then it will be the new base
    #merge target and source_tmp
    visualizePCD([source_temp,target_temp,source_cpy])
    print(np.asarray(source_temp.points))
    # tmp = np.add(source_temp,target_temp)
    pcdd = open3d.geometry.PointCloud()
    pcdd = source_temp + target_temp
    visualizePCD(pcdd)
    print(source_temp)
    print(pcdd)
    #conver the new array to point cloud
    # np_points = np.random.rand(100, 3)
    # From numpy to Open3D
    # visualizePCD(pcdd)
    # source_temp.points = tmp
    # print(source_temp)


    
    # for i in range(START+1,START+2):
    #     print("{:d}.pcd: ".format(i))
    #     target = allPCD[i]
        
    #     evaluation = open3d.evaluate_registration(source, target,threshold, prev_transform)
    #     reg_p2p = open3d.registration_icp(source, target, threshold, trans_init, open3d.TransformationEstimationPointToPoint(), open3d.ICPConvergenceCriteria(max_iteration = 2000))
    #     print(reg_p2p)
    #     print("Transformation is:")
    #     print(reg_p2p.transformation)
    #     print("")
        
    #     target.paint_uniform_color([r,g,b])
    #     r += 0.008
    #     # finalPCD.append(target)
    #     source.transform(reg_p2p.transformation)
    #     # prev_transform = reg_p2p.transformation


    # draw_registration_result(source,source_cpy,trans_init)
    # visualizePCD(source)
    # visualizePCD()