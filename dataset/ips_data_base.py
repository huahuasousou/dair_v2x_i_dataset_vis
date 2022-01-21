import os
import cv2
import re
import open3d as o3d
import numpy as np

"""
input: calib txt path
return: P2: (4,4) 3D camera coordinates to 2D image pixels
        vtc_mat: (4,4) 3D velodyne Lidar coordinates to 3D camera coordinates
"""
def read_calib(calib_path):

    K11=np.array((9.837163130824076e+02,0,0,\
        0,8.595545273466768e+02,0,\
        9.404020659139954e+02,5.756271947729268e+02,1),np.float32)


    K11=K11.reshape((3,3)).T
    K11= np.insert(K11, 3, values=0, axis=1)
    K11= np.insert(K11, 3, values=0, axis=0)    
    print("K11:",K11)


    P11=np.array((0.14200562691952617, -0.9888265486076867, -0.045348193919961095, 1.0285320021923907,\
                -0.4095134968418579, -0.0169787561689188, -0.9121460506647551, -1.2267393975465035,\
                0.9011842751776943, 0.14810056923444437, -0.4073488966044802, 0.12756363884073307,\
                0, 0, 0, 1),np.float32)
    
    P11=P11.reshape((4,4))
    #P11[0:3,0:3]=P11[0:3,0:3].T
    
    print("P11:",P11)

    test_P=np.array([9.9613460808857857e-01,-8.4555111027209293e-02,-2.3796549484966172e-02,5.2099604408767163e+03,\
                -4.4009003785814141e-02,-2.4595980581401863e-01,-9.6828042503693657e-01,-2.6677084559796695e+04,\
                7.6020064154105588e-02,9.6558490415038412e-01,-2.4873026097139672e-01,9.3040370481562015e+04,\
                0, 0, 0, 1])

    test_P=np.reshape(test_P, [4, 4])

    return (K11, P11)


"""
description: read lidar data given 
input: lidar bin path "path", cam 3D to cam 2D image matrix (4,4), lidar 3D to cam 3D matrix (4,4)
output: valid points in lidar coordinates (PointsNum,4)
"""
def load_pcd_velo(velo_filename, n_vec=4):
    scan=o3d.io.read_point_cloud(velo_filename)
    scan=np.asarray(scan.points)
    scan=np.insert(scan, 3, values=0.2, axis=1)
    scan = scan.reshape((-1, n_vec))
    return scan

def read_velodyne(path, P, vtc_mat,IfReduce=True):
    max_row = 1080  # y
    max_col = 1920  # x
    lidar=load_pcd_velo(path)
    #lidar = np.fromfile(path, dtype=np.float32).reshape((-1, 4))
    print("lidar:",lidar)
    if not IfReduce:
        return lidar

    mask = lidar[:, 0] > 0
    lidar = lidar[mask]
    lidar_copy = np.zeros(shape=lidar.shape)
    lidar_copy[:, :] = lidar[:, :]

    velo_tocam = vtc_mat
    lidar[:, 3] = 1
    lidar = np.matmul(lidar, velo_tocam.T)
    img_pts = np.matmul(lidar, P.T)
    velo_tocam = np.mat(velo_tocam).I
    velo_tocam = np.array(velo_tocam)
    normal = velo_tocam
    normal = normal[0:3, 0:4]
    lidar = np.matmul(lidar, normal.T)
    lidar_copy[:, 0:3] = lidar
    x, y = img_pts[:, 0] / img_pts[:, 2], img_pts[:, 1] / img_pts[:, 2]
    mask = np.logical_and(np.logical_and(x >= 0, x < max_col), np.logical_and(y >= 0, y < max_row))

    return lidar_copy[mask]


"""
description: convert 3D camera coordinates to Lidar 3D coordinates.
input: (PointsNum,3)
output: (PointsNum,3)
"""
def cam_to_velo(cloud,vtc_mat):
    mat=np.ones(shape=(cloud.shape[0],4),dtype=np.float32)
    mat[:,0:3]=cloud[:,0:3]
    mat=np.mat(mat)
    normal=np.mat(vtc_mat).I
    normal=normal[0:3,0:4]
    transformed_mat = normal * mat.T
    T=np.array(transformed_mat.T,dtype=np.float32)
    return T

"""
description: convert 3D camera coordinates to Lidar 3D coordinates.
input: (PointsNum,3)
output: (PointsNum,3)
"""
def velo_to_cam(cloud,vtc_mat):
    mat=np.ones(shape=(cloud.shape[0],4),dtype=np.float32)
    mat[:,0:3]=cloud[:,0:3]
    mat=np.mat(mat)
    normal=np.mat(vtc_mat).I
    normal=normal[0:3,0:4]
    transformed_mat = normal * mat.T
    T=np.array(transformed_mat.T,dtype=np.float32)
    return T

def read_image(path):
    im=cv2.imdecode(np.fromfile(path, dtype=np.uint8), -1)
    return im

def read_detection_label(path):

    boxes = []
    names = []

    with open(path) as f:
        for line in f.readlines():
            line = line.split()
            this_name = line[0]
            if this_name != "DontCare":
                line = np.array(line[-7:],np.float32)
                boxes.append(line)
                names.append(this_name)

    return np.array(boxes),np.array(names)

def read_tracking_label(path):

    frame_dict={}

    names_dict={}

    with open(path) as f:
        for line in f.readlines():
            line = line.split()
            this_name = line[2]
            frame_id = int(line[0])
            ob_id = int(line[1])

            if this_name != "DontCare":
                line = np.array(line[10:17],np.float32).tolist()
                line.append(ob_id)


                if frame_id in frame_dict.keys():
                    frame_dict[frame_id].append(line)
                    names_dict[frame_id].append(this_name)
                else:
                    frame_dict[frame_id] = [line]
                    names_dict[frame_id] = [this_name]

    return frame_dict,names_dict

if __name__ == '__main__':
    path = 'H:/dataset/traking/training/label_02/0000.txt'
    labels,a = read_tracking_label(path)
    print(a)

