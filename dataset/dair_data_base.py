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
    with open(calib_path) as f:
        for line in f.readlines():
            """
            if line[:2] == "P2":
                P2 = re.split(" ", line.strip())
                P2 = np.array(P2[-12:], np.float32)
                P2 = P2.reshape((3, 4))
            """
            if line[:2] == "P2":
                P2 = re.split(" ", line.strip())
                P2 = np.array(P2[-9:], np.float32)
                P2 = P2.reshape((3, 3))
                b = np.array([[0,0,0]])
                P2=np.c_[P2,b.T]

            if line[:14] == "Tr_velo_to_cam" or line[:11] == "Tr_velo_cam":
                vtc_mat = re.split(" ", line.strip())
                vtc_mat = np.array(vtc_mat[-12:], np.float32)
                vtc_mat = vtc_mat.reshape((3, 4))
                vtc_mat = np.concatenate([vtc_mat, [[0, 0, 0, 1]]])
    return (P2, vtc_mat)


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
    if not IfReduce:
        return lidar

    mask = lidar[:, 0] > 0
    lidar = lidar[mask]
    lidar_copy = np.zeros(shape=lidar.shape)
    lidar_copy[:, :] = lidar[:, :]#读取拷贝一份点云list

    velo_tocam = vtc_mat#复制一份外参
    lidar[:, 3] = 1#点云xyz把强度值设为1，其实应该是为了接下来运算当作系数
    lidar = np.matmul(lidar, velo_tocam.T)#这里进行了一次运算！变换到3d相机坐标系了！跟住lidar看看去哪儿了
    img_pts = np.matmul(lidar, P.T)#变换到相机3d坐标系后，可以继续变换到2D图像坐标系了，应该是为了删除无用点云，跟住img_pts看看去哪儿了
    velo_tocam = np.mat(velo_tocam).I#这里还取逆是为了逆变换，注意T向量也取逆了，这个应该是争取的方式。R和转置效果一样，但是T向量只能取逆。之前的转制不是为了旋转R矩阵，只是为了批量运算
    velo_tocam = np.array(velo_tocam)#再转为数组
    normal = velo_tocam#这里又取了一次外参？？？
    normal = normal[0:3, 0:4]#删掉已经无用的最后一行0,0,0,1
    lidar = np.matmul(lidar, normal.T)#这里又做了一次逆运算,T是为了批量运算。又回到lidar坐标系了，验证此时数据恢复为最初，和copy一致
    lidar_copy[:, 0:3] = lidar#这里不明白，xyz重新赋值，最后一列强度值0.2也没有删掉，除了损失精度还有什么意义？
    x, y = img_pts[:, 0] / img_pts[:, 2], img_pts[:, 1] / img_pts[:, 2]#这个函数里面的2d运算看懂了，是为了取得视角mask，删掉无用点云，不涉及2d图像显示，但是会影响3d视野可见点云
    mask = np.logical_and(np.logical_and(x >= 0, x < max_col), np.logical_and(y >= 0, y < max_row))
    return lidar_copy
    #return lidar_copy[mask]#暂时屏蔽，输出全部点云


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
            if len(line)==16:# test result
                if this_name != "DontCare":
                    line = np.array(line[-8:-1],np.float32)
                    boxes.append(line)
                    names.append(this_name)
            else:   
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

