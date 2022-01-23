import numpy as np
from vedo import *
import cv2
import vtk
from .color_map import generate_objects_color_map,generate_objects_colors,generate_scatter_colors
from .box_op import convert_box_type,get_line_boxes,get_mesh_boxes,velo_to_cam,get_box_points
import copy
class Viewer:
    """
    default box type: "OpenPCDet", (x,y,z,l,w,h,yaw)
    """
    def __init__(self,box_type = "OpenPCDet",bg=(255, 255, 255)):
        self.objects_color_map = generate_objects_color_map('rainbow')
        self.box_type = box_type
        self.vi = Plotter(bg=bg)
        self.set_lights()

        # data for rendering in 3D scene
        self.actors = []
        self.actors_without_del = []
        self.tracks_actors_dict = {}

        # data for rendering in 2D scene
        self.cam_intrinsic_mat = None
        self.cam_extrinsic_mat = None
        self.boxes_info = [] # (boxes:array(N,7), ids:array(N,), colors:array(N,3) or str, box_info:list(N,))
        self.points_info = [] # (boxes:array(N,3), colors:array(N,3) or str)
        self.image = None



    def set_lights(self):
        def get_light(pos=(0, 0, 0), focalPoint=(0, 0, 0)):
            light = vtk.vtkLight()

            light.SetPosition(pos)
            light.SetFocalPoint(focalPoint)
            light.SetIntensity(0.65)

            return light

        light_actors = []

        light_actors.append(get_light(pos=(400, 400, 100), focalPoint=(0, 0, 0)))
        light_actors.append(get_light(pos=(-400, 400, 100), focalPoint=(0, 0, 0)))
        light_actors.append(get_light(pos=(400, -400, 100), focalPoint=(0, 0, 0)))
        light_actors.append(get_light(pos=(-400, -400, 100), focalPoint=(0, 0, 0)))
        for a in light_actors:
            self.vi.renderer.AddLight(a)


    def set_ob_color_map(self,color_map_name='rainbow'):
        """
        seting objects colors map, all the options are same as matplotlab.pypot
        reference  https://matplotlib.org/stable/tutorials/colors/colormaps.html
        :param color_map_name: (str), the name of objects color map, such as "rainbow", "viridis","brg","gnuplot","hsv"
        :return: (list), a list of random colors
        """
        self.objects_color_map = generate_objects_color_map(color_map_name)

        return self.objects_color_map

    def set_ego_car(self,ego_car_path = "viewer/ego_car.3ds"):
        """
        setting ego car
        :param ego_car_path: (str), path of ego car model
        :return:
        """
        ego_car = load(ego_car_path)
        ego_car.pos(-0.5, 0, -1.6)
        ego_car.scale(0.9)
        self.actors_without_del+=[ego_car]

    def set_intrinsic_mat(self,intrinsic_mat):
        """
        set the camera intrinsic matrix
        :param intrinsic_mat: (array or list(4,4)), intrinsic matrix
        :return:
        """
        self.cam_intrinsic_mat = intrinsic_mat

    def set_extrinsic_mat(self,extrinsic_mat):
        """
        set the camera extrinsic matrix (velo 3D coordinates to cam 3D)
        :param extrinsic_mat: (array or list(4,4)), extrinsic matrix
        :return:
        """
        self.cam_extrinsic_mat = extrinsic_mat

    def add_points(self,points,
                   radius = 2,
                   color = (150,150,150),
                   scatter_filed=None,
                   alpha=1,
                   del_after_show='True',
                   add_to_3D_scene = True,
                   add_to_2D_scene = True,
                   color_map_name = "rainbow"):
        """
        add the points actor to viewer
        :param points: (list or array(N,3)),
        :param r: (float), radius of points to show
        :param c: (str,list(N,4),array(N,4)), color name or a list of rgba colors
        :param scatter_filed: (list(N,),array(N,)), scatter filed rendering to colors
        :param alpha:  (float), [0,1] transparency of points actor
        :param del_after_show: (bool), clear the points actor after show
        :param add_to_3D_scene: (bool)
        :param add_to_2D_scene: (bool)
        :param color_map_name: (str), the name of objects color map, such as "rainbow", "viridis","brg","gnuplot","hsv"
        :return:
        """
        if scatter_filed is not None:   #进这里
            colors = generate_scatter_colors(scatter_filed,color_map_name=color_map_name)#设定rgb颜色
        else:
            colors = color

        if add_to_2D_scene:   #进这里
            self.points_info.append((points,colors))#添加进队列，xyz和RGB

        if add_to_3D_scene:   #进这里
            if del_after_show:   #进这里
                self.actors.append(Points(points,r=radius,c=colors,alpha=alpha))#猜测actors是3d里的point存储list

            else:
                self.actors_without_del.append(Points(points,r=radius,c=colors,alpha=alpha))

    def add_spheres(self,points,
                    radius = 0.3,
                    color='red',
                    res=30,
                    scatter_filed=None,
                    alpha=0.5,
                    del_after_show='True'):
        """
        add the spheres actor to viewer
        :param points: (list or array(N,3)), the centers of spheres
        :param radius: (float), radius of points to show
        :param color: (str,list(N,4),array(N,4)), color name or a list of rgba colors
        :param res: (float), resolution of spheres
        :param scatter_filed: (list(N,),array(N,)), scatter filed rendering to colors
        :param alpha:  (float), [0,1] transparency of points actor
        :param del_after_show: (bool), clear the points actor after show
        :return:
        """

        if scatter_filed is not None:
            colors = generate_scatter_colors(scatter_filed)[:,:3]
        else:
            colors = color

        if del_after_show:
            self.actors.append(Spheres(points,r=radius,res=res,c=colors,alpha=alpha))

        else:
            self.actors_without_del.append(Spheres(Points,r=radius,res=res,c=colors,alpha=alpha))

    def add_3D_boxes(self,boxes=None,
                     ids=None,
                     box_info=None,
                     color="blue",
                     add_to_3D_scene=True,
                     mesh_alpha = 0,
                     show_corner_spheres = True,
                     corner_spheres_alpha = 1,
                     corner_spheres_radius=0.1,
                     show_heading = True,
                     heading_scale = 1,
                     show_lines = True,
                     line_width = 2,
                     line_alpha = 1,
                     show_ids = True,
                     show_box_info=True,
                     del_after_show=True,
                     add_to_2D_scene=True,
                     caption_size=(0.05,0.05)
                     ):
        """
        add the boxes actor to viewer
        :param boxes: (array(N,7)), 3D boxes
        :param ids: list(N,), the ID of each box
        :param box_info: (list(N,)), a list of str, the infos of boxes to show
        :param color: (str),the default color of boxes
        :param add_to_3D_scene: (bool)
        :param mesh_alpha: (float), the transparency of box mesh
        :param show_corner_spheres: (bool), show the corner points of box
        :param corner_spheres_alpha: (float), the transparency of corner spheres
        :param corner_spheres_radius: (float), the radius of of corner spheres
        :param show_heading: (bool), show the box heading
        :param heading_scale: (float), the arrow size of heading
        :param show_lines: (bool), show the lines of box
        :param line_width: (float), line width
        :param line_alpha: (float), line transparency
        :param show_ids: (bool), show object ids in the 3D scene
        :param show_box_info: (bool)，show object infos in the 3D Scene
        :param del_after_show: (bool), clear the boxes after show
        :param add_to_2D_scene: (bool), add the boxes to images
        :return:
        """
        if boxes is None:
            return
        boxes= convert_box_type(boxes,self.box_type)#这里，转换不同数据集xyzwlhangle到统一的顺序
        if boxes is None:
            return

        if ids is not None:
            colors = generate_objects_colors(ids,self.objects_color_map)
        else:
            colors = color#这里，颜色保持函数传入时的，默认蓝色

        if add_to_2D_scene:
            self.boxes_info.append((boxes,ids,colors,box_info))#把boxes的7个数组和ids不知道是啥，和颜色，和分类添加到统一的boxes_info里

        if add_to_3D_scene:
            if del_after_show:#进这里
                self.actors += get_mesh_boxes(boxes,#返回vtk格式的box list
                                              colors,
                                              mesh_alpha,
                                              ids,
                                              show_ids,
                                              box_info,
                                              show_box_info,
                                              caption_size)
                self.actors += get_line_boxes(boxes,#返回vtk格式的线段 list
                                              colors,
                                              show_corner_spheres,
                                              corner_spheres_alpha,
                                              corner_spheres_radius,
                                              show_heading,
                                              heading_scale,
                                              show_lines,
                                              line_width,
                                              line_alpha)
            else:
                self.actors_without_del += get_mesh_boxes(boxes,
                                                          colors,
                                                          mesh_alpha,
                                                          ids,
                                                          show_ids,
                                                          box_info,
                                                          show_box_info,
                                                          caption_size)
                self.actors_without_del += get_line_boxes(boxes,
                                                          colors,
                                                          show_corner_spheres,
                                                          corner_spheres_alpha,
                                                          corner_spheres_radius,
                                                          show_heading,
                                                          heading_scale,
                                                          show_lines,
                                                          line_width,
                                                          line_alpha)

    def add_3D_cars(self,boxes=None,
                     ids=None,
                     box_info=None,
                     color="blue",
                     mesh_alpha = 0.1,
                     show_ids = False,
                     show_box_info=False,
                     del_after_show=True,
                     car_model_path="viewer/car.obj",
                     caption_size = (0.1, 0.1)
                    ):

        if boxes is None:
            return
        boxes= convert_box_type(boxes,self.box_type)#这里，依旧先转换数据集格式到统一的7位list
        if boxes is None:
            return

        if ids is not None:
            colors = generate_objects_colors(ids,self.objects_color_map)
        else:
            colors = color#这里，依旧直接使用传入颜色

        for i in range(len(boxes)):
            bb = boxes[i]#临时变量bb，取出boxes的一组数据

            size = bb[3:6]#取whl三个数字

            ang=bb[6]#取出角度
            ang = int(ang / (2 * np.pi) * 360)#转换角度

            if type(colors) is str:
                color = colors#赋值颜色
            else:
                color = colors[i]

            if ids is not None:#不进入
                ob_id = ids[i]
                if ob_id in self.tracks_actors_dict.keys():
                    previous_ori=self.tracks_actors_dict[ob_id].GetOrientation()[2]
                    self.tracks_actors_dict[ob_id].pos(0,0,0)
                    self.tracks_actors_dict[ob_id].rotateZ(ang-previous_ori)
                    self.tracks_actors_dict[ob_id].pos(bb[0], bb[1], bb[2])

                    info = ""
                    if ids is not None and show_ids:
                        info = "ID: " + str(ids[i]) + '\n'
                    if box_info is not None and show_box_info:
                        info += str(box_info[i])
                    if info != '':
                        self.tracks_actors_dict[ob_id].caption(info,
                                                               point=(bb[0], bb[1] - bb[4] / 2, bb[2] + bb[5] / 2),
                                                               size=caption_size,
                                                               alpha=1,
                                                               c=color,
                                                               font="Calco",
                                                               justify='left')
                        self.tracks_actors_dict[ob_id]._caption.SetBorder(False)
                        self.tracks_actors_dict[ob_id]._caption.SetLeader(False)

                    if del_after_show:
                        self.actors.append(self.tracks_actors_dict[ob_id])
                    else:
                        self.actors_without_del.append(self.tracks_actors_dict[ob_id])
                else:

                    new_car=load(car_model_path)
                    new_car.scale((0.12,0.3,0.3))

                    new_car.scale(size)
                    new_car.rotateZ(ang)
                    new_car.pos(bb[0], bb[1], bb[2])

                    new_car.c(color)
                    new_car.alpha(mesh_alpha)
                    self.tracks_actors_dict[ob_id]=new_car
                    info = ""
                    if ids is not None and show_ids:
                        info = "ID: " + str(ids[i]) + '\n'
                    if box_info is not None and show_box_info:
                        info += str(box_info[i])
                    if info != '':
                        self.tracks_actors_dict[ob_id].caption(info,
                                                               point=(bb[0], bb[1] - bb[4] / 2, bb[2] + bb[5] / 2),
                                                               size=caption_size,
                                                               alpha=1,
                                                               c=color,
                                                               font="Calco",
                                                               justify='left')
                        self.tracks_actors_dict[ob_id]._caption.SetBorder(False)
                        self.tracks_actors_dict[ob_id]._caption.SetLeader(False)

                    if del_after_show:
                        self.actors.append(self.tracks_actors_dict[ob_id])
                    else:
                        self.actors_without_del.append(self.tracks_actors_dict[ob_id])

            else:#进入这里，添加车辆模型
                new_car = load(car_model_path)
                new_car.scale((0.12, 0.3, 0.3))

                new_car.scale(size)
                new_car.rotateZ(ang)
                new_car.pos(bb[0], bb[1], bb[2])

                new_car.c(color)
                new_car.alpha(mesh_alpha)

                info = ""

                if box_info is not None and show_box_info:
                    info += str(box_info[i])
                if info != '':
                    new_car.caption(info,
                                   point=(bb[0], bb[1] - bb[4] / 2, bb[2] + bb[5] / 2),
                                   size=caption_size,
                                   alpha=1,
                                   c=color,
                                   font="Calco",
                                   justify='cent')
                    new_car._caption.SetBorder(False)
                    new_car._caption.SetLeader(False)
                if del_after_show:
                    self.actors.append(new_car)
                else:
                    self.actors_without_del.append(new_car)

    def add_image(self,im,deep_copy):
        """
        add images for display
        :param im: (array(W,H,3)), image array
        :return:
        """
        if deep_copy==True:
            self.image = copy.deepcopy(im)
        else:
            self.image=im
        return

    def show_3D(self):
        """
        show objects in 3D scenes, before show_3D, you should add some objects into the current scenes
        :param bg_color: (tuple(3,) or list(3,) or str), background color of 3D scene
        :return:
        """
        #self.vi.show(self.actors+self.actors_without_del,axes=1,resetcam=False, camera={'pos': (-10, 0, 5), 'focalPoint': (5, 0, 2), 'viewup': (0, 0, 1)})
        
        self.vi.show(self.actors+self.actors_without_del,axes=1,resetcam=True,camera={'pos': (-100, -100, 150), 'focalPoint': (5, 0, 2), 'viewup': (0, 0, 1)})
        self.vi.clear()
        if False:   #调试外参关闭
            self.actors.clear()
            self.points_info.clear()
            self.boxes_info.clear()

    def show_2D(self,box_color = (255,0,0),show_box_info=False,show_ids=True,points_colors=(0,0,255)):
        """
        show object on image
        :param box_color: (list or tuple(3,)), default color
        :param show_box_info: (bool), show box infos
        :param show_ids: (bool),show box ids
        :param show_ids: (tuple(3,),default points color
        :return:
        """

        if (self.cam_extrinsic_mat is None) or (self.cam_intrinsic_mat is None) or (self.image is None):
            return

        H,W,_ = self.image.shape#这里按图像设置长宽

        for info in self.boxes_info:    #将boxes_info中7位+类等list，取出一条给info
            boxes, ids, colors, box_info=info   #然后将info这一条分别赋值给4个变量方便使用

            if boxes is None:
                continue#不进入这里
            elif len(boxes) == 0:
                continue#不进入这里
            else:
                        #下列只是添加bounding box的
                for box_id in range(len(boxes)):
                    box = boxes[box_id]#boxes是xyz坐标，按顺序取出一个
                    if type(colors) is not str:
                        color = [colors[box_id][2],colors[box_id][1],colors[box_id][0]]#按RGB设置颜色，跳过
                    else:
                        color = box_color#设置颜色

                    pts_3d_cam = get_box_points(box)    #得到bounding box在原始lidar坐标系下，添加了朝向，只会影响朝向暂时跳过排查
                    pts_3d_cam = velo_to_cam(pts_3d_cam[:,0:3],self.cam_extrinsic_mat)#此时有归一化系数，取前三。继续添加外惨运算，到3d相机坐标系，最大嫌疑在此处

                    img_pts = np.matmul(pts_3d_cam, self.cam_intrinsic_mat.T)  # (N, 3)#运算内参，到图像坐标系，此处没问题，所以最大问题就在这个之前
                    x, y = img_pts[:, 0] / img_pts[:, 2], img_pts[:, 1] / img_pts[:, 2]

                    x = np.clip(x, 2, W-2)
                    y = np.clip(y, 2, H-2)

                    x = x.astype(np.int)
                    y = y.astype(np.int)

                    self.image[y, x] = color

                    x2 = x + 1
                    self.image[y, x2] = color
                    y2 = y + 1
                    self.image[y2, x] = color
                    self.image[y2, x2] = color

                    info = ""
                    if ids is not None and show_ids:
                        info +=  str(ids[box_id])+" "
                    if box_info is not None and show_box_info:
                        info += str(box_info[box_id])

                    if info != '':

                        text = info
                        org = ((max(x) - min(x)) // 2 + min(x), min(y) - 5)
                        fontFace = cv2.FONT_HERSHEY_DUPLEX
                        fontScale = 0.7
                        fontcolor = color  # BGR
                        thickness = 1
                        lineType = 4
                        cv2.putText(self.image, text, org, fontFace, fontScale, fontcolor, thickness, lineType)

        if True:
            #此处添加lidar点云到2D画面
            for points,colors in self.points_info:

                if type(colors) is tuple:

                    color = [colors[2],colors[1],colors[0]]
                else:
                    color = points_colors   #设置颜色

                pts_3d_cam = velo_to_cam(points[:, 0:3], self.cam_extrinsic_mat)

                img_pts = np.matmul(pts_3d_cam, self.cam_intrinsic_mat.T)  # (N, 3)
                x, y = img_pts[:, 0] / img_pts[:, 2], img_pts[:, 1] / img_pts[:, 2]

                x = np.clip(x, 2, W - 2)
                y = np.clip(y, 2, H - 2)

                x = x.astype(np.int)
                y = y.astype(np.int)
                self.image[0,0]=[0,0,0]

                self.image[y, x] = color

        cv2.imshow('im',self.image)
        cv2.waitKey(1)
        if False:   #调试外参关闭
            self.points_info.clear()
            self.boxes_info.clear()

