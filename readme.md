# 3D Detection & Tracking Viewer
The project is based on hailanyi/3D-Detection-Tracking-Viewer and is modified, you can find the original version of the code below:
https://github.com/hailanyi/3D-Detection-Tracking-Viewer

This project was developed for viewing 3D object detection results from the Dair-V2X-I datasets.

It supports rendering 3D bounding boxes and rendering boxes on images.

## Features

* Captioning box ids(infos) in 3D scene
* Projecting 3D box or points on 2D image  

## Design pattern
This code includes two parts, one for convert tools, other one for visualization of 3D detection results.
## Change log
* (2022.02.01) Adapted to the Dair-V2X-I dataset
## Prepare data 
* Dair-V2X-I detection dataset
* Convert the Dair-V2X-I dataset to kitti format using the conversion tool

## Requirements (Updated 2021.11.2)
```
python==3.7.11
numpy==1.21.4
vedo==2022.0.1
vtk==8.1.2
opencv-python==4.1.1.26
matplotlib==3.4.3
open3d==0.14.1
```
It is recommended to use anaconda to create the visualization environment
```
conda create -n dair_vis python=3.8
```
To activate this environment, use
```
conda activate dair_vis
```
Install the requirements
```
pip install -r requirements.txt
```
To deactivate an active environment, use
```
conda deactivate
```



## Convert tools 
* Prepare a dataset of the following structure:
* "kitti_format" must be an empty folder to store the conversion result
* "source_format" to store the source Dair-V2X-I datasets.
```
# For Dair-V2X-I Dataset  
dair_v2x_i
├── kitti_format
├── source_format
│   ├── single-infrastructure-side
│   │   ├── calib
│   │   │   ├── camera_intrinsic
│   │   │   └── virtuallidar_to_camera
│   │   └── label
│   │       ├── camera
│   │       └── virtuallidar
│   ├── single-infrastructure-side-example
│   │   ├── calib
│   │   │   ├── camera_intrinsic
│   │   │   └── virtuallidar_to_camera
│   │   ├── image
│   │   ├── label
│   │   │   ├── camera
│   │   │   └── virtuallidar
│   │   └── velodyne
│   ├── single-infrastructure-side-image
│   └── single-infrastructure-side-velodyne

```

* If you have the same folder structure, you only need change the "root path" to your local path from config/config.yaml
* Running the jupyter notebook server and open the "convert.ipynb"
* The code is very simple , so there are no input parameters for advanced customization, you need to comment or copy the code to implemented separately following functions :
-Convert calib files to KITTI format
-Convert camera-based label files to KITTI format
-Convert lidar-based label files to KITTI format
-Convert image folders to KITTI format
-Convert velodyne folders to KITTI format

After the convet you will get the following result.
the 
```      
dair_v2x_i
├── kitti_format
│   ├── calib
│   ├── image_2
│   ├── label_2
│   ├── label_velodyne
│   └── velodyne
 
```
* The label_2 base the camera label, and use the lidar label information replace the size information(w,h,l). In the camera view looks like better.
* The label_velodyne base the velodyne label.
* P2 represents the camera internal reference, which is a 3×3 matrix, not the same as KITTI. It convert frome the "cam_K" of the json file.
* Tr_velo_to_cam: represents the camera to lidar transformation matrix, as a 3×4 matrix.


## Usage
#### 1. Set the path to the dataset folder used for input to the visualizer
If you have completed the conversion operation, the path should have been set correctly. Otherwise you need to set "root_path" in the config/config.yaml to the correct path

#### 2. Choose whether camera or lidar based tagging for visualization
You need to set the "label_select" parameter in config.yaml to "cam" or "vel", to specify the label frome label_2 or velodyne_label.

#### 2. Run and Terminate
* You can start the program with the following command
```
python dair_3D_detection_viewer.py
```
* Pressing space in the lidar window will display the next frame
* Terminating the program is more complicated, you cannot terminate the program at static image status. You need to press the space quickly to make the frames play continuously, and when it becomes obvious that the system is overloaded with resources and the program can't respond, press Ctrl-C in the terminal window to terminate it. Try a few more times and you will eventually get the hang of it.

## Notes on the Dair-V2X-I dataset
* In the calib file of this dataset, "cam_K" is the real intrinsic matrix parameter of the camera, not "P". Although they are very close in value and structure.
* There are multiple camera images with different focal and perspectives in this dataset, and the camera intrinsic matrix reference will change with each image file. Therefore, when using this dataset, please make sure that the calib file you are using corresponds to the image file (e.g. do not use only the 000000.txt parameter for all image files)
* The sequence of files in this dataset is non-contiguous (e.g. missing the 000023), do not only use 00000 to lens(dataset) to get the sequence of file names directly.
* The dataset provides optimized labels for both lidar and camera, and after testing, there are errors in the projection of the lidar label on camera (but the projection matrix is correct, only the label itself has issues). Likewise, there is a disadvantage of using the camera's label in lidar. Therefore it is recommended to use the corresponding label for lidar, and use the fused label for the camera.
* There are some other objects in the label, for example you can see some trafficcone.
