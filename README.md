# rmcl_example

Examples to test the [rmcl](https://github.com/uos/rmcl) ROS package.

## Worlds, Maps and Simulation

In the `maps` folder there are several triangle meshes that can be used as a map for the robot.
The robot and each map can be loaded into one simulation by calling

```console
$ roslaunch rmcl_example start_robot.launch
```

The map can be changed by either changing the launch file's default arguments or via command line.


After that, try to visualize the triangle mesh map via RViz.
[rmcl](https://github.com/uos/rmcl) itself does not provide any mesh visualization tools.
I myself use a RViz plugin of the [mesh_tools](https://github.com/uos/mesh_tools). The results should look as follows:

|  Gazebo  |  RViz  |
|:--------:|:------:|
| ![Cube World Gazebo](dat/img/cube_gazebo.png "Cube World Gazebo") | ![Cube Map Rviz](dat/img/cube_rviz.png "Cube Map Rviz") |
| ![Sphere World Gazebo](dat/img/sphere_gazebo.png "Sphere World Gazebo") | ![Sphere Map Rviz](dat/img/sphere_rviz.png "Sphere Map Rviz") |
| ![Cylinder World Gazebo](dat/img/cylinder_gazebo.png "Cylinder World Gazebo") | ![Cylinder Map Rviz](dat/img/cylinder_rviz.png "Cylinder Map Rviz") |
| ![Tray World Gazebo](dat/img/tray_gazebo.png "Tray World Gazebo") | ![Tray Map Rviz](dat/img/tray_rviz.png "Tray Map Rviz") |

Alternatively maps of existing simulations as in [uos_tools](https://github.com/uos/uos_tools) can be used.

## MICP Localization

Change the default arguments in `launch/rmcl_micp.launch` to your needs:

```xml
...
<arg name="map" default="$(find rmcl_example)/maps/cube.dae" />
<arg name="config" default="$(find rmcl_example)/config/micp_cpu.yaml"/>
...
```

and call

```console
$ roslaunch rmcl_example rmcl_micp.launch
```

The outputs should look as follows:

```console
MICP initiailized

-------------------------
    --- BACKENDS ---    
-------------------------
Available combining units:
- CPU
- GPU
Selected Combining Unit: CPU

Available raytracing backends:
- Embree (CPU)
- Optix (GPU)

MICP load params

-------------------------
     --- FRAMES ---      
-------------------------
- base:			base_footprint
- odom:			odom_combined
  - base -> odom:	yes
- map:			map
Estimating: base_footprint -> map
Providing: odom_combined -> map

-------------------------
     --- SENSORS ---     
-------------------------
- velodyne
  - data:		Topic
    - topic:		/velodyne/points
    - msg:		sensor_msgs/PointCloud2
    - data:		yes
    - frame:		velodyne
  - model:		Params
  - type:		spherical - loaded
  - micp:
    - backend:		embree
MICP load params - done. Valid Sensors: 1
TF Rate: 50
Waiting for pose guess...
```

After that you can set a pose in RViz via `2D Pose Estimate` and see the robot localizing itself given the range measurements of the Velodyne LiDaR.

![MICP](dat/vid/rmcl_micp_1280.gif)

If visualizations are enabled in the micp config file, the ray casting correspondences (RCC) can be visualized per sensor `X` as marker on the topic `micp_localizations/sensors/X/correspondences`.

![Ray casting correspondences (RCC)](dat/img/spc.png "Ray casting correspondences (RCC)")

