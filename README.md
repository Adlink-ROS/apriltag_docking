# apriltag_docking

## Install
```
mkdir -p autodock_ros2_ws/src
cd ~/autodock_ros2_ws/src
git clone https://github.com/H-HChen/neuronbot2.git
git clone https://github.com/H-HChen/apriltag_ros.git -b foxy-devel
git clone https://github.com/AprilRobotics/apriltag.git
git clone https://github.com/H-HChen/apriltag_docking.git 
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install    #ignore all warning plz 
``` 

## Modify tag family, tag size and tag_ids
```
cd ~/autodock_ros2_ws/
vim src/apriltag_ros/apriltag_ros/cfg/tags_36h11_filter.yaml
vim src/apriltag_docking/autodock_controller/param/neuronbot.yaml
```
### tags_36h11_filter.yaml
Set tags size and tag family and tag_frames
```
image_transport: 'raw'    # image format
family: '36h11'           # tag family name
size: 0.08
threads: 2
max_hamming: 0          # maximum allowed hamming distance (corrected bits)
z_up: true              # rotate about x-axis to have Z pointing upwards

# see "apriltag.h" for more documentation on these optional parameters
decimate: 1.0           # decimate resolution for quad detection
blur: 1.0               # sigma of Gaussian blur for quad detection
refine-edges: 1         # snap to strong gradients
debug: 0                # write additional debugging images to current working directory
tag_ids: [0]            # tag ID
tag_frames: [dock_frame]  # optional frame name
tag_sizes: [0.08]   # optional tag-specific edge size
```
### neuronbot.yaml

Set tag family and tad id
```
autodock_controller:
  ros__parameters:
      cmd_vel_angular_rate: 0.25
      cmd_vel_linear_rate: 0.25
      default_turn: 1.0
      final_approach_distance: 1.0
      finish_distance: 0.5
      jog_distance: 0.2
      lost_tag_max: 5
      max_center_count: 10
      tune_angle: 0.42
      tag_frame: "dock_frame"
```
## Simulation in gazebo
1. Launch Neuronbot2 and tag in gazebo
```
ros2 launch neuronbot2_gazebo neuronbot2_world.launch.py world_model:=tag.model use_camera:=top
```
2. Launch apriltag_docking 

    Remember to change names of camera_namespace and topic name
```
ros2 launch apriltag_docking autodock_gazebo.launch.py
```
