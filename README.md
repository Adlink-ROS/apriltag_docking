# apriltag_docking

This repo shows the docking demo by leverging the AprilTag and NeuronBot2 with ROS 2 foxy-devel.

![](readme_resource/apriltag_docking.gif)


## Download packages
```bash
mkdir -p autodock_ros2_ws/src
cd ~/autodock_ros2_ws/
wget https://raw.githubusercontent.com/Adlink-ROS/neuronbot2_ros2.repos/z_demo-apriltag/neuronbot2_ros2.repos
vcs import src < neuronbot2_ros2.repos
```

## Install dependencies
```bash
cd ~/autodock_ros2_ws/
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro foxy
```

## Build packages
```bash
cd ~/autodock_ros2_ws/
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
``` 

## AprilTag configurations
If you want to change the settings of AprilTag, e.g. tag family, tag size and tag_ids, please check below yaml files:

```bash
cd ~/autodock_ros2_ws/
vim src/apriltag_ros/apriltag_ros/cfg/tags_36h11_filter.yaml
vim src/apriltag_docking/autodock_controller/param/neuronbot.yaml
```
In ```tags_36h11_filter.yaml```, set tag **size**, tag **family**, and **tag_frames**.

```yaml
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
Also check ```neuronbot.yaml```

```yaml
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

## Simulation in Gazebo

Step 1. Launch Neuronbot2 and AprilTag model in Gazebo

```bash
ros2 launch neuronbot2_gazebo neuronbot2_world.launch.py world_model:=tag.model use_camera:=top
```

Step 2. Launch apriltag_docking

```bash
ros2 launch apriltag_docking autodock_gazebo.launch.py open_rviz:=true
```

Step 3. Send request to start docking

```bash
ros2 run apriltag_docking docking_client --ros-args -p docking:=start
```

## Use Physical NeuronBot2 and RealSense D435

Below instructions are only for users having a real NeuronBot2 with RealSense D435

Step 1. Launch Neuronbot2 and RealSense in real world

```bash
ros2 launch neuronbot2_bringup bringup_launch.py use_camera:=top
```

Step 2. Launch apriltag_docking

```bash
ros2 launch apriltag_docking autodock_neuronbot.launch.py open_rviz:=true
```

Step 3. Send request to start docking

```bash
ros2 run apriltag_docking docking_client --ros-args -p docking:=start
```

## Integrating with BehaviorTree

Download BT_ros2:

```bash
cd ~/autodock_ros2_ws/src
git clone https://github.com/Adlink-ROS/BT_ros2.git
```

Build BT_ros2 with auto-docking workspace:

```bash
cd ~/autodock_ros2_ws/
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_AUTODOCK=ON
```

Run BT with auto-docking XML:

```bash
cd ~/autodock_ros2_ws/
source /opt/ros/foxy/setup.bash
source ~/autodock_ros2_ws/install/local_setup.bash
ros2 launch bt_ros2 bt_ros2.launch.py bt_xml:=/home/ros/autodock_ros2_ws/src/BT_ros2/bt_xml/bt_auto_docking.xml
```