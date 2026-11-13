# Robot_manipulator

Robot_manipulator is a 5-DoF robot arm in ROS Melodic and Gazebo. It finds an object with an RGB-D camera, estimates the object's 6-DoF pose and picks it up. I developed it as a personal project in 2020.

![Object detection with ORB features, in simulation](pictures/obj_detection.png)

## What I did
- **Object detection and 6-DoF pose.** find_object_2d matches ORB features of trained objects in the camera image. With the depth image and PnP, it publishes each object's 6-DoF pose as a TF frame.
- **Table-top segmentation.** A PCL nodelet pipeline downsamples the point cloud, estimates normals and segments the table plane. It then extracts the objects above the table with a convex hull and a polygonal prism.
- **Obstacle-aware planning.** MoveIt plans with an IKFast solver for the 5-DoF arm. A pass-through filter feeds the depth cloud into an OctoMap, so plans avoid obstacles the camera sees.
- **Grasping.** Python planners read the object's TF frame and drive the arm through the grasp. The arm moves above the object, orients and opens the gripper, descends, grasps and lifts.
- **Dynamic grasping.** When the tracked object is more than 10 cm from the gripper along x or y, the arm plans a new approach and follows it.

## Results
In Gazebo the robot detects a trained object, estimates its pose and grasps it. If the object is moved on the table, the arm follows it.

## How to run
I built and ran it on Ubuntu 18.04 with ROS Melodic (desktop-full) in 2020. I haven't rerun these steps since.

```bash
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone https://github.com/AdilShaikh1/Robot_manipulator
git clone https://github.com/JenniferBuehler/gazebo-pkgs            # gazebo_grasp_plugin
git clone https://github.com/JenniferBuehler/general-message-pkgs    # messages that gazebo-pkgs needs
cd ~/catkin_ws && rosdep install --from-paths src --ignore-src -y
catkin_make && source devel/setup.bash
```

Detect an object and pick it up:

```bash
roslaunch five_dof_manipulator arm6d.launch     # Gazebo world, arm, controllers, object detection
roslaunch five_dof_manipulator moveit6d.launch  # MoveIt, RViz and the OctoMap
rosrun five_dof_manipulator jointplan.py        # find object_27 and grasp it
```

For dynamic grasping, run `demo6d.launch` instead of `arm6d.launch`, then run `rosrun five_dof_manipulator demo_obj.py` and drag the can in Gazebo.

## Repository layout
- `five_dof_manipulator/`: robot description and meshes, Gazebo worlds, controllers, perception launch files, planners and demos
- `robot_moveit_config/`: MoveIt configuration (OMPL and CHOMP pipelines)
- `robot_arm_ikfast_plugin/`: IKFast kinematics plugin for the arm
- `pictures/`: screenshots

## Media
![The arm tracks a can as it is moved on the table](pictures/dynamic_obj_grab.png)

![Pose estimation in RViz from the depth camera's point cloud](pictures/pose_estimation.png)

![The arm at the pick table in the Gazebo world](pictures/pnc1.png)

![The arm between two tables in the Gazebo world](pictures/pnc2.png)

## License
The five_dof_manipulator package and the pictures are MIT licensed (see LICENSE). Generated IKFast solver code keeps its original license header.
