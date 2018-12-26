## Project: Kinematics Pick & Place
---
**Steps to complete the project:**  

1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 

[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png
[robot_DH]: ./misc_images/

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.

[This](README.md) is the writeup for this project.

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

The pick and place simulator was ran in demo mode to analyze the kinematics of the Kuka KR210 robot. Steps to run this is available in [this](Project_Setup.md) file.

The `kr210.urdf.xacro` is the URDF(Unified Robot Description Format) file that contains the robot model. The below image shows the robot.

![alt text][image1]

For performing the kinematic analysis of the robot, the `joint` tag has to be examined. The following is the joint tag from base_link(parent) to link 1(child).
```xml
<joint name="joint_1" type="revolute">
    <origin xyz="0 0 0.33" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="link_1"/>
    <axis xyz="0 0 1"/>
    <limit lower="${-185*deg}" upper="${185*deg}" effort="300" velocity="${123*deg}"/>
  </joint>
```
The origin sub-tag provides information about the relative position of the child with respect to the parent link. 

The relative position of joints extracted from the urdf file is summarized in the below table.

Joint Name | Parent link | Child link | x(m) | y(m) | z(m) | roll | pitch | yaw
--- | --- | --- | --- | --- | --- | --- | --- | ---
joint_1 | base_link | link_1 | 0 | 0 | 0.33 | 0 | 0 | 0
joint_2 | link_1 | link_2 | 0.35 | 0 | 0.42 | 0 | 0 | 0
joint_3 | link_2 | link_3 | 0 | 0 | 1.25 | 0 | 0 | 0
joint_4 | link_3 | link_4 | 0.96 | 0 | -0.054 | 0 | 0 | 0
joint_5 | link_4 | link_5 | 0.54 | 0 | 0 | 0 | 0 | 0
joint_6 | link_5 | link_6 | 0.193 | 0 | 0 | 0 | 0 | 0
gripper_joint | link_6 | gripper_link | 0.11 | 0 | 0 | 0 | 0 | 0

The following figure shows these measurements in the robot joint space.

![URDF measurements][robot_DH]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | L1 | qi
1->2 | - pi/2 | L2 | 0 | -pi/2 + q2
2->3 | 0 | 0 | 0 | 0
3->4 |  0 | 0 | 0 | 0
4->5 | 0 | 0 | 0 | 0
5->6 | 0 | 0 | 0 | 0
6->EE | 0 | 0 | 0 | 0


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles. 

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


