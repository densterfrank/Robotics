# Installation
To install and set up the necessary ROS packages for running the multi-robot code in your Ubuntu 20.04 virtual machine environment, follow these steps:

Open a terminal within your **Ubuntu 20.04 virtual machine**.

Change directory to the source folder of your AIIL workspace in your Noetic workspace:

```
cd aiil_workspace/noetic_workspace/src/
```

Clone the required repository by running the following command:

```
git clone https://github.com/s3894695/multi_robot.git
```

This will clone the repository containing the multi-robot code into the source folder of your Noetic workspace.

By completing these steps, you will have successfully installed the required packages for running the multi-robot code.

# Running ROS Code on Multiple Robots

To run the ROS code on multiple robots, follow the steps below:

### Step 1 
Open the **first terminal** and SSH into robot1 (Trenzalore):
`ssh husarion@[robot name]`

```
ssh husarion@trenzalore
```

### Step 2
Open the **second terminal** and SSH into robot2 (Targaryen):
`ssh husarion@[robot name]`

```
ssh husarion@targaryen
```

Once both terminals are connected to the respective robots, proceed to the next step.

### Step 3
Set robot2 (Targaryen) to the ROS master to robot1 (Trenzalore):

On the **second terminal**, enter the following command

```
set_ros_ip
set_ros_master trenzalore
```

### Step 4
Launch the main ROS package on robot1 (Trenzalore)

In the **first terminal**, set the ROS namespace to robot1 and launch the main package:

```
ROS_NAMESPACE=robot1 roslaunch multi_robot main.launch
```

### Step 5
Launch the ROS package on robot2 (Targayen)

In the **second terminal**, set the ROS namespace to robot2 and launch the same ROS package as before:

```
ROS_NAMESPACE=robot2 roslaunch multi_robot main.launch
```

### Step 6
Select a Pattern and Launch it on the Master Robot

Choose one of the available patterns, either `straightPattern.launch` or `goalPattern.launch`, based on your requirements.

Open the **third terminal** and SSH into the master robot (Trenzalore): `ssh husarion@[master robot name]`

```
ssh husarion@trenzalore
```

Once logged into the master robot, run the following command to launch the selected pattern (assuming you chose goalPattern.launch): `roslaunch multi_robot [Pattern].launch`
```
roslaunch multi_robot goalPattern.launch
```

This step will start the execution of the chosen pattern on the master robot, allowing it to coordinate and control the behavior of both robot1 and robot2 based on the selected pattern.
