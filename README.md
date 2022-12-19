# MTE 544 Final Project - Kalman Filter Localization
> By Ayush Ghosh


## Setup
To setup the `mte544_kalman_filter` package, build and source this workspace: 

```colcon build```

```source install/setup.bash```

## Running the setup:
Ensure the workspace is sourced in the terminal:

```source install/setup.bash```

1. Launch mte544_kalman_node and Rviz using the launch file:

```ros2 launch mte544_kalman_filter run_kalman_filter.launch.py```


2. Wait for the launch file to finish loading. A Rviz window will popup.
Open a new terminal in the root of this workspace, and run the `path` bag file:

```ros2 bag play src/mte544_kalman_filter/bag_files/path```

Wait a few moments for the bag file to start sending data. The robot TF, the odometry path (dotted red line) and the kalman filter estimated path will also show (green solid line). The MSE value will also be printed from the launch terminal. 

## Viewing the State Estimation plots

As the node is running, it will keep track of the state estimates. Once you are done with the RViz visualization, hit `Ctl^C` to stop the node, which will also automatically display the state estimate plot.

## Trial Videos:
Each trial/expriment conducted with this kalman filter has been recorded and can be found here: `Trial_Videos`. Inside this folder, the videos have been group according to what setting was changed in the trials.