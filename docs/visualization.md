# Visualization

### Overview 
The data for each recording can be visualized as seen in the demo. The following can be seen: 

* A 3D animation of the [Lifting from the Deep](https://github.com/DenisTome/Lifting-from-the-Deep-release) output creating after parsing a rosbag file. 
* A visualization of the OptiTrack markers in [rviz](http://wiki.ros.org/rviz). 
* A video playback of the USB camera output 
* A 2D plot of selected metrics (Pose, Velocity, Acceleration)  

### Testing the Visualization Scripts
The [roslaunch](http://wiki.ros.org/roslaunch) file used for recording is ```launch/rosbag_visualize.launch```. It can be run by executing the following command: 

```
$ roslaunch lift_help_predictor rosbag_visualize.launch 
``` 

This should display the demo data, which is included in the repository. 

### Configuring the Visualization 
The following arguments can also be passed to ```rosbag_visualize.launch```: 

* *bag_path*: name of the bag file to visualize. 
* *rviz_config*: configuration file for OptiTrack marker visualization. 
* *rate*: playback speed as a fraction. Ex. 0.5 is half speed. 
* *delay*: delay in seconds before the animation begins.
* *samples_in_average*: for 2D visualizations, running averages of velocity and accelerations are computed. Select the number of samples considered. 
* *optitrack_rate_hz*: OptiTrack speed; used to more accurately compute velocities and accelerations. 

Read the comments in ```rosbag_visualize.launch``` to help understand the underlying code.  


