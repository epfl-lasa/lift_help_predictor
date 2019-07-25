# Recording

### Overview 
There are two things that are recorded to a [ROS bag file](http://wiki.ros.org/Bags): OptiTrack marker poses, and a video from a USB camera. 

Before testing below the scripts below, ensure the following:

* The OptiTrack motion capture system is on and configured properly.
* A USB camera is attached to your PC. 

### Testing the Recording Scripts 
The [roslaunch](http://wiki.ros.org/roslaunch) file used for recording is ```launch/record_experiment.launch```. It can be run by executing the following command: 

``` 
$ roslaunch lift_help_predictor record_experiment.launch 
```

After launching this code, the USB camera and OptiTrack data should be visible in two separate windows. However, recording *will not* begin. Before recording, check that OptiTrack is publishing the ROS topics you would like to record: 

```
$ rostopic list
```

To begin and stop recording, use the following commands:

```
 # To start recording
 $ rosservice call /record/cmd record

 # To finish recording
 $ rosservice call /record/cmd stop
```

After running these commands, a new bag file beginning with "test" should appear in the ```data/bag``` directory. 

### Configuring the Recording

The following arguments can also be passed to ```record_experiment.launch```: 
* *file_name*: name to be appended to the timestamp when recording started.
* *topic*: the ROS topics to record, matching names selected in Optitrack.  
* *path_save*: path to the directory where the .bag file will be recorded.
* *rviz_config*: the display configuration for 3D pose data.

To see the default values for these arguments, read  ```record_experiment.launch```. 
  
