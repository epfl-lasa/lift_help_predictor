# Parsing

### Overview 
In order to extract only joint and object pose data, [ROS bag](http://wiki.ros.org/Bags) files may be parsed into .csv files. As well, using the USB camera recording, human joint positions are retreieved using the [Lifting from the Deep](https://github.com/DenisTome/Lifting-from-the-Deep-release) algorithm.

Files are retrieved from the ```data/bag``` directory, and stored in the ```data/csv``` directory. The current implementation creates a .csv file for each recording. 

### Testing the Parsing Scripts 
The [roslaunch](http://wiki.ros.org/roslaunch) file used for recording is ```launch/rosbag_to_csv.launch```. It can be run by executing the following command: 

```
$ roslaunch lift_help_predictor rosbag_to_csv.launch
```

This should, by default, reprocess the ```demo.bag``` file,
and overwrite the ```demo.csv``` file. See ```demo.csv``` to view the format of the outputted data. 

### Configuring the Parsing
The following arguments can also be passed to ```rosbag_to_csv.launch```:

* *participant*: the script will process all the bag files that contain this string. 
* *topic_names*: rostopics to be parsed, which should match the recorded topics in the bag file. 

To change any of the parsing details, see the code in ```scripts/parse_rosbag.py```. 