# Baxter recorder
This node records frames (including end effectors) in JSON, cameras (including kinect and depth or other external camera) in AVI/H264 and actions (custom package) in JSON of experiments using Baxter.

## Recording
### Usage
Make sure all the transform frames frames you are interested in are specified within the [config file](config/frames.yaml). Then the basic usage is as follow:
```
roslaunch recorder record.launch
```
The recorder works in interactive mode, takes several seconds to setup and then asks the user to press Enter to start recording:
```
[INFO] [WallTime: 1455640471.609206] Creating new empty dataset under /home/user/ros_ws/src/recorder/data/dataset_machine_7209_8380647351778469359
[INFO] [WallTime: 1455640471.683224] Opening camera left_hand_camera in 1280x800 pixels...
[INFO] [WallTime: 1455640474.662197] Opening camera right_hand_camera in 1280x800 pixels...
[INFO] [WallTime: 1455640477.634414] Cameras opened!
[INFO] [WallTime: 1455640477.634700] Waiting all subscribers to be ready...
Subscribers ready, press <Enter> to start recording...
[INFO] [WallTime: 1455640479.332954] Starting recording ... press <Ctrl-C> to stop
^C
```
The default records left_gripper, right_gripper (in frames.json), left_hand_camera (left.avi) and right_hand_camera (right.avi).
It may also record other sources, assuming that their dependencies are installed and have started publishing:
- Any other frame published on /tf (feature `frames`, just update the file [`frames.yaml`](config/frames.yaml) and add the name of the new frames to record
- Kinect RGB (feature `kinect`, assuming that openni_launch is publishing with the default camera name "camera")
- Kinect Depth (feature `depth`)
- Head camera (feature `head`, provided by an external webcam, this CANNOT be Baxter's camera since only 2 cameras can be opened at the same time)
- Actions (feature `actions`, see the warning betlow)

Features can be enabled in command line:
```
roslaunch recorder record.launch kinect:=true depth:=true head:=true
```

### Warning about thr_infrastructure_msgs
This recorder also records a custom topic (actions) from a custom package.
Using this script without noticing this warning will more likely raise an ImportError, you should comment the import as well as the subscriber declaration in that case:
```
#from thr_infrastructure_msgs.msg import ActionHistoryEvent
...
#self.actions_sub = rospy.Subscriber('/thr/action_history', ActionHistoryEvent, self.cb_action_history, queue_size=100)
```
Note that commenting this will disable the `actions` feature.

### Waiting components
The recorder checks that all recordings sources (aka components) are setup before starting recording, to make sure that no data will be empty.
If the recorder loops indefinitely because of a non-ready component, check that it is being published (openni for the kinect for instance).

## Using the recorded data
### Format description
The cameras output a video each in a dataset folder created on-the-fly and containing the following files:

- `frames.json`: Position and orientation of each object/frame according to each time frame. It also contains the time in seconds associated to each time frame (starting at 0 second)
- `depth.avi`: Kinect depth
- `head.avi`: Baxter's head published on `/usb_camera/image_raw` (By an additional camera since Baxter is not able to open more than 2 cameras at a time)
- `left.avi`: Baxter's left arm
- `right.avi`: Baxter's right arm
- `actions.json`: history of the robot actions executed during the recording, as well as their status succeeded or failed (custom action framework)

All the files are synchronized in this way:
- Each time frame `i` of a video source corresponds to the frame `i` of any other video and to the `i`th element of `frames.json`
- The time in seconds of `actions.json` corresponds to the time in seconds of `frames.json` at +/- 0.1 sec.

### Framerate and resolution

- Object pose recordings at 20fps in `frames.json`
- Head camera: mpeg4, H264, yuv420p, 1280x720 (16/9), bgr8 (colour), 20fps
- Left and right cameras: mpeg4, H264, yuv420p, 1280x800 (8/5), bgr8 (colour), 20fps
- Kinect RGB: mpeg4, H264, yuv420p, 640x480 (4:3), bgr8 (colour), 20fps
- Kinect Depth: mpeg4, H264, yuv420p, 640x480 (4:3), 8UC1 (greyscale), 20fps
- The framerate of 20fps can be changed in argument, if the camera source provides less images then duplicated images will be added to match the requested framerate

### Format of `frames.json`

If `frames` is the root of the JSON file, then `frames['metadata']` containts information about the recording:
```
{'objects':
    ['right_gripper', 'left_gripper', 'base'],  # Name of the recorder objects/frames (feature 'frames')
 'timestamp': 1455645293.507784,  # Timestamp of shooting (given for information, but the recorded time starts at 0)
 'world': 'base'}  # Name of the world frame
```
`frames['transforms']` is a list of each time frame, e.g. at time frame 10:
`frames['transforms'][10]` =
```
{'objects':                # Dict of all requested frames, whatever they are or aren't visible
   'right_gripper': {      # First frame is the right_gripper
       'visible': True}},  # The frame right_gripper is visible at time frame 10, so the 'pose' field is available
           {'pose':                     # pose of this object, existing only if object is visible
               [[0.5145395144762627,    # x position wrt the world frame specified in metadata
                 -0.7726182517639723,   # y position wrt the world frame specified in metadata
                 0.13528087874252306],  # z position wrt the world frame specified in metadata
                [-0.4949455884830569,   # qx quaternion wrt the world frame specified in metadata
                 0.5162929784057031,    # qy quaternion wrt the world frame specified in metadata
                 -0.45660650551546555,  # qz quaternion wrt the world frame specified in metadata
                 0.5291322367906568]]}, # qw quaternion wrt the world frame specified in metadata
   [...]                                # Other objects of this time frame are here
 'time': 0.451503039}                   # Time in seconds of time frame 10
 
```
### Format of `actions.json`

```
[
    {
      "action": "grasp",                # Name of the action
      "arm": "right",                   # Action client executing the action
      "event": "start",                 # Type of event ("start", "success" or "failure")
      "parameters": ["glass"],          # Parameters of the action
      "time": 5.204932928               # Timestamp (synchronized with field 'time' in frame.json
    },
    {
      "action": "pick",
      "arm": "left",
      "event": "start",
      "parameters": ["bottle"],
      "time": 7.381244898
    },
    [...]
]
```

## Notes
 - In the terms used by this recorder, make sure your do not mix up time frames and transform frames (tf)
 - The action list (`actions.json`) does not contain any reference to time frames at the moment, only the time in seconds (TODO: to be fixed)
