# Introduction
2D Project Audio Playing operates using ROS2.\
The workflow of the programe is as such:
```
Publish a waypoint using command → Actives an audio track → Publish completion
```

## How to run
**Step 1**:
```
colcon build --packages-select audio_state voice_package waypoint_state
```
**Step 2**: Run this on every new terminal.
```
cd ~/ros2_ws
```
```
source install/setup.bash
```
**Step 3**:
```
ros2 run voice_package sub_publish
```
**Step 4**: On a different terminal.
```
ros2 run voice_package voice_activator
```
**Step 5(Optional)**: On a different termine to view audio state topic.
```
ros2 topic echo /audio_state
```
**Step 6**: Insert a waypoint.
```
ros2 run voice_package sub_client <waypoint>
```

## Requirements
If pip is not found for python3.10
```
curl -sS https://bootstrap.pypa.io/get-pip.py | python3.10
```
```
python3.10 -m pip install --upgrade pip
```
If alsa-utils is not installed
```
sudo apt install alsa-utils
```
Install pygame
```
python3 -m pip install pygame
```
