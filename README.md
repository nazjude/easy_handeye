# hkclr_easy_handeye

## Prerequisites

- Ubuntu 18.04
- ROS2 Melodic

### Install

1. If you are on a fresh install, create a ROS workspace (here we use `~/handeye_ws`).

```
cd ~/handeye_ws/src
git clone --recurse-submodules https://github.com/nazjude/hkclr_easy_handeye.git
```

2. if you have had the repo cloned and want to update, you could:

```
git submodule update --init --recursive
git pull
```

3. Compile the ROS workspace with `catkin_make` in `~/handeye_ws`.

```
cd ~/handeye_ws
catkin_make
```

### Test
1. Launch calibration tool:

```
cd ~/handeye_ws
bash smarteye_calib_run.bash 
```

2. Take a photo in a new terminal:


```
source ~/handeye_ws/devel/setup.bash
rosservice call /hv1000/get_imageWithDepth "header:(tab for the rest)
```
