```bash

sudo apt install python3-numpy

```

The cv_bridge python backend still has a dependency on python boost (`equal or higher than 1.58.0`), and install them as follows in Ubuntu:

```bash

sudo apt install libboost-python-dev

```

## Build

### Fetch the latest code and build

```bash

cd <YOUR_ROS2_WORKSPACE>
git clone https://github.com/ros-perception/vision_opencv.git
cd vision_opencv
git checkout ros2
colcon build --symlink-install

```

```

```
