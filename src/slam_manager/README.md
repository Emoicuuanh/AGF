# How to install

1. If catkin_make error regarding to `cv_bridge`:

   Refer: [https://github.com/ros-perception/vision_opencv/issues/389](https://github.com/ros-perception/vision_opencv/issues/389)

   ```bash
   cd /usr/include/
   sudo ln -s opencv4/ opencv (replace "4" with your version)
   ```
2. If topic `/map_image/full/compressed` is not published:

   ```bash
   sudo apt install ros-noetic-image-transport-plugins
   ```
