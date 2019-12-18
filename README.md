# Introduction
[![N|Solid](http://turlucode.com/wp-content/uploads/2017/10/turlucode_.png)](http://turlucode.com/)

Track [ArUco markers](https://docs.opencv.org/4.1.2/d9/d6d/tutorial_table_of_content_aruco.html) using MATLAB and OpenCV. ![A test image](https://turlucode.com/wp-content/uploads/2019/12/aruco-axis.png)

To date, MATLAB is not providing any toolbox which can detect ArUco markers. The only way to have any support on ArUco markers, is via the [OpenCV C++ API](https://nl.mathworks.com/matlabcentral/fileexchange/47953-computer-vision-toolbox-opencv-interface).

# Using the library

## Prerequisites

### OpenCV

You need to have [OpenCV](https://opencv.org/) together with the [`opencv_contrib`](https://github.com/opencv/opencv_contrib) projects installed in your computer.

Most Linux distros provide OpenCV via their package manager. You can however also [compile manually](https://docs.opencv.org/4.1.2/d7/d9f/tutorial_linux_install.html):


Here is an example on how you can manually compile and install OpenCV 4.1.2:
```sh
# Close OpenCV
git clone https://github.com/opencv/opencv.git && cd opencv && git checkout 4.1.2 && cd .. &&

# Clone opencv_contrib
git clone https://github.com/opencv/opencv_contrib.git && cd opencv_contrib && git checkout 4.1.2 && cd ..

# Build OpenCV
cd opencv && mkdir build && cd build && \
cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D INSTALL_C_EXAMPLES=OFF \
      -D INSTALL_PYTHON_EXAMPLES=OFF \
      -D WITH_V4L=ON \
      -D WITH_QT=ON \
      -D WITH_OPENGL=ON \
      -D WITH_CUBLAS=1 \
      -D WITH_TBB=ON \
      -D WITH_OPENMP=ON \
      -D WITH_CUDA=ON \
      -D ENABLE_PRECOMPILED_HEADERS=OFF \
      -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
      -D BUILD_EXAMPLES=OFF \
      .. && make -j4

# Install OpenCV
sudo make install

# Remove repos
cd ../.. rm -rf /opencv && rm -rf /opencv_contrib
```

### MATLAB Computer Vision Toolbox OpenCV Interface

The [Computer Vision Toolbox OpenCV Interface](https://nl.mathworks.com/matlabcentral/fileexchange/47953-computer-vision-toolbox-opencv-interface) of MATLAB also needs to be installed. For that, you need to have a mathworks account.

To install it, just use the "Add-Ons" button of the "Home" tab:  ![A test image](https://turlucode.com/wp-content/uploads/2019/12/matlab_addons.png)

This is the addon you should install:

![A test image](https://turlucode.com/wp-content/uploads/2019/12/computer_vision_toolbox_opencv_interface.png)


## Build MEX Function

Before you can build the [`mex`](https://nl.mathworks.com/help/matlab/matlab_external/choosing-mex-applications.html) function, you need to locate:

- the OpenCV include files (header files), e.g. by using `sudo find / -name 'libopencv_aruco*' 2>/dev/null`
- the OpenCV library files (lib files), e.g. by using `sudo find / -name 'aruco.hpp' 2>/dev/null`

In the below example the OpenCV 4.1.2 
- headers are located in `/usr/local/include/opencv4` and the
- lib files are located in `/usr/local/lib`.

To build the `aruco_detect.cpp` mex function, navigate within MATLAB to the root of this project and run the following command (in MATLAB of course):

```sh
mexOpenCV -L/usr/local/lib -lopencv_aruco -I/usr/local/include/opencv4 aruco_detect.cpp
```

## Run the mex function

### Add missing OpenCV libs in MATLAB's path

Before you can run the mex function you need to create a symbolic link of the `libopencv_aruco.so.*` file in the MATLAB search path, otherwise MATLAB cannot locate it in run-time.

In the below example the `libopencv_aruco.so.*` file is located in `/usr/local/lib/` (as also mentioned above).

```sh
ln -s /usr/local/lib/libopencv_aruco.so.4.1 <MATLAB_ROOT_FOLDER_PATH>/bin/glnxa64/libopencv_aruco.so.4.1
```

If you are using a newer version of gcc than MATLAB is, then you need to link the `libstdc++.so.6` as well.

First create a backup of the existing one:

```sh
mv <MATLAB_ROOT_FOLDER_PATH>/sys/os/glnxa64/libstdc++.so.6 <MATLAB_ROOT_FOLDER_PATH>/sys/os/glnxa64/libstdc++.so.6.backup
```

then link the systems `libstdc++.so.6` to MATLAB's search-path:

```sh
ln -s /usr/lib/libstdc++.so.6 <MATLAB_ROOT_FOLDER_PATH>/sys/os/glnxa64/libstdc++.so.6
```

If you still getting errors when running the mex function you can also try to start MATLAB as:

```sh
LD_PRELOAD=/usr/lib/libstdc++.so.6 <MATLAB_ROOT_FOLDER_PATH>/bin/matlab -desktop
```

### Run the example

Now you can run the example `test/aruco_detect_test.m` file.

# Issues and Contributing
  - Please let us know by [filing a new 
issue](https://github.com/turlucode/matlab-aruco/issues/new).
  - You can contribute by [opening a pull 
request](https://github.com/turlucode/matlab-aruco/compare).