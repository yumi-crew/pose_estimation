Installation:
~~~~
mkdir -p /home/$USER/pose_estimation/src
# Clone repo into src
vcs import /home/$USER/pose_estimation_ws/src < /home/$USER/pose_estimation_ws/src/pose_estimation/pose_estimation.repos
cd /home/$USER/pose_estimation_ws/
colcon build --symlink-install
source install/local_setup.bash
~~~~


External dependencies:
  - Eigen3
  ~~~~
  sudo apt install libeigen3-dev
  ~~~~
  - OpenCV
  ~~~~
  mkdir opencv && cd opencv
  git clone https://github.com/opencv/opencv.git
  git clone https://github.com/opencv/opencv_contrib.git
  mkdir build && cd build
  cmake -D WITH_OPENMP=ON -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib/modules ../opencv
  make -j8
  sudo make install
  ~~~~
  - Xtensor and xtl (xtensor template library) [Install sequentially]
  ~~~~
  mkdir xtl && cd xtl
  git clone https://github.com/xtensor-stack/xtl.git
  mkdir build && cd build
  cmake -D CMAKE_INSTALL_PREFIX=/usr/local ../xtl
  make -j8
  sudo make install
  ~~~~
  ~~~~
  mkdir xtensor && cd xtensor
  git clone https://github.com/xtensor-stack/xtensor.git
  mkdir build && cd build
  cmake -DCMAKE_INSTALL_PREFIX=/usr/local ../xtensor
  make -j8
  sudo make install
  ~~~~
  - Xtensor-BLAS
  ~~~~
  mkdir xtensor_blas && cd xtensor_blas
  git clone https://github.com/xtensor-stack/xtensor-blas.git
  mkdir build && cd build
  cmake -DCMAKE_INSTALL_PREFIX=/usr/local ../xtensor-blas
  make -j8
  sudo make install
  ~~~~
  - PCL version 1.10 https://github.com/PointCloudLibrary/pcl/releases/tag/pcl-1.10.0
  ~~~~
  mkdir pcl && cd pcl
  [Do not use this (1.81...), download from link above] git clone https://github.com/PointCloudLibrary/pcl.git
  mkdir build && cd build
  cmake ../pcl
  make -j2
  sudo make install
  ~~~~
  - HALCON: https://www.mvtec.com/products/halcon/
