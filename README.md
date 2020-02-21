Installation:
~~~~
mkdir -p /home/$USER/pose_estimation/src
# Clone repo into src
vcs import /home/$USER/pose_estimation_ws/src < /home/$USER/pose_estimation_ws/src/pose_estimation/pose_estimation.repos
cd /home/$USER/pose_estimation_ws/
colcon build --symlink-install
source install/local_setup.bash`
~~~~


External dependencies:

  -Eigen3
  ~~~~
  sudo apt isntall libeigen3-dev
  ~~~~
  - OpenCV
  ~~~~
  mkdir OpenCV_build && cd OpenCV_build
  git clone https://github.com/opencv/opencv.git
  cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local ..
  ~~~~
  - Xtensor and xtl (xtensor template library)
  ~~~~
  mkdir xtl_build && cd xtl_build
  git clone https://github.com/xtensor-stack/xtl.git
  cmake -D CMAKE_INSTALL_PREFIX=usr/local ..
  make install
  ~~~~
  ~~~~
  mkdir xtensor_build && cd xtensor_build
  git clone https://github.com/xtensor-stack/xtensor.git
  cmake -DCMAKE_INSTALL_PREFIX=/usr/local ..
  make install
  ~~~~
  - Xtensor-BLAS
  ~~~~
  mkdir xtensor_blas_build && cd xtensor_blas_build
  git clone https://github.com/xtensor-stack/xtensor-blas.git
  cmake -DCMAKE_INSTALL_PREFIX=/usr/local ..
  make install
  ~~~~
