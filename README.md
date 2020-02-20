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
  -OpenCV
  -Xtensor
  -Xtensor-BLAS
  -Eigen3