---
# KIST-Dual-Arm-ROS

## ROS Version : ROS melodic

## Required libraries
### Mujoco Version : more than 2.0
### Eigen Library : more than 3.0.0
### RBDL library : more than 2.6.0
> RBDL 2.6.0 (Note: ORB ver is not working for this project.)
```sh
git clone https://github.com/rbdl/rbdl
cd rbdl
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=Release -DRBDL_BUILD_ADDON_URDFREADER=ON -DRBDL_USR_ROS_URDF_LIBRARY=OFF ..
make all
make install
sudo ldconfig
```
### QPOASES
```sh
git clone https://github.com/coin-or/qpOASES.git
cd qpOASES
mkdir build
cd build
cmake ..
make all
make install
sudo ldconfig
```
### OMPL
## First, you need to download the OMPL installation script. [Link](https://ompl.kavrakilab.org/installation.html)
> After download the installation script, Make the script executable
```
cd ~/(directory where you download the script)
sudo chmod u+x install-ompl-ubuntu.sh
```
> Next, you need to execute the script with option (--github)
```
./install-ompl-ubuntu.sh --github
```
