# Drone Rescue Project
储存大创项目所用的ROS C++代码

## 使用方法
### 下载并编译
```bash
cd ~
git clone https://github.com/zxh0916/drone_rescue.git
cd drone_rescue
catkin build
```
### 运行
#### 第一个终端：打开roscore和MAVROS
##### 安装MAVROS
根据[px4官方安装教程](https://docs.px4.io/main/en/ros/mavros_installation.html)：
```bash
sudo apt-get updat
sudo apt-get install ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras ros-${ROS_DISTRO}-mavros-msgs
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
```

```bash
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```
#### 第二个终端：启动gazebo仿真
```bash
cd PX4-Autopilot
make px4_sitl gazebo
```
#### 第三个终端：启动QGC地面站

##### 首次运行QGC：安装依赖并下载QGC
```bash
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libqt5gui5 -y
sudo apt install libfuse2 -y
cd ~
wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage
sudo chmod 777 ~/QGroundControl.AppImage
```
##### 运行QGC
```bash
~/QGroundControl.AppImage
```

#### 第四个终端：启动ROS节点
```bash
source ~/drone_rescue/devel/setup.bash
rosrun phone_finder find_the_phone
```
