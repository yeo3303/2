可以选择拉这个仓库里的或者直接方案2试试我当时的步骤
```
2
├── DroneCompetition/   (ROS2 工作空间)
└── ardupilot_gazebo/   (模型和插件源码)
```
方案一：
直接拉仓库，拉完后可以运行下面脚本安装依赖编译和环境变量。
```
#!/bin/bash
set -e
sudo apt update
sudo apt install -y cmake build-essential libgz-sim8-dev
sudo apt install -y ros-jazzy-mavros ros-jazzy-mavros-extras ros-jazzy-geometry-msgs
cd ~/ardupilot_gazebo
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
sudo make install
grep -qxF 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH' ~/.bashrc || echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH' >> ~/.bashrc
grep -qxF 'export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$GZ_SIM_RESOURCE_PATH' ~/.bashrc || echo 'export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$GZ_SIM_RESOURCE_PATH' >> ~/.bashrc
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$GZ_SIM_RESOURCE_PATH
cd ~/DroneCompetition
colcon build --symlink-install
echo "Setup completed. Please restart terminal or run 'source ~/.bashrc'"
```
方案2：
我的解决如下：
（删除目前残留,可以自行备份）
```
cd ~
rm -rf ardupilot_gazebo
```

（克隆官方的仓库（默认mian分支））
```
git clone https://github.com/ArduPilot/ardupilot_gazebo.git
cd ardupilot_gazebo
```
（检查分支（可选））
```
git branch
```
（可以看到* mian）
（补充了这一个通信插件，链接gazebo和ardupilot的）
```
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
sudo make install
```
./run.sh（启动三个，下面是我的文件内容）
```
source ~/.bashrc
source /opt/ros/jazzy/setup.bash
gnome-terminal --tab --title="Gazebo" -- bash -c "gz sim -v4 -r ~/ardupilot_gazebo/worlds/iris_runway.sdf; exec bash"
gnome-terminal --tab --title="ArduPilot SITL" -- bash -c "cd ~/ardupilot/ArduCopter && sim_vehicle.py -v ArduCopter --model JSON --map; exec bash"
sleep 5
gnome-terminal --tab --title="mavros" -- bash -c "ros2 launch mavros apm.launch fcu_url:=udp://:14550@; exec bash"
sleep 3
```
启动的时候ardu的窗口里面反复出现下面这段（一定！），并且可以在地图上看到家标志(等他冒冒完吧，我后来试了几次好像早一点都会失败TwT)如果出现了一些说什么in（错误的前缀）的话可以重新再来一次）。
```
no link
link 1 down
link 1 OK
heartbeat OK
Got COMMAND_ACK: REQUEST_AUTOPILOT_CAPABILITIES: ACCEPTED
AP: ArduCopter V4.8.0-dev (416146b4)
AP: 6037593ffd9d4f26b371ed750a9604da
AP: Frame: QUAD/PLUS
AP: ArduCopter V4.8.0-dev (416146b4)
AP: 6037593ffd9d4f26b371ed750a9604da
AP: Frame: QUAD/PLUS
Got COMMAND_ACK: REQUEST_AUTOPILOT_CAPABILITIES: ACCEPTED
AP: ArduCopter V4.8.0-dev (416146b4)
AP: 6037593ffd9d4f26b371ed750a9604da
AP: Frame: QUAD/PLUS
Mission is stale
Got COMMAND_ACK: REQUEST_AUTOPILOT_CAPABILITIES: ACCEPTED
Flight battery 100 percent
AP: ArduCopter V4.8.0-dev (416146b4)
AP: 6037593ffd9d4f26b371ed750a9604da
AP: Frame: QUAD/PLUS
Got COMMAND_ACK: REQUEST_AUTOPILOT_CAPABILITIES: ACCEPTED
AP: EKF3 IMU1 origin set
AP: EKF3 IMU0 origin set
AP: ArduCopter V4.8.0-dev (416146b4)
AP: 6037593ffd9d4f26b371ed750a9604da
AP: Frame: QUAD/PLUS
Got COMMAND_ACK: REQUEST_AUTOPILOT_CAPABILITIES: ACCEPTED
AP: ArduCopter V4.8.0-dev (416146b4)
AP: 6037593ffd9d4f26b371ed750a9604da
AP: Frame: QUAD/PLUS
Got COMMAND_ACK: REQUEST_AUTOPILOT_CAPABILITIES: ACCEPTED
AP: ArduCopter V4.8.0-dev (416146b4)
AP: 6037593ffd9d4f26b371ed750a9604da
AP: Frame: QUAD/PLUS
AP: ArduCopter V4.8.0-dev (416146b4)
AP: 6037593ffd9d4f26b371ed750a9604da
AP: Frame: QUAD/PLUS
Mission is stale
```
（说明其完全初始化（这里不能太早，我有次太早了飞控还没解锁就失败了））
最后
```
cd /home/yeo/DroneCompetition
source install/setup.bash
ros2 run drone_control Gps_Test_Node
```
（记得改路径和文件）
