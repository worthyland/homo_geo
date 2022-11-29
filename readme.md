# 定义环境变量
source devel/setup.sh

函数命名采用大驼峰法

普通变量命名采用小驼峰法

与数学相关的变量命名为：
ref_posDesire_base（机体坐标系在参考坐标系的期望位置  "_"将之分成三部分：1.参考的坐标系 2.变量名称含义 3.当前的坐标系）





修改mavros中IMU的发布频率为250hz

rosrun mavros mavcmd long 511 31 4000 0 0 0 0 0

511 为修改指令

31 为imu的编码 , 对应的话题名称为mavros/imu/data (103 对应的话题名称为mavros/imu/data_raw)

4000 为4000us，对应为250hz

后面五个零固定写法

rosrun mavros mavcmd long 511 32 16666 0 0 0 0 0 
//（mavros/local_position/pose）60hz


rosbag record /record /mavros/local_position/pose /image_draw

# 安装mavros

sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras

cd /opt/ros/noetic/lib/mavros //or (wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh)

sudo ./install_geographiclib_datasets.sh

# 常用软件
VScode：sudo dpkg -i ***.deb

Terminator：sudo apt install terminator

Plotjuggler：

sudo apt install ros-noetic-plotjuggler

sudo apt install ros-noetic-plotjuggler-ros

rosrun plotjuggler plotjuggler

Net-tools：

sudo apt install net-tools

ifconfig

ssh：

sudo apt install openssh-server

ssh 192.168.**.**(ssh 远程电脑ipv4地址（同一局域网）)

远程桌面

推荐 nomachine
