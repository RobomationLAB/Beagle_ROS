# Beagle ROS Package
Beagle 2021 (ROBOMATION & COGAPLEX)

# install newest version of nodejs, npm
```
sudo apt-get update
sudo apt-get install nodejs
sudo apt-get install npm

sudo npm cache clean -f
sudo npm install -g n
sudo n stable
node -v
sudo npm install -g npm
npm -v
```

# install ros-kinetic-rosbridge package
```
sudo apt-get install ros-kinetic-rosbridge-suite
```
# OR install ros-noetic-rosbridge package
```
sudo apt-get install ros-noetic-rosbridge-suite
```

# install Beagle package
```
cd ~

git clone https://github.com/RobomationLAB/Beagle_ROS.git

cd Beagle_ROS
catkin_make
cd src/GUI
npm install
source ~/Beagle_ROS/devel/setup.bash
```

# Setting AI camera
- AI camera�� ����(�κ����̼� Ȩ������ ����)
- ��Ʈ���� ȭ���� URL�ּҸ� �����Ѵ�. 
- ������Ʈ/cam/src/camera.h ������ 53��° ���� ����ǥ �ȿ� ������ URL�� �־��ش�. 
���� �ܺο��� �Է°����ϵ��� ���� ����
```
cap_img.open("http://192.168.66.1:9527/videostream.cgi?loginuse=admin&loginpas=admin");	
```
- ���� �ٽ� ����
- �͹̳ο���
```
$ cd ~/Beagle_ROS
:~/Beagle_ROS$ catkin_make
```

# Start Beagle package
- �͹̳�1 - GUI���� ����
```
roslaunch rosbridge_server rosbridge_websocket.launch
```
- �͹̳�2 - GUI ���α׷� ����
```
$ cd ~/Beagle_ROS/src/GUI/
:~/Beagle_ROS/src/GUI$ npm start
```
GUI�� �߸� connecting Beagle ��ư�� ���� Connected �۾��� ���̴��� Ȯ��

- �͹̳�3 - ��� ���α׷� ����
```
$ sudo chmod 777 /dev/ttyUSB0 //������� ���� �б⾲�� ���� ���� �ο�
$ cd ~/Beagle_ROS/devel/lib/beagle/
:~/Beagle_ROS/devel/lib/beagle$ ./beagle
```
GUI�� start_lidar��ư�� Ŭ���Ѵ�. (�μ���)
GUI�� up, down��ư�� ���� ���ӵ� ���� ���� �� publish ��ư�� ������ ����
GUI�� left, right ��ư�� ���� ���ӵ� ��(dgree)�� ���� �� publish ��ư�� ������ ����

- �͹̳�4 - ī�޶� ����
```
$ cd ~/Beagle_ROS/devel/lib/camera/
:~/Beagle_ROS/devel/lib/camera$ ./camera
```

- �͹̳�5 - rviz������ ros �����͵��� Ȯ���ϱ�
```
$ rviz
```

```
fixed frame: odom
Laserscan
Odometry
Image
```


# NanoLidar
NanoLidar 2021 (ROBOMATION & COGAPLEX)

- �͹̳�1 - USB ��ȣ�� Ȯ�� (0������ �������)

```
$ls /dev/ttyUSB*
/dev/ttyUSB0
```
- Beagle_ROS/src/nanolidar.launch/nanolidar.launch ������ port�� ttyUSB�� �������ش�. 
```
$ sudo chmod 777 /dev/ttyUSB0 
$ source Beagle_ROS/devel/setup.bash 
$ roslaunch nanolidar nanolidar.launch
```



- �͹̳�2 - ������̴� ����
```
$ rostopic pub /nanolidarCMD std_msgs/String "data: 'start'"
```
- �͹̳�2 - ������̴� ����
```
$ rostopic pub /nanolidarCMD std_msgs/String "data: 'stop'"
```
- �͹̳�3 - rviz������ lidar���� ros �����͵��� Ȯ���ϱ�
```
$ rviz
```

```
fixed frame: laser
Laserscan
```