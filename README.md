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
- AI camera를 연결(로보메이션 홈페이지 참고)
- 스트리밍 화명의 URL주소를 복사한다. 
- 프로젝트/cam/src/camera.h 파일의 53번째 줄의 따옴표 안에 복사한 URL을 넣어준다. 
추후 외부에서 입력가능하도록 수정 예정
```
cap_img.open("http://192.168.66.1:9527/videostream.cgi?loginuse=admin&loginpas=admin");	
```
- 이후 다시 빌드
- 터미널에서
```
$ cd ~/Beagle_ROS
:~/Beagle_ROS$ catkin_make
```

# Start Beagle package
- 터미널1 - GUI서버 실행
```
roslaunch rosbridge_server rosbridge_websocket.launch
```
- 터미널2 - GUI 프로그램 실행
```
$ cd ~/Beagle_ROS/src/GUI/
:~/Beagle_ROS/src/GUI$ npm start
```
GUI가 뜨면 connecting Beagle 버튼을 눌러 Connected 글씨가 보이는지 확인

- 터미널3 - 비글 프로그램 실행
```
$ sudo chmod 777 /dev/ttyUSB0 //블루투스 동글 읽기쓰기 접근 권한 부여
$ cd ~/Beagle_ROS/devel/lib/beagle/
:~/Beagle_ROS/devel/lib/beagle$ ./beagle
```
GUI의 start_lidar버튼을 클릭한다. (두세번)
GUI의 up, down버튼을 눌러 선속도 양을 결정 후 publish 버튼을 누르면 적용
GUI의 left, right 버튼을 눌러 각속도 양(dgree)을 결정 후 publish 버튼을 누르면 적용

- 터미널4 - 카메라 열기
```
$ cd ~/Beagle_ROS/devel/lib/camera/
:~/Beagle_ROS/devel/lib/camera$ ./camera
```

- 터미널5 - rviz툴에서 ros 데이터들을 확인하기
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

- 터미널1 - USB 번호를 확인 (0번으로 잡은경우)

```
$ls /dev/ttyUSB*
/dev/ttyUSB0
```
- Beagle_ROS/src/nanolidar.launch/nanolidar.launch 파일의 port에 ttyUSB를 수정해준다. 
```
$ sudo chmod 777 /dev/ttyUSB0 
$ source Beagle_ROS/devel/setup.bash 
$ roslaunch nanolidar nanolidar.launch
```



- 터미널2 - 나노라이다 시작
```
$ rostopic pub /nanolidarCMD std_msgs/String "data: 'start'"
```
- 터미널2 - 나노라이다 정지
```
$ rostopic pub /nanolidarCMD std_msgs/String "data: 'stop'"
```
- 터미널3 - rviz툴에서 lidar관련 ros 데이터들을 확인하기
```
$ rviz
```

```
fixed frame: laser
Laserscan
```