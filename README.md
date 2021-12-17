# Beagle
Beagle 2021 (COGAPLEX & ROBOMATION)

### install newest version of nodejs, npm
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

### install ros-kinetic-rosbridge package
```
sudo apt-get install ros-kinetic-rosbridge-suite
roslaunch rosbridge_server rosbridge_websocket.launch
```
### OR install ros-noetic-rosbridge package
```
sudo apt-get install ros-noetic-rosbridge-suite
roslaunch rosbridge_server rosbridge_websocket.launch
```

### install Beagle package
```
cd ~

git clone https://github.com/papaGeppetto/Beagle.git

cd Beagle
catkin_make
cd src/GUI
npm install
source ~/Beagle/devel/setup.bash
```
### start Beagle package
```

```


# NanoLidar
NanoLidar 2021 (ROBOMATION)
