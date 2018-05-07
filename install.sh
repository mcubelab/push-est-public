#!/bin/bash
# Peter KT Yu, 2015

# See also push-est-public/catkin_ws/src/isam_pose/install.sh

function ask {
    echo $1        # add this line
    read -n 1 -r
    if [[ $REPLY =~ ^[Yy]$ ]]
    then
        return 1;
    else
        exit
        echo "Abort.."
    fi
}

if [ "$#" == 0 ] || [ "$1" == "APT" ]; then
    echo "Install useful packages from apt-get"
    sudo apt-get update
    
    sudo apt-get --yes install git gitk git-gui geany geany-plugins vim terminator meshlab recordmydesktop meld sagasu openssh-server retext filezilla vlc ipython mesa-utils bmon libyaml-0-2
    sudo apt-get --yes install hardinfo cpufrequtils   # for speedup cpu
        
    sudo apt-get --yes install kcachegrind kcachegrind-converters
    sudo apt-get --yes install build-essential cmake debhelper freeglut3-dev gtk-doc-tools libboost-filesystem-dev libboost-iostreams-dev libboost-program-options-dev libboost-random-dev libboost-regex-dev libboost-signals-dev libboost-system-dev libboost-thread-dev libcurl4-openssl-dev libfreeimage-dev libglew-dev libgtkmm-2.4-dev libltdl-dev libgsl0-dev libportmidi-dev libprotobuf-dev libprotoc-dev libqt4-dev libqwt-dev libtar-dev libtbb-dev libtinyxml-dev libxml2-dev ncurses-dev pkg-config protobuf-compiler python-matplotlib libqhull-dev python-pygame doxygen mercurial libglib2.0-dev python-dev gfortran f2c libf2c2-dev spacenavd libspnav-dev python-numpy python-scipy python-vtk python-pip libgmp3-dev libblas-dev liblapack-dev libv4l-dev subversion libxmu-dev libusb-1.0-0-dev python-pymodbus graphviz curl libwww-perl libterm-readkey-perl 

    sudo apt-get --yes install libgl1-mesa-dev  # for libbot libGL.so
    
    sudo apt-get --yes install libopenal-dev # for drake converting wrl file
    sudo apt-get --yes install libgl1-mesa-dev  # for libbot libGL.so
    sudo apt-get --yes install compizconfig-settings-manager
    sudo pip install chan 
    sudo pip install openpyxl
    sudo easy_install pycollada
fi


if [ "$#" == 0 ] || [ "$1" == "ROSK" ]; then
    echo "Install ROS Kinetic"

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116
    sudo apt-get update
    sudo apt-get --yes install ros-kinetic-desktop-full
    sudo rosdep init
    rosdep update
    sudo apt-get --yes install python-rosinstall
    source /opt/ros/kinetic/setup.bash

    sudo apt-get --yes install ros-kinetic-moveit
#    sudo apt-get --yes install ros-kinetic-pcl-ros
#    sudo apt-get --yes install ros-kinetic-joy
    sudo apt-get --yes install ros-kinetic-perception  # for cv_bridge

#    sudo cp joint_state_publisher/joint_state_publisher /opt/ros/kinetic/lib/joint_state_publisher/joint_state_publisher
fi

if [ "$#" == 0 ] || [ "$1" == "REALSENSE" ]; then 
  sudo apt-get --yes install libglfw3-dev libusb-1.0-0-dev pkg-config
  sudo apt-get --yes install libssl-dev
  
  mkdir -p $HOME/software
  cd $HOME/software
  git clone git@github.com:IntelRealSense/librealsense.git --depth 1
  
  cd librealsense
  ./scripts/patch-realsense-ubuntu-xenial.sh
  
  
  sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
  sudo udevadm control --reload-rules && udevadm trigger
  
  
  mkdir build && cd build
  cmake ../ -DBUILD_EXAMPLES=true
  make && sudo make install
fi
                  
if [ "$#" == 0 ] || [ "$1" == "LCM" ]; then
    # install LCM
    wget -P ~/Downloads -q https://github.com/lcm-proj/lcm/archive/v1.3.1.tar.gz
    
    mkdir -p $HOME/software
    cp ~/Downloads/lcm-1.3.1.tar.gz $HOME/software
    rm ~/Downloads/lcm-1.3.1.tar.gz
              
    cd $HOME/software/
    tar -xzvf lcm-1.3.1.tar.gz
    cd lcm-1.3.1/
    ./bootstrap.sh
    ./configure
    make
    sudo make install
    cd ..
    rm lcm-1.3.1.tar.gz
fi
                  
                  
if [ "$#" == 0 ] || [ "$1" == "LIBBOT" ]; then
    mkdir -p $HOME/software
    cd $HOME/software
    git clone https://github.com/mit212/libbot.git
    cd libbot
    sudo make -j
    sudo cp -r ../build/* /usr/local/  # some weird situation for VM ware
    sudo cp -r build/* /usr/local/
fi

if [ "$#" == 0 ] || [ "$1" == "APRILTAGCPP" ]; then
    mkdir -p $HOME/software
    cd $HOME/software
    git clone https://github.com/mit212/apriltags-cpp.git
    cd apriltags-cpp
    mkdir build
    cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release
    make
    sudo make install
fi

