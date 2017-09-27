#!/bin/bash

thisFile=$_
if [ $BASH ] 
then
  # may be a relative or absolute path
  thisFile=${BASH_SOURCE[0]}
fi

set_base()
{
  # use cd and pwd to get an absolute path
  configParentDir="$(cd "$(dirname "$thisFile")/../.." && pwd)"

  export CODE_BASE=$configParentDir
  export DATA_BASE=$configParentDir/../pushdata
  export PATH=$PATH:$CODE_BASE/software/build/bin
}

check_exists_and_run()
{
  if [ -f $1 ]; then
    . $1
  fi
}

setup()
{
  export PATH=$PATH:$CODE_BASE/build/bin
  export LD_LIBRARY_PATH=$CODE_BASE/build/lib:$CODE_BASE/build/lib64:$LD_LIBRARY_PATH
  export CLASSPATH=$CLASSPATH:/usr/local/share/java/lcm.jar:$CODE_BASE/build/share/java/lcmtypes_abb_lcmtypes.jar
  export CLASSPATH=$CLASSPATH:$CODE_BASE/build/share/java/drake.jar
  export PKG_CONFIG_PATH=$CODE_BASE/build/lib/pkgconfig:$PKG_CONFIG_PATH


  # python path
  export PYTHONPATH=$PYTHONPATH:$CODE_BASE/software/build/lib/python2.7/site-packages:$CODE_BASE/software/build/lib/python2.7/dist-packages
  # enable some warnings by default
  export CXXFLAGS="$CXXFLAGS -Wreturn-type -Wuninitialized"
  export CFLAGS="$CFLAGS -Wreturn-type -Wuninitialized"
}

set_ros()
{
  if [ -f $CODE_BASE/catkin_ws/devel/setup.bash ]; then
    source $CODE_BASE/catkin_ws/devel/setup.bash
  elif [ -f /opt/ros/kinetic/setup.bash ]; then
    source /opt/ros/kinetic/setup.bash
  else
    source /opt/ros/indigo/setup.bash
  fi
  export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$CODE_BASE/ros_ws/
  
  export EDITOR='geany'
}

# some useful commands
alias gitsub='git submodule update --init --recursive'
alias gitpull='git -C $CODE_BASE pull'

alias rebash='source ~/.bashrc'
alias open='gnome-open'

alias yolo='rosservice call /robot2_SetSpeed 1600 180'
alias faster='rosservice call /robot2_SetSpeed 200 50'
alias fast='rosservice call /robot2_SetSpeed 100 30'
alias slow='rosservice call /robot2_SetSpeed 50 15'

alias gohome='rosservice call robot2_SetJoints "{j1: 0, j2: 0, j3: 0, j4: 0, j5: 90, j6: 0}"'

alias teleop='rosrun teleop teleop'

alias pman='bot-procman-sheriff -l $CODE_BASE/software/config/procman.pmd'


alias getjoint='rosservice call -- robot2_GetJoints'
alias getcart='rosservice call -- robot2_GetCartesian'
alias setjoint='rosservice call -- robot2_SetJoints'
alias setcart='rosservice call -- robot2_SetCartesian'
alias setspeed='rosservice call /robot2_SetSpeed'
alias zeroft='rosservice call zero'
alias lcmlocal='sudo ifconfig lo multicast; sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo'

alias disp='$CODE_BASE/software/isamShapePose/script/displayResultFingers_pygame.py '

function disp {
   if [ $# -eq 1 ]; then
    $CODE_BASE/software/isamShapePose/script/displayResultFingers_pygame.py $DATA_BASE/result/$1 1
   fi
}

function ipmasq {
   if [ $# -eq 0 ]; then
     echo 'sharing wlan0 to eth0'
     sudo iptables -t nat -A POSTROUTING -o wlan0 -j MASQUERADE 
     sudo iptables -A FORWARD -i wlan0 -o eth0 -m state --state RELATED,ESTABLISHED -j ACCEPT 
     sudo iptables -A FORWARD -i eth0 -o wlan0 -j ACCEPT
   elif [ $# -eq 1 ]; then
     echo "sharing $1 to eth0"
     sudo iptables -t nat -A POSTROUTING -o $1 -j MASQUERADE
     sudo iptables -A FORWARD -i $1 -o eth0 -m state --state RELATED,ESTABLISHED -j ACCEPT
     sudo iptables -A FORWARD -i eth0 -o $1 -j ACCEPT
   elif [ $# -eq 2 ]; then
     echo "sharing $1 to $2"
     sudo iptables -t nat -A POSTROUTING -o $1 -j MASQUERADE
     sudo iptables -A FORWARD -i $1 -o $2 -m state --state RELATED,ESTABLISHED -j ACCEPT
     sudo iptables -A FORWARD -i $2 -o $1 -j ACCEPT
   fi
}

function set_bash {
   PROMPT_COMMAND='history -a'
   history -a

   # sorting in old style
   LC_COLLATE="C"
   export LC_COLLATE
   
   ulimit -c unlimited
   export HISTTIMEFORMAT="%d/%m/%y %T "
}

set_base
setup
set_ros
set_bash

# aliases
alias cdpush='cd $CODE_BASE'
alias c='cd $CODE_BASE/software/isamShapePose/; clion'
alias catmake='cd $CODE_BASE/catkin_ws; catkin_make; cd -'
exec "$@"
