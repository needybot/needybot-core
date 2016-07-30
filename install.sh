#!/usr/bin/env bash

BUILD_DIR="$1"
DEVEL_DIR="$1"
PACKAGE_DIR=$(dirname "$0")

# if needybot package bashrc is not sourced in .bashrc, source dat
# custom bash config can still also be put into ~/.bashrc.local

if ! grep "export NEEDYBOT_PACKAGE" ~/.bashrc; then
  echo "export NEEDYBOT_PACKAGE=$PACKAGE_DIR" >> ~/.bashrc
else
  sed 's@NEEDYBOT_PACKAGE=[^;]*@NEEDYBOT_PACKAGE='"$PACKAGE_DIR"'@' -i ~/.bashrc
fi

if [ $BUILD_DIR != "" ]; then
  if ! grep "export NEEDYBOT_BUILD_DIR" ~/.bashrc; then
    echo "export NEEDYBOT_BUILD_DIR=$1" >> ~/.bashrc
  else
    sed 's@NEEDYBOT_BUILD_DIR=[^;]*@NEEDYBOT_BUILD_DIR='"$BUILD_DIR"'@' -i ~/.bashrc
  fi
fi

if [ $DEVEL_DIR != "" ]; then
  if ! grep "export NEEDYBOT_DEVEL_DIR" ~/.bashrc; then
    echo "export NEEDYBOT_DEVEL_DIR=$1" >> ~/.bashrc
  else
    sed 's@NEEDYBOT_DEVEL_DIR=[^;]*@NEEDYBOT_DEVEL_DIR='"$DEVEL_DIR"'@' -i ~/.bashrc
  fi
fi

if ! grep -Fxq "source $PACKAGE_DIR/bashrc" ~/.bashrc; then
  echo "source $PACKAGE_DIR/bashrc" >> ~/.bashrc
fi

# List apt-get dependencies here
APT_GET_DEPS=(
  apt-transport-https
  build-essential
  ca-certificates
  git
  libimobiledevice-utils
  ntp
  python-pip
  python-tk
  ros-indigo-arbotix-*
  ros-indigo-kobuki-ftdi
  ros-indigo-rocon-qt-library
  ros-indigo-rocon-remocon
  ros-indigo-rosbridge-server
  ros-indigo-turtlebot
  ros-indigo-turtlebot-apps
  ros-indigo-turtlebot-interactions
  ros-indigo-turtlebot-simulator
  screen
  wget

)

# Loop each dependency checking if already satisfied
for i in "${APT_GET_DEPS[@]}"
do
  if [ $(dpkg-query -W -f='${Status}' $i 2>/dev/null | grep -c "ok installed") -eq 0 ];
  then 
    echo "Installing apt-get dependency $i"
    sudo apt-get install -y $i
  fi
done

# Create .pydistutils.cfg file and add settings
if [ ! -f ~/.pydistutils.cfg ];
then
  printf "[install]\nuser=1\n" >> ~/.pydistutils.cfg
fi

# Install pip dependencies from requirements.txt file
echo "Installing pip dependencies in requirements.txt"
pip install -r ${PACKAGE_DIR}/requirements.txt
