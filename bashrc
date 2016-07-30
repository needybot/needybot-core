# exports section
export TURTLEBOT_BATTERY=/sys/class/power_supply/BAT0
export TURTLEBOT_3D_SENSOR=kinect
export PATH=$NEEDYBOT_PACKAGE/bin:$HOME/.local/bin:/usr/bin:/opt/ros/indigo/bin:$PATH

# source files
source /opt/ros/indigo/setup.bash
source $NEEDYBOT_DEVEL_DIR/setup.bash
