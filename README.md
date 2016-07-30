# Needybot 
Core Needybot ROS package.

## Quick Start - Installing ROS
###Ubuntu 14.04/12.04
*Use Ubuntu.* See [this note](https://github.com/wieden-kennedy/needybot-ros/wiki/Building-a-Needybot-Instance#note-on-os-version)
on the version of Ubuntu to use.
If you need to install ROS on your system, or are rebuilding, use
the bootstrap script found [here](https://github.com/needybot/needybot-ros-bootstrap).

This will take care of adding the correct ROS source repository to your apt-get
sources list, install all dependencies, ROS, and add the correct source files
to your shell `.rc` file.

###Installing the ROS Package
If you've already got ROS Indigo installed, you can download this package and run `catkin make` from your catking workspace.

    $ cd ~/catkin_ws
    $ git clone https://github.com/needybot/needybot-core
    $ catkin_make 

Next, make sure to re-source you .bashrc as new ENV vars are added on a fresh install.

    $ source ~/.bashrc 

###Running Your Needybot
Were working on providing a thorough set of [tutorials](https://github.com/needybot/needybot-ros-bootstrap). We're working on guides to go along with them and actively building more tutorials. For the time being feel free to jump into the tutrials we've provided, read the code, and start mucking around.

--

## Documentation and Troubleshooting
We are working on providing thorough documentation for Needybot. Please hold tight. 

If you can't find what you're looking for, or needy some help, ask a Needybot core engineer.

--

## Issues
Issues should be filed in GitHub, and should be detailed enough that
any engineer reading the issue can pick it up and work on its resolution.
There are no hard and fast rules on how to make issues explicitly detailed,
so just try to be thorough in your descriptions of the filed issue.
