#!/bin/bash

# Platform detection
shell=`echo $SHELL | awk -F '/' '{print $NF}'`
ubuntu_version=`lsb_release -r | cut -f2`
if [[ $ubuntu_version == 18.04 ]]; then
    ros1_distro="melodic"
    ros2_distro="dashing"
    config_file=./yaml/ros_menu_18.04.yaml
elif [[ $ubuntu_version == 20.04 ]]; then
    ros1_distro="noetic"
    ros2_distro="foxy"
    config_file=./yaml/ros_menu_20.04.yaml
elif [[ $ubuntu_version == 22.04 ]]; then
    # no ros1 distro anymore
    ros1_distro="none"
    ros2_distro="humble"
    config_file=./yaml/ros_menu_22.04.yaml
else
    echo "Sorry, we don't support Ubuntu $ubuntu_version."
    exit
fi

if [ "$USE_CONTAINER" '==' "True" ] || [ "$USE_CONTAINER" '==' "TRUE" ]
then
    echo "We won't install ROS / ROS 2 in your host since you use container instead."
    echo "The container will be installed in the first time you run it."
else
    # ROS environment installation
    echo -n "Do you want to install ROS automatically? (y/N): "
    read ros_install
    if [ "$ros_install" '==' "y" ] || [ "$ros_install" '==' "Y" ];
    then
        echo -n "Do you want to install ROS Desktop? (y/N): "
        read ros_desktop_install
        if [ "$ros_desktop_install" '==' "y" ] || [ "$ros_desktop_install" '==' "Y" ];
        then
            # Install ROS 1
            ./scripts/install_${ros1_distro}.sh

            # Install ROS 2
            ./scripts/install_${ros2_distro}.sh
        else
            # Install ROS 1
            ./scripts/install_${ros1_distro}_base.sh

            # Install ROS 2
            ./scripts/install_${ros2_distro}_base.sh
        fi

        # Install ROS dependencies and related packages
        ./scripts/install_dep.sh

    else
        echo "Skip installing ROS"
    fi
fi

# Install ROS menu and config file
if [ -f ~/ros_menu/config.yaml ]; then
    echo  "ROS Startup Menu was already installed!"
    echo -n "Do you want to reinstall? (Your setting in the file config.yaml would be over written.) (y/N): "
    read over_write
else
    over_write="y"
fi
if [ ! "$over_write" '==' "y" ] && [ ! "$over_write" '==' "Y" ]; then
    echo "Skip installing ROS Startup Menu!"
    exit
fi
rm -f ~/.ros_menu
ln -s `pwd` ~/.ros_menu
if [[ -n $1 ]]; then
    config_file=./yaml/$1
fi
cp $config_file config.yaml

if ! grep -q ros_menu ~/.${shell}rc; then
    cat <<EOF >> ~/.${shell}rc
alias eb='gedit ~/.bashrc'
alias sb='source ~/.bashrc'
alias gs='git status'
alias gp='git pull'
alias cw='cd ~'
alias cs='cd ~/src'
alias cm='cd ~ && catkin_make'

alias cma='catkin_make -DCATKIN_WHITELIST_PACKAGES=""'
alias cop='catkin_make --only-pkg-with-deps'
alias copr='catkin_make -DCMAKE_BUILD_TYPE=Release --only-pkg-with-deps'
alias sds='source devel/setup.bash'
alias axclient='rosrun actionlib axclient.py'
alias killg='killall -9 gzserver && killall -9 gzclient && killall -9 rosmaster'

alias cba='colcon build --symlink-install'
alias cbr='colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'
alias cbp='colcon build --symlink-install --packages-select'

alias rosdinstall='rosdep install -y -r -q --from-paths src --ignore-src --rosdistro'

alias rosmelo="source /opt/ros/melodic/setup.bash"
alias roseloq='source /opt/ros/eloquent/setup.bash && source ./install/setup.bash && export PYTHONPATH=/opt/ros/eloquent/lib/python3.6/site-packages'
alias rosfoxy='source /opt/ros/foxy/setup.bash && source ./install/setup.bash && source ./install/local_setup.bash'
alias rosdinstall='rosdep install -y -r -q --from-paths src --ignore-src --rosdistro'

# ROS Startup Menu #
ros_bashrc_path=~/.ros_menu/ros_bashrc
if [ -f \$ros_bashrc_path ]; then
    source \$ros_bashrc_path
fi
# End of ROS Startup Menu #
EOF
fi

echo "ROS Startup Menu installed successfully"
