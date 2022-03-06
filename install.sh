#!/bin/bash

# Platform detection
shell=`echo $SHELL | awk -F '/' '{print $NF}'`
if [[ $(grep 20.04 /etc/issue) ]]; then
    ros1_distro="noetic"
    ros2_distro="foxy"
    config_file=./yaml/ros_menu_20.04.yaml
else
    ros1_distro="melodic"
    ros2_distro="eloquent"
    config_file=./yaml/ros_menu_18.04_OpenSG.yaml
fi

# ROS environment installation
echo -n "Do you want to install ROS automatically? (y/N): "
read ros_install
if [ "$ros_install" '==' "y" ] || [ "$ros_install" '==' "Y" ];
then
    # Install ROS 1
    ./scripts/install_${ros1_distro}.sh

    # Install ROS 2
    ./scripts/install_${ros2_distro}.sh

    # Install ROS dependencies and related packages
    ./scripts/install_dep.sh

else
    echo "Skip installing ROS"
fi

# Install ROS menu and config file
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
alias rosfoxy='source /opt/ros/foxy/setup.bash && source ./install/setup.bash && export PYTHONPATH=/opt/ros/foxy/lib/python3.8/site-packages'
alias rosdinstall='rosdep install -y -r -q --from-paths src --ignore-src --rosdistro'
# Neuron Startup Menu #
ros_bashrc_path=~/.ros_menu/ros_bashrc
if [ -f \$ros_bashrc_path ]; then
    source \$ros_bashrc_path
fi
# End of Neuron Startup Menu #
EOF
fi

echo "Neuron Startup Menu installed successfully"
