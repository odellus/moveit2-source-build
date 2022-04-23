# Install MoveIt2!
I wasn't satisfied with MoveIt2!'s build instructions, so I made my own that include installing the ROS2 galactic distro because how you install that is **really really important**.  



1. Install ROS2 galactic with [this method](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html), which we've slightly modified for building MoveIt2! from source
    ```bash
    sudo apt-get install -y locales
    locale-gen en_US en_US.UTF-8
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    # set default sh to bash
    sudo ln -sf bash /bin/sh
    export LANG=en_US.UTF-8 
    sudo apt-get install -y software-properties-common 
    sudo add-apt-repository universe
    sudo apt-get update
    sudo apt-get install -y curl gnupg lsb-release
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    ```
2.  Run the following to install dependencies for building MoveIt2!
    ```bash
    sudo apt-get update
    sudo apt-get install -y \
    build-essential \
    cmake \
    git \
    libbullet-dev \
    python3-colcon-common-extensions \
    python3-flake8 \
    python3-pip \
    python3-pytest-cov \
    python3-rosdep \
    python3-setuptools \
    python3-vcstool \
    wget \
    clang-format-10
    # install some pip packages needed for testing
    python3 -m pip install -U \
    argcomplete \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    pytest-repeat \
    pytest-rerunfailures \
    pytest
    ```
3. Install `ros-galactic-desktop`
    ```bash
    sudo apt-get update
    sudo apt-get install -y ros-galactic-desktop
    ```
4. Build MoveIt2!
    ```bash
    # THIS LINE IS REALLY IMPORTANT! Don't forget this line!
    . /opt/ros/galactic/setup.bash
    sudo rosdep init
    rosdep update
    export COLCON_WS=~/ws_moveit2
    mkdir -p $COLCON_WS/src
    cd $COLCON_WS/src
    git clone https://github.com/ros-planning/moveit2.git -b main
    for repo in moveit2/moveit2.repos $(f="moveit2/moveit2_$ROS_DISTRO.repos"; test -r $f && echo $f); do vcs import < "$repo"; done
    rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
    cd $COLCON_WS
    colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```

# Uninstall
If anything goes wrong, the following should completely remove ROS2 from your system if you follow our installation instructions.
```bash
# Delete workspace
rm -rf $COLCON_WS
# Remove rosdep's installed source file
sudo rm /etc/ros/rosdep/sources.list.d/20-default.list
# Remove all ros packages, including colcon and rosdep
sudo purge ros-* python3-colcon-common-extensions python3-rosdep
```

# Punt
If nothing fucking works, please feel free to use the Dockerfile to ameliorate your skepticism that the build process does indeed work. I did.
```bash
cd /path/to/moveit2-source-build
docker build -t moveit2-source-build:v0.0.1 -f Dockerfile .
```
