FROM ubuntu:20.04

WORKDIR /app

# setup locale
RUN apt-get update && \
    apt-get install locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    # set default sh to bash
    ln -sf bash /bin/sh


# add repo to package source
RUN apt-get update && \
    export LANG=en_US.UTF-8 && \
    apt-get install -y software-properties-common && \
    add-apt-repository universe && \
    apt-get update && \
    apt-get install -y curl gnupg lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# install prereqs
RUN apt-get update && \
    apt-get install -y \
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
    clang-format-10 && \
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

# install ros2 galactic
RUN apt-get update && \
    apt-get install -y ros-galactic-desktop

# build moveit2
RUN apt-get update && \
    export LANG=en_US.UTF-8 && \
    . /opt/ros/galactic/setup.bash && \
    rosdep init && \
    rosdep update && \
    export COLCON_WS=/app/ws_moveit2 && \
    mkdir -p $COLCON_WS/src && \
    cd $COLCON_WS/src && \
    git clone https://github.com/ros-planning/moveit2.git -b $ROS_DISTRO && \
    for repo in moveit2/moveit2.repos $(f="moveit2/moveit2_$ROS_DISTRO.repos"; test -r $f && echo $f); do vcs import < "$repo"; done && \
    rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y && \
    cd $COLCON_WS && \
    colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release

# build moveit2_tutorials
RUN apt-get update && \
    export LANG=en_US.UTF-8 && \
    export COLCON_WS=/app/ws_moveit2 && \
    . $COLCON_WS/install/setup.bash && \
    apt-get install -y python3-colcon-mixin && \
    colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update default && \
    cd $COLCON_WS/src && \
    git clone https://github.com/ros-planning/moveit2_tutorials.git -b galactic && \
    # echo "Running vcs import..." && \
    vcs import < moveit2_tutorials/moveit2_tutorials.repos || true && \
    # echo "Installing dependencies with rosdep install!" && \
    rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y && \
    cd $COLCON_WS && \
    colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release
    # colcon build --mixin release
