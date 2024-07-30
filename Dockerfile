FROM ros:humble-ros-base

# Instalar dependências
# RUN apt-get update && apt-get install -y locales \
#     && locale-gen en_US en_US.UTF-8 \
#     && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
#     && export LANG=en_US.UTF-8


# RUN apt-get install -y software-properties-common \
#     && add-apt-repository universe \
#     && apt-get update && apt-get install -y curl

# RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
#     && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
#     && apt-get update \
#     && apt-get install -y ros-humble-desktop \
#     && apt-get install -y ros-dev-tools \

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    ros-humble-desktop=0.10.0-1* \
    ros-humble-tf-transformations \
    ros-humble-compressed-image-transport \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /ros2_ws/src \
    && cd /ros2_ws \
    && colcon build --symlink-install

RUN apt-get update  && apt-get install -y git git-lfs && git lfs install

## Install cargo
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | bash -s -- -y
ENV PATH="/root/.cargo/bin:${PATH}"


RUN cd /ros2_ws/src && git clone https://github.com/pollen-robotics/reachy_2023.git

COPY --chmod=755 ros_entrypoint.sh /app/ros_entrypoint.sh

ENTRYPOINT ["/app/ros_entrypoint.sh"]

RUN apt install python3-pip -y
RUN pip3 install setuptools==58.2.0

COPY requirements.txt /ros2_ws/
RUN pip3 install -r /ros2_ws/requirements.txt

RUN cd /ros2_ws && bash ./src/reachy_2023/dependencies.sh
RUN cd /ros2_ws && pip3 install -r ./src/reachy_2023/requirements.txt

#scripts são geralmente escritos para ser executados com bash, mas o shell padrão em muitos contêineres Docker é sh
RUN . /opt/ros/humble/setup.sh && cd /ros2_ws && colcon build  

#RUN mkdir /app




# RUN mkdir /dev \    
#     && cd /dev

#RUN cd /dev \
 #   && git clone https://github.com/pollen-robotics/reachy-sdk-api.git \
  #  && git clone https://github.com/pollen-robotics/reachy-sdk.git

#RUN apt-get update

#RUN cd /dev/reachy-sdk-api \
  #  && pip3 install -e python

#RUN cd /dev/reachy-sdk \
 #   && pip3 install -e . 
