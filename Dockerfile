FROM ros:jazzy 

SHELL ["/bin/bash", "-c"]

RUN sudo apt update && \
    sudo apt install -y git \
                        ssh \
                        vim \
                        net-tools \
                        python3-vcstool \
                        python3-rosdep \ 
                        python3-colcon-common-extensions

RUN mkdir pibot_ws/src -p 
COPY ./pibot.repos pibot_ws/src

RUN cd pibot_ws/src && \
    vcs import < pibot.repos 

RUN cd pibot_ws/ && \
    rosdep update && \
    rosdep install --from-path src --ignore-src -y --skip-keys='depthai nav2_minimal_tb3_sim nav2_minimal_tb4_sim'

RUN cd pibot_ws && \
    source /opt/ros/jazzy/setup.bash && \
    colcon build --packages-skip nav2_system_tests && \
    rm -rf src build log

RUN rm -rf /var/lib/apt/lists*
