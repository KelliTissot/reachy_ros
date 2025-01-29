FROM pollenrobotics/reachy_2023:1.1



COPY requirements.txt /rehabot/requirements.txt
# COPY requirements.txt /opt/reachy_ws/src/reachy_2023/reachy_kdl_kinematics/reachy_kdl_kinematics/rehabot/requirements.txt
RUN pip install -r /rehabot/requirements.txt

COPY setup.py /opt/reachy_ws/src/reachy_2023/reachy_kdl_kinematics/setup.py

COPY ./src /rehabot
RUN pip install -e /rehabot

RUN cd /opt/reachy_ws/ && colcon build --symlink-install
