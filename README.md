# start and finish detection node

Start the node.
Use the ROS2 service call:

    To start: ros2 service call /start_finish_control std_srvs/srv/SetBool "{data: true}"
    To stop: ros2 service call /start_finish_control std_srvs/srv/SetBool "{data: false}"