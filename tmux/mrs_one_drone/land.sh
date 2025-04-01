echo "landing"

ros2 service call /uav1/control_manager/landoff_tracker/land std_srvs/srv/Trigger '{}' 
