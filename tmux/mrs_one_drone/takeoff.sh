echo "arming"

ros2 service call /uav1/hw_api/arming std_srvs/srv/SetBool '{"data": true}' 

echo "toggling output"

ros2 service call /uav1/control_manager/toggle_output std_srvs/srv/SetBool '{"data": true}' 

echo "toggling offboard"

ros2 service call /uav1/hw_api/offboard std_srvs/srv/Trigger '{}' 

echo "switching tracker"

ros2 service call /uav1/control_manager/switch_tracker mrs_msgs/srv/String '{"value": "LandoffTracker"}' 

echo "taking off"

ros2 service call /uav1/control_manager/landoff_tracker/takeoff mrs_msgs/srv/Vec1 '{"goal": "1.5"}' 

sleep 5.0

echo "switching controller"

ros2 service call /uav1/control_manager/switch_controller mrs_msgs/srv/String '{"value": "MpcController"}' 

echo "switching tracker"

ros2 service call /uav1/control_manager/switch_tracker mrs_msgs/srv/String '{"value": "MpcTracker"}' 

echo "goto"

./goto.py
