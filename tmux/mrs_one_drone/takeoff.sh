echo "arming"

ros2 service call /uav1/hw_api/arming std_srvs/srv/SetBool '{"data": true}' 

echo "toggling output"

ros2 service call /uav1/control_manager/toggle_output std_srvs/srv/SetBool '{"data": true}' 

echo "toggling offboard"

ros2 service call /uav1/hw_api/offboard std_srvs/srv/Trigger '{}' 

echo "taking off"

ros2 service call /uav1/uav_manager/takeoff std_srvs/srv/Trigger '{}' 
