all: mavlink_control rc_control

mavlink_control: mavlink_control.cpp serial_port.cpp autopilot_interface.cpp
	g++ -I mavlink/include/mavlink/v1.0 mavlink_control.cpp serial_port.cpp autopilot_interface.cpp -o mavlink_control -lpthread

rc_control: serial_port.cpp rc_control.cpp 
	g++ -I mavlink/include/mavlink/v1.0 rc_control.cpp serial_port.cpp -o rc_control -lpthread

clean:
	 rm -rf *o mavlink_control rc_control
