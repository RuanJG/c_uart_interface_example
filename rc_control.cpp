/****************************************************************************
 *
 *   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
 *   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mavlink_control.cpp
 *
 * @brief An example offboard control process via mavlink
 *
 * This process connects an external MAVLink UART device to send an receive data
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */



// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "rc_control.h"

struct Time_Stamps
{
	Time_Stamps()
	{
		reset_timestamps();
	}

	uint64_t heartbeat;
	uint64_t sys_status;
	uint64_t battery_status;
	uint64_t radio_status;
	uint64_t local_position_ned;
	uint64_t global_position_int;
	uint64_t position_target_local_ned;
	uint64_t position_target_global_int;
	uint64_t highres_imu;
	uint64_t attitude;

	void
	reset_timestamps()
	{
		heartbeat = 0;
		sys_status = 0;
		battery_status = 0;
		radio_status = 0;
		local_position_ned = 0;
		global_position_int = 0;
		position_target_local_ned = 0;
		position_target_global_int = 0;
		highres_imu = 0;
		attitude = 0;
	}

};


// Struct containing information on the MAV we are currently connected to

struct Mavlink_Messages {

	int sysid;
	int compid;

	// Heartbeat
	mavlink_heartbeat_t heartbeat;

	// System Status
	mavlink_sys_status_t sys_status;

	// Battery Status
	mavlink_battery_status_t battery_status;

	// Radio Status
	mavlink_radio_status_t radio_status;

	// Local Position
	mavlink_local_position_ned_t local_position_ned;

	// Global Position
	mavlink_global_position_int_t global_position_int;

	// Local Position Target
	mavlink_position_target_local_ned_t position_target_local_ned;

	// Global Position Target
	mavlink_position_target_global_int_t position_target_global_int;

	// HiRes IMU
	mavlink_highres_imu_t highres_imu;

	// Attitude
	mavlink_attitude_t attitude;

	// System Parameters?


	// Time Stamps
	Time_Stamps time_stamps;

	void
	init()
	{
		time_stamps.reset_timestamps();
	}

};

struct control_status {
	Serial_Port *serial;
	int system_id;
	int autopilot_id;
	int companion_id;
	bool time_to_exit;
	struct Mavlink_Messages message_buf;

	void init()
	{
		serial = NULL;	
		system_id = 0;
		autopilot_id = 0;
		companion_id = 0;
		time_to_exit = false;
		message_buf.init();
		printf("ruan: init control_data \n");
	}
}control_data;

uint64_t
get_time_usec()
{
	static struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
	return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
}

int send_heartbeat_messages(control_status *cs)
{
	int ret = 0;
	mavlink_heartbeat_t sp;
	mavlink_message_t message;

// fill with the sp 


	mavlink_msg_heartbeat_encode(cs->system_id,cs->companion_id,&message, &sp);
	int len = cs->serial->write_message(message);
	if ( not len > 0 ){
		ret = -1;
		printf("Error: could not send heartbeat message\n");
	}
	return ret;
}

void read_messages(control_status *cs)
{
	bool success;               // receive success flag
	Time_Stamps this_timestamps;
	Mavlink_Messages *current_messages = &cs->message_buf;

	// Blocking wait for new data
	while ( not cs->time_to_exit )
	{
		// ----------------------------------------------------------------------
		//   READ MESSAGE
		// ----------------------------------------------------------------------
		mavlink_message_t message;
		success = cs->serial->read_message(message);

		// ----------------------------------------------------------------------
		//   HANDLE MESSAGE
		// ----------------------------------------------------------------------
		if( success )
		{

			// Store message sysid and compid.
			// Note this doesn't handle multiple message sources.
			current_messages->sysid  = message.sysid;
			current_messages->compid = message.compid;



			// Handle Message ID
			switch (message.msgid)
			{

				case MAVLINK_MSG_ID_HEARTBEAT:
				{
					//printf("MAVLINK_MSG_ID_HEARTBEAT\n");
					mavlink_msg_heartbeat_decode(&message, &(current_messages->heartbeat));
					current_messages->time_stamps.heartbeat = get_time_usec();
					this_timestamps.heartbeat = current_messages->time_stamps.heartbeat;
					break;
				}

				case MAVLINK_MSG_ID_SYS_STATUS:
				{
					//printf("MAVLINK_MSG_ID_SYS_STATUS\n");
					mavlink_msg_sys_status_decode(&message, &(current_messages->sys_status));
					current_messages->time_stamps.sys_status = get_time_usec();
					this_timestamps.sys_status = current_messages->time_stamps.sys_status;
					break;
				}

				case MAVLINK_MSG_ID_BATTERY_STATUS:
				{
					//printf("MAVLINK_MSG_ID_BATTERY_STATUS\n");
					mavlink_msg_battery_status_decode(&message, &(current_messages->battery_status));
					current_messages->time_stamps.battery_status = get_time_usec();
					this_timestamps.battery_status = current_messages->time_stamps.battery_status;
					break;
				}

				case MAVLINK_MSG_ID_RADIO_STATUS:
				{
					//printf("MAVLINK_MSG_ID_RADIO_STATUS\n");
					mavlink_msg_radio_status_decode(&message, &(current_messages->radio_status));
					current_messages->time_stamps.radio_status = get_time_usec();
					this_timestamps.radio_status = current_messages->time_stamps.radio_status;
					break;
				}

				case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
				{
					//printf("MAVLINK_MSG_ID_LOCAL_POSITION_NED\n");
					mavlink_msg_local_position_ned_decode(&message, &(current_messages->local_position_ned));
					current_messages->time_stamps.local_position_ned = get_time_usec();
					this_timestamps.local_position_ned = current_messages->time_stamps.local_position_ned;
					break;
				}

				case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
				{
					//printf("MAVLINK_MSG_ID_GLOBAL_POSITION_INT\n");
					mavlink_msg_global_position_int_decode(&message, &(current_messages->global_position_int));
					current_messages->time_stamps.global_position_int = get_time_usec();
					this_timestamps.global_position_int = current_messages->time_stamps.global_position_int;
					break;
				}

				case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
				{
					//printf("MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED\n");
					mavlink_msg_position_target_local_ned_decode(&message, &(current_messages->position_target_local_ned));
					current_messages->time_stamps.position_target_local_ned = get_time_usec();
					this_timestamps.position_target_local_ned = current_messages->time_stamps.position_target_local_ned;
					break;
				}

				case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
				{
					//printf("MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT\n");
					mavlink_msg_position_target_global_int_decode(&message, &(current_messages->position_target_global_int));
					current_messages->time_stamps.position_target_global_int = get_time_usec();
					this_timestamps.position_target_global_int = current_messages->time_stamps.position_target_global_int;
					break;
				}

				case MAVLINK_MSG_ID_HIGHRES_IMU:
				{
					//printf("MAVLINK_MSG_ID_HIGHRES_IMU\n");
					mavlink_msg_highres_imu_decode(&message, &(current_messages->highres_imu));
					current_messages->time_stamps.highres_imu = get_time_usec();
					this_timestamps.highres_imu = current_messages->time_stamps.highres_imu;
					break;
				}

				case MAVLINK_MSG_ID_ATTITUDE:
				{
					//printf("MAVLINK_MSG_ID_ATTITUDE\n");
					mavlink_msg_attitude_decode(&message, &(current_messages->attitude));
					current_messages->time_stamps.attitude = get_time_usec();
					this_timestamps.attitude = current_messages->time_stamps.attitude;
					break;
				}

				default:
				{
					// printf("Warning, did not handle message id %i\n",message.msgid);
					break;
				}


			} // end: switch msgid

		} // end: if read message

		// give the write thread time to use the port
		//if ( writing_status > false )
		//	usleep(100); // look for components of batches at 10kHz

	} // end: while not received all

	return;
}

void* start_read_thread(void *args)
{
	// takes an autopilot object argument
	struct control_status *cs = (struct control_status *)args;

	while ( not cs->time_to_exit )
	{
		read_messages(cs);
		usleep(100000); // Read batches at 10Hz
	}

	// done!
	return NULL;
}
// ------------------------------------------------------------------------------
//   Parse Command Line
// ------------------------------------------------------------------------------
// throws EXIT_FAILURE if could not open the port
void parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate)
{

	// string for command line usage
	const char *commandline_usage = "usage: mavlink_serial -d <devicename> -b <baudrate>";

	// Read input arguments
	for (int i = 1; i < argc; i++) { // argv[0] is "mavlink"

		// Help
		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
			printf("%s\n",commandline_usage);
			throw EXIT_FAILURE;
		}

		// UART device ID
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
			if (argc > i + 1) {
				uart_name = argv[i + 1];

			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

		// Baud rate
		if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
			if (argc > i + 1) {
				baudrate = atoi(argv[i + 1]);

			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

	}
	// end: for each input argument

	// Done!
	return;
}


// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C
void quit_handler( int sig )
{
	printf("\n");
	printf("TERMINATING AT USER REQUEST\n");
	printf("\n");
	// serial port
	try {
		if( control_data.serial != NULL )
			control_data.serial->handle_quit(sig);
	}
	catch (int error){}

	// end program here
	printf("Ruan: exit by ctrl-c\n");
	exit(0);

}

// ------------------------------------------------------------------------------
//   TOP
// ------------------------------------------------------------------------------
int top (int argc, char **argv)
{

	// --------------------------------------------------------------------------
	//   PARSE THE COMMANDS
	// --------------------------------------------------------------------------

	// Default input arguments
	char *uart_name = (char*)"/dev/ttyACM0";
	int baudrate = 115200;//57600;
	int result;
	char cmd;
	bool quit_app = false;
	pthread_t read_tid;

	// do the parse, will throw an int if it fails
	parse_commandline(argc, argv, uart_name, baudrate);

	Serial_Port serial_port(uart_name, baudrate);

	control_data.init();
	control_data.serial         = &serial_port;
	serial_port.start();

	signal(SIGINT,quit_handler);

	result = pthread_create( &read_tid, NULL, &start_read_thread, &control_data );


	printf("Into cmd mode :\n>");
	while(not quit_app){
		// first time connect init 
		if( control_data.system_id ==0  || control_data.companion_id == 0){
			control_data.system_id = control_data.message_buf.sysid?control_data.message_buf.sysid:0;
			control_data.companion_id = control_data.message_buf.compid?control_data.message_buf.compid:0;
		}	

		cmd = getchar();
		switch(cmd){
			case 'q':
				quit_app = true;
				break;
			default:
				printf("read char=0x%x\n>",cmd);
				break;

		}
	}

	printf("ruan: wait for thread exit \n");
	control_data.time_to_exit = true;
	pthread_join(read_tid,NULL);

	printf("ruan: exit by normal way \n");
	serial_port.stop();
	return 0;

}




// ------------------------------------------------------------------------------
//   Main
// ------------------------------------------------------------------------------
int main(int argc, char **argv)
{
	// This program uses throw, wrap one big try/catch here
	try
	{
		int result = top(argc,argv);
		return result;
	}

	catch ( int error )
	{
		fprintf(stderr,"mavlink_control threw exception %i \n" , error);
		return error;
	}

}


