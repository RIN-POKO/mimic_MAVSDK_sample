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

#define FLIGHT_MODE_MANUAL 65536
#define FLIGHT_MODE_STABILIZED 458752
#define FLIGHT_MODE_ACRO 327680
#define FLIGHT_MODE_ALTITUDE 131072
#define FLIGHT_MODE_OFFBOARD 393216
#define FLIGHT_MODE_POITION 196608
#define FLIGHT_MODE_HOLD 50593792
#define FLIGHT_MODE_MISSION 67371008
#define FLIGHT_MODE_RETURN 84148224
#define FLIGHT_MODE_FOLLOW_ME 134479872
#define FLIGHT_MODE_PRECISION_LAND 151257088
#define	FLIGHT_MODE_TAKE_OFF 33816576
#define FLIGHT_MODE_LAND 100925440

#define ARM_STATE_ARM 189
#define ARM_STATE_DISARM 61

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "mavlink_control.h"


// ------------------------------------------------------------------------------
//   TOP
// ------------------------------------------------------------------------------
int
top (int argc, char **argv)
{

	// --------------------------------------------------------------------------
	//   PARSE THE COMMANDS
	// --------------------------------------------------------------------------

	// Default input arguments
#ifdef __APPLE__
	char *uart_name = (char*)"/dev/tty.usbmodem1";
#else
	char *uart_name = (char*)"/dev/ttyUSB0";
#endif
	int baudrate = 57600;

	bool use_udp = false;
	char *udp_ip = (char*)"127.0.0.1";
	int udp_port = 14540;
	bool autotakeoff = false;

	// do the parse, will throw an int if it fails
	parse_commandline(argc, argv, uart_name, baudrate, use_udp, udp_ip, udp_port, autotakeoff);


	// --------------------------------------------------------------------------
	//   PORT and THREAD STARTUP
	// --------------------------------------------------------------------------

	/*
	 * Instantiate a generic port object
	 *
     * このオブジェクトは、オートパイロットと通信するオフボードコンピュータのポートの開閉を処理します。 
	 * mavlink_message_t オブジェクトを読み書きするメソッドを持っています。 
	 * pthreading のコンテキストでの読み書きを助けるために、 pthread のミューテックスロックでポート操作をガードします。
	 * シリアルポートでも UDP ポートでもかまいません。
	 */
	Generic_Port *port;
	if(use_udp)
	{
		port = new UDP_Port(udp_ip, udp_port);
	}
	else
	{
		port = new Serial_Port(uart_name, baudrate);
	}


	/*
	 * Instantiate an autopilot interface object
	 *
	 * これは MAVlink 上での読み込みと書き込みのための2つのスレッドを起動します。
	 * 読み込みスレッドは MAVlink メッセージをリッスンし、 current_messages アトリビュートにプッシュします。 
	 * 書き込みスレッドは今のところ、ローカル NED フレームの位置ターゲット（mavlink_set_position_target_local_ned_t）を
	 * ストリームするだけで、これは update_setpoint() メソッドを使って変更します。 
	 * enable_offboard_control() メソッドを使用して、"offboard_control "モードに入るための信号を送信します。 
	 * このモードの終了は disable_offboard_control() で知らせる。 そうしないと、車両はフェイルセーフに移行してしまう。
	 *
	 */
	Autopilot_Interface autopilot_interface(port);

	/*
	 * Setup interrupt signal handler
	 *
	 * Ctrl-Cで合図される早期終了に応答する。 
	 * ハンドラは、必要であればオフボード・モードを終了し、スレッドとポートを閉じるように命令する。  
	 * この例のハンドラは、上記のオブジェクトへの参照を必要とする。
	 *
	 */
	port_quit         = port;
	autopilot_interface_quit = &autopilot_interface;
	signal(SIGINT,quit_handler);

	/*
	 * Start the port and autopilot_interface
	 *
	 * ここでポートがオープンされ、読み取りスレッドと書き込みスレッドが開始される。
	 * 
	 */
	port->start();
	autopilot_interface.start();


	// --------------------------------------------------------------------------
	//   RUN COMMANDS
	// --------------------------------------------------------------------------

	/*
	 * Now we can implement the algorithm we want on top of the autopilot interface
	 */
	commands(autopilot_interface, autotakeoff);


	// --------------------------------------------------------------------------
	//   THREAD and PORT SHUTDOWN
	// --------------------------------------------------------------------------

	/*
	 * Now that we are done we can stop the threads and close the port
	 */
	autopilot_interface.stop();
	port->stop();

	delete port;

	// --------------------------------------------------------------------------
	//   DONE
	// --------------------------------------------------------------------------

	// woot!
	return 0;

}


// ------------------------------------------------------------------------------
//   COMMANDS
// ------------------------------------------------------------------------------

void
commands(Autopilot_Interface &api, bool autotakeoff)
{


	printf("SEND CALIBRATION COMMAND\n");	
	api.autopilot_calibrate();
	usleep(100); // 100us
	api.set_message_interval(MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 1000000); // 1e+06s
	usleep(100); // 100us
	api.set_message_interval(MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 1000000); // 1e+06s
	usleep(100); // 100us

	usleep(2 * 1000 * 1000); // 50s
	// --------------------------------------------------------------------------
	//   START OFFBOARD MODE
	// --------------------------------------------------------------------------

	// api.enable_offboard_control();
	// usleep(100); // give some time to let it sink in

	// now the autopilot is accepting setpoint commands


	if(autotakeoff)
	{
		// arm autopilot
		api.arm_disarm(true);
		usleep(100); // give some time to let it sink in (100us)
	}

	// --------------------------------------------------------------------------
	//   SEND OFFBOARD COMMANDS
	// --------------------------------------------------------------------------
	printf("SEND OFFBOARD COMMANDS\n");

	// // SEND THE COMMAND

	// takeoff
	float flight_hight  = 10; //[m]e
	float ground_level = api.current_messages.highres_imu.pressure_alt;
	float target_altitude = flight_hight + ground_level;
	printf("flight_hight = %f [m]\n",flight_hight);
	api.takeoff(NAN, NAN,NAN,NAN, target_altitude);
	usleep(100 * 1000); // 100ms
	// usleep(20 * 1000 * 1000); // 100ms
	while (true)
	{
		printf("flight hight: %f [m]\n", api.current_messages.highres_imu.pressure_alt - ground_level);
		if(FLIGHT_MODE_TAKE_OFF == api.current_messages.heartbeat.custom_mode){
			printf("flight_mode: Takeoff\n");
			break;
		}
		usleep(500 * 1000); // 500ms	
	}
	while (true)
	{
		printf("flight hight: %f [m]\n", api.current_messages.highres_imu.pressure_alt - ground_level);
		if(FLIGHT_MODE_HOLD == api.current_messages.heartbeat.custom_mode){
			printf("flight_mode: Hold\n");
			break;
		}
		usleep(500 * 1000); // 500ms	
	}
	

	// land;
	//着陸後自動的にディスアームする
	api.land(PRECISION_LAND_MODE_DISABLED, NAN, NAN, NAN, NAN);
	usleep(100 * 1000); // 100ms
	while (true)
	{
		printf("flight hight: %f [m]\n", api.current_messages.highres_imu.pressure_alt - ground_level);
		if(FLIGHT_MODE_LAND == api.current_messages.heartbeat.custom_mode){
			printf("flight_mode: Land\n");
			break;
		}
		usleep(500 * 1000); // 500ms	
	}
	while (true)
	{
		printf("flight hight: %f [m]\n", api.current_messages.highres_imu.pressure_alt - ground_level);
		if(ARM_STATE_DISARM == api.current_messages.heartbeat.base_mode){
			printf("Disarm\n");
			break;
		}
		usleep(500 * 1000); // 500ms	
	}
	// // disarm autopilot
	// api.arm_disarm(false);
	usleep(100); // give some time to let it sink in

	// --------------------------------------------------------------------------
	//   STOP OFFBOARD MODE
	// --------------------------------------------------------------------------

	// api.disable_offboard_control();

	// now pixhawk isn't listening to setpoint commands


	// --------------------------------------------------------------------------
	//   GET A MESSAGE
	// --------------------------------------------------------------------------
	printf("READ SOME MESSAGES \n");

	// copy current messages
	Mavlink_Messages messages = api.current_messages;

	// local position in ned frame
	mavlink_local_position_ned_t pos = messages.local_position_ned;
	printf("Got message LOCAL_POSITION_NED (spec: https://mavlink.io/en/messages/common.html#LOCAL_POSITION_NED)\n");
	printf("    pos  (NED):  %f %f %f (m)\n", pos.x, pos.y, pos.z );

	// hires imu
	mavlink_highres_imu_t imu = messages.highres_imu;
	printf("Got message HIGHRES_IMU (spec: https://mavlink.io/en/messages/common.html#HIGHRES_IMU)\n");
	printf("    ap time:     %lu \n", imu.time_usec);
	printf("    acc  (NED):  % f % f % f (m/s^2)\n", imu.xacc , imu.yacc , imu.zacc );
	printf("    gyro (NED):  % f % f % f (rad/s)\n", imu.xgyro, imu.ygyro, imu.zgyro);
	printf("    mag  (NED):  % f % f % f (Ga)\n"   , imu.xmag , imu.ymag , imu.zmag );
	printf("    baro:        %f (mBar) \n"  , imu.abs_pressure);
	printf("    altitude:    %f (m) \n"     , imu.pressure_alt);
	printf("    temperature: %f C \n"       , imu.temperature );

	printf("\n");


	// --------------------------------------------------------------------------
	//   END OF COMMANDS
	// --------------------------------------------------------------------------

	return;

}


// ------------------------------------------------------------------------------
//   Parse Command Line
// ------------------------------------------------------------------------------
// throws EXIT_FAILURE if could not open the port
void
parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate,
		bool &use_udp, char *&udp_ip, int &udp_port, bool &autotakeoff)
{

	// string for command line usage
	const char *commandline_usage = "usage: mavlink_control [-d <devicename> -b <baudrate>] [-u <udp_ip> -p <udp_port>] [-a ]";

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
				i++;
				uart_name = argv[i];
			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

		// Baud rate
		if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
			if (argc > i + 1) {
				i++;
				baudrate = atoi(argv[i]);
			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

		// UDP ip
		if (strcmp(argv[i], "-u") == 0 || strcmp(argv[i], "--udp_ip") == 0) {
			if (argc > i + 1) {
				i++;
				udp_ip = argv[i];
				use_udp = true;
			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

		// UDP port
		if (strcmp(argv[i], "-p") == 0 || strcmp(argv[i], "--port") == 0) {
			if (argc > i + 1) {
				i++;
				udp_port = atoi(argv[i]);
			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

		// Autotakeoff
		if (strcmp(argv[i], "-a") == 0 || strcmp(argv[i], "--autotakeoff") == 0) {
			autotakeoff = true;
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
void
quit_handler( int sig )
{
	printf("\n");
	printf("TERMINATING AT USER REQUEST\n");
	printf("\n");

	// autopilot interface
	try {
		autopilot_interface_quit->handle_quit(sig);
	}
	catch (int error){}

	// port
	try {
		port_quit->stop();
	}
	catch (int error){}

	// end program here
	exit(0);

}


// ------------------------------------------------------------------------------
//   Main
// ------------------------------------------------------------------------------
int
main(int argc, char **argv)
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


