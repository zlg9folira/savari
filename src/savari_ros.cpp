/**
 *  Developed and maintained by:
 *  	2019-Foad Hajiaghajani
 *  	https://buffalo.edu/~foadhaji
 *  	foadhaji [at] buffalo.edu
 *
 *  Industry:
 *  	Connected and Self-driving Car Applications and Systems (Research)
 *
 *	Introduction:
 *  	This code defines a Robot-Operating-system package (ROS driver) as an interface between SAVARI
 *  	DSRC on-board-unit (OBU) and a Linux machine. This ROS node publishes all the vehicle-related
 *  	information for the connected OBU (self vehicle) and remote vehicle (ego vehicle) whose
 *  	basic-safety-messages (BSMs) are periodically received by self vehicle - The ROS driver is
 *  	developed for a pair of OBUs working simultaneously.
 *  	NOTE: This ROS pkg does not provide client OBU application.
 *		------------------------------------------------
 * 		Compiled by and verified on SAVARI SDK v5.10.1.7
 *		Compiled by and verified on C++11-ROS_Kinetic-Ubuntu_16.04
 *		------------------------------------------------
 *
 *	Required:
 *		[ROS]			Robot Operating System
 *		[ROS pkg]		sensor_msgs_NavSatFix [http://wiki.ros.org/sensor_msgs]
 *		[ROS pkg]		savari_msgs [https://github.com/zlg9folira/savari_msgs]
 *
 *	Configuration:
 *		[C++]			C++11 (verified)
 *  	[TCP client]	[Leader] 	Savari MW-1000 802.11p DSRC radio 	-IP add: 192.168.100.21/24
 *  					[Follower]	Savari MW-1000 802.11p DSRC radio 	-IP add: 192.168.100.41/24
 *  	[TCP server]	Linux Ubuntu (16.04 verified) 		-IP add: 192.168.100.150/24
 *  	[TCP port]		1*35 (verified)
 *  	[ROS Distro]	Kinetic (verified)
 *  	[ROS version]	1.12.14 (verified)
 *
 *  Build & Install:
 *  	[Shell]			catkin_make --only-pkg-with-deps savari
 *  Usage help:
 *  	[Shell]			rosrun savari savari_ros -h
 *
 *	All files are subject to further change or update..
 *	Last update: Jan 2020
 *
 *
 */

#ifndef _savari_ros /* Prevent multiple inclusion */
#define _savari_ros

#include <iostream>
#include <iomanip>
#include <fstream>
#include <stdlib.h>
#include <cstring>
#include "ros/ros.h"
#include <signal.h>
#include <stdio.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <bsm_tcp_msg_enc.h>
#include <bsm_tcp_msg_dec.h>
#include <msg_conversion_handler.h>
#include <ros_handler.h>

#define SA struct sockaddr

#define black  "\x1b[30m"
#define red "\x1b[31m"
#define green  "\x1b[32m"
#define yellow  "\x1b[33m"
#define blue  "\x1b[34m"
#define magenta  "\x1b[35m"
#define cyan  "\x1b[36m"
#define white  "\x1b[37m"
#define colend  "\x1b[0m"


class savariTCP
{
public:
	int port;
	int sock_;
	int t_out = 0;		/* timeout */
	int self_gs_;		/* SELF vehicle gps status [0] off [1] on */
	int ego_gs_;		/* EGO vehicle gps status [0] off [1] on */
	struct sockaddr_in serv_addr_;

	int set_port(int p_){
		if (p_)
			return(this->port = p_);
		else
			return(this->port = this->default_port);
	}

	void terminate(){
		close(this->sock_);
	}

	int msg_count(int c_){
		if (!this->t_out){
			if (!this->self_gs_ && !this->ego_gs_){
				fflush(stdout);
				printf("\r\t\t\tMsg count: %d\t%s[SELF GPS ERR]%s\t%s[EGO GPS ERR]%s",c_,red,colend,red,colend);
				return (c_+1);
			}else if(!this->self_gs_){
				fflush(stdout);
				printf("\r\t\t\tMsg count: %d\t%s[SELF GPS ERR]%s",c_,red,colend);
				return (c_+1);
			}else if(!this->ego_gs_){
				fflush(stdout);
				printf("\r\t\t\tMsg count: %d\t%s[EGO GPS ERR]%s",c_,red,colend);
				return (c_+1);
			}
			fflush(stdout);
			printf("\r\t\t\tMessage count: %d\t%s[GPS status OK]%s",c_,green,colend);
		}
		return (c_+1);
	}

	int do_not_exit(int s_){
		printf("%s[DO NOT EXIT]%s\t\tROS will automatically terminate if client OBU disconnects!\n",yellow,colend);
		return (s_+1);
	}

	void usage(){
		printf("%s[Usage]%s\t\trosrun savari savari_ros -p [PORT]\n",cyan,colend);
		printf("%s[Example]%s\trosrun savari savari_ros -p 1234\n",cyan,colend);
		printf("%s[Default]%s\trosrun savari savari_ros\n\n",cyan,colend);
	}

private:
	int default_port = 1235; // default TCP port
};

/**
 *
 * The SAVARI ROS driver features non-blocking socket connections and is supposed
 * to be terminated automatically. The ROS driver will trigger a countdown timer
 * once the client OBU terminates or the connection with OBU fails. We do not recommend
 * force termination on ROS terminal (driver).
 * To avoid repeated transient-socket connections or socket leaks,
 * signalHandler() would forcely shut the ROS server socket down
 * if a termination signal (ctrl+c) is used by user.
 *
 * To check the status of live TCP connections, use the following command
 * in Linux Terminal and make sure there has not been TCP connections
 * left OPEN with status CLOSE_WAIT and foreign IP address corresponding
 * with the connected SAVARI OBU:
 *
 * $ netstat -ntp
 *
 */
void signalHandler(int sig_num){
	savariTCP st_;
	signal(SIGINT,signalHandler);
	st_.terminate();
	printf("\n");
	fflush(stdout);
}

int main(int argc, char **argv)
{
	int opt;
	int msg_s = 0;
	int msg_c = 0;
	ros::init(argc, argv, "savari_ros");
	savariRosNode savariNode;
	savariTCP stcp;
	// Actual loop_rate (Hz) is contingent upon socket msg reception rate
	ros::Rate loop_rate(20);
	signal (SIGINT, signalHandler);

	/**
	 * @param [deadline] specifies the non-blocking timeout due. In case the ROS server
	 * does not receive packets over the pre-established TCP channel, it will trigger
	 * the countdown timer starting from [deadline]. Once the timer is over, the ROS server
	 * will safely close the connections and reset the cache.
	 *
	 * NOTE: The client OBU must be activated within the [deadline] time frame and before
	 * the ROS server exits the "Listen" routine.
	 */
	int deadline = 20; //sec

	if (argc<2){
		printf("\n%s[Socket]%s\t\tPort set to default: %d\n",green,colend,stcp.set_port(0));
	}else{
		while ((opt = getopt(argc, argv, "p:h")) != -1) {
			switch(opt) {
			case 'p':
				if(!atoi(optarg) || atoi(optarg) < 0){
					printf("%s[Socket]%s\t\tInvalid port selected\n",red,colend);
					return (0);
				}
				stcp.set_port((int)atoi(optarg));
				printf("%s[Socket]%s\t\tPort set to %d\n",green,colend,stcp.port);
				break;
			case 'h':
				stcp.usage();
				return (0);
			default:
				printf("%s[Input]%s\t\t\tArguments not supported!\n",yellow,colend);
				printf("%s[Socket]%s\t\tPort set to default: %d\n",green,colend,stcp.set_port(0));
			}
		}
	}
	// socket create and verification
	stcp.sock_ = socket(AF_INET, SOCK_STREAM, 0);
	int one = 1;
	setsockopt(stcp.sock_, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));
	int idletime = 10;
	setsockopt(stcp.sock_, IPPROTO_TCP, TCP_KEEPIDLE, &idletime, sizeof(idletime));

	if (stcp.sock_ == -1) {
		printf("%s[Socket]%s\t\tSocket creation failed...\n",red,colend);
		exit(0);
	}
	else
		printf("%s[Socket]%s\t\tSocket successfully created..\n",green,colend);
	bzero(&stcp.serv_addr_, sizeof(stcp.serv_addr_));

	// Forcefully attaching socket to the port
	if (setsockopt(stcp.sock_, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &one, sizeof(one)))
	{
		perror("setsockopt");
		exit(EXIT_FAILURE);
	}

	// Assign IP, PORT
	stcp.serv_addr_.sin_family = AF_INET;
	stcp.serv_addr_.sin_addr.s_addr = htonl(INADDR_ANY);
	stcp.serv_addr_.sin_port = htons(stcp.port);

	// Binding socket to the IP and verification
	if ((bind(stcp.sock_, (SA*)&stcp.serv_addr_, sizeof(stcp.serv_addr_))) != 0) {
		printf("%s[Socket]%s\t\tSocket bind failed...\n",red,colend);
		exit(0);
	}
	else
		printf("%s[Socket]%s\t\tSocket successfully binded..\n",green,colend);

	// Now server is ready to listen and verification
	if ((listen(stcp.sock_, 5)) < 0) {
		printf("%s[Socket]%s\t\tListen failed..\n",red,colend);
		exit(0);
	}
	else
		printf("%s[Socket]%s\t\tROS server now listening..\n",green,colend);

	printf("%s[DO NOT EXIT]%s\t\tWaiting for client OBU..\n",yellow,colend);

	// ROS LOOP
	while (savariNode.rosOK())
	{
		socklen_t sizeOfserv_addr = sizeof(stcp.serv_addr_);
		fd_set set;
		struct timeval timeout;
		int connfd_;
		int rv_;
		FD_ZERO(&set); /* clear the set */
		FD_SET(stcp.sock_, &set); /* add our file descriptor to the set */
		timeout.tv_sec = deadline;
		timeout.tv_usec = 0;
		rv_ = select(stcp.sock_ + 1, &set, NULL, NULL, &timeout);

		if(rv_ == -1)
		{
			perror("select");
			return 1;
		}
		else if(rv_ == 0)
		{
			printf("\n%s[Timeout]%s\t\tClient OBU is not connected or got disconnected..\n\n",red,colend); /* a timeout occured */
			stcp.t_out = 1;
			close (connfd_);
			close (stcp.sock_);
			ros::shutdown();
		}
		else{
			connfd_ = accept (stcp.sock_,(struct sockaddr *) &stcp.serv_addr_, (socklen_t*)&sizeOfserv_addr);
			if (connfd_ >= 0) {
				if (!msg_s){
					printf("%s[CONNECTED]%s\t\tROS topics are generated and advertised!\n",green,colend);
					msg_s+=1;
				}
				// Receive raw data [char] for self & ego vehicles
				savariNode.payload_enc = {0};
				int ret = read (connfd_, &savariNode.payload_enc, sizeof(savariNode.payload_enc));
				// Transform raw data to numerical format
				savariNode.payload_dec = {0};
				savariNode.payload_dec = decode_tcp_payload(savariNode.payload_enc);
				if (!savariNode.payload_dec.self.gpsstatus)
					stcp.self_gs_ = 0;
				else
					stcp.self_gs_ = 1;
				if (!savariNode.payload_dec.ego.gpsstatus)
					stcp.ego_gs_ = 0;
				else
					stcp.ego_gs_ = 1;

				// Fill out and publish ROS messages
				savariNode.fill_ros_msg();
				savariNode.publish();

				//Print logs
				//printf("%s[BSM INTERVAL]%s\t\t%lf ms\n",white,colend,savariNode.payload_dec.self.bsm_interval);
				//printf("%s[DISTANCE]%s\t\t%lf meter\n",white,colend,savariNode.payload_dec.self.distance);

				close (connfd_);
			}else{
				printf("%s[Socket]%s\t\tROS server acccept failed..\n", red,colend);
				close (connfd_);
				close (stcp.sock_);
				return 0;
			}
		}
		msg_c = stcp.msg_count(msg_c);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}


#endif /* ndef _savari_ros */

