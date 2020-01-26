/* `_bsm_tcp_msg_dec.h'
 * 	2019 - Foad Hajiaghajani - foadhaji [at] buffalo.edu
 * 	Compiled by and verified on SAVARI SDK v5.10.1.7
 *	Compiled by and verified on C++11-ROS_Kinetic-Ubuntu_16.04 
 *	This header file must be loaded in /savari_ros/include/
 */
#include <inttypes.h>
#ifndef _bsm_tcp_msg_dec /* Prevent multiple inclusion */
#define _bsm_tcp_msg_dec

// Numerical struct of BSM and application data .. to be used with connected ROS SOCKET
// Size:
typedef struct {
		double latitude;
		double longitude;
		double elevation;
		double speed;
		double heading;
		uint32_t gpsstatus;
		uint32_t speed_confidence;
		uint32_t head_confidence;
		uint32_t throtl_confidence;
		uint32_t elev_confidence;
		uint32_t time_confidence;
		uint32_t pos_confidence;
		double radius;
		double confidence;
		uint32_t count;
		double length;
		double width;
		double vehicleheight;
		int vehiclemass;
		uint32_t basicvehicleclass;
		uint32_t vehicletype;
		double bumperheight_front;
		double bumperheight_rear;
		uint32_t auxbrakes;
		uint32_t wheelbrake;
		uint32_t wheelbrakeavailable;
		uint32_t sparebit;
		uint32_t brakeboost;
		int abs;
		int stabilitycontrol;
		int traction;
		double angle;
		double longaccel;
		double lataccel;
		double vertaccel;
		double yawrate;
		double throttle_pos;
		uint32_t event_hazardlights;
		uint32_t event_absactivate;
		uint32_t event_tractionctrlloss;
		uint32_t event_stabilityctrlactivated;
		uint32_t event_hazardbraking;
		uint32_t event_airbag;
		uint32_t siren_in_use;
		uint32_t highbeam;
		uint32_t lowbeam;
		uint32_t rightturnsignal;
		uint32_t leftturnsignal;
		uint32_t hazardlights;
		uint32_t autolightcontrol;
		uint32_t dtimerunlights;
		uint32_t foglights;
		uint32_t parkinglights;
		uint32_t lightbarinuse;
		int wipers_swfnt;
		int wipers_rtfnt;
		int wipers_swrear;
		int wipers_rtrear;
		double positionalaccuracy0; //SemiMajorAxisAccuracy
		double positionalaccuracy1; //SemiMinorAxisAccuracy
		double positionalaccuracy2; //SemiMajorAxisOrientation
		uint32_t transmissionstate0;
		uint32_t transmissionstate1;
		uint32_t transmissionstate2;
		uint32_t transmissionstate3;
		int event_descr_len;
		double bsm_interval; //Contains dynamic beaconing rate (100-1000 ms)
		double distance; //distance (m) between pair of vehicles
}bsm_data_dec;



#endif /* ndef _bsm_tcp_msg_dec */
