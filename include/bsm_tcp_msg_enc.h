/* `_bsm_tcp_msg_enc.h'
 * 	2019 - Foad Hajiaghajani - foadhaji [at] buffalo.edu
 * 	Compiled by and verified on SAVARI SDK v5.10.1.7
 *	Compiled by and verified on C++11-ROS_Kinetic-Ubuntu_16.04 
 *	This header file must be loaded in /savari_ros/include/
 */

#ifndef _bsm_tcp_msg_enc /* Prevent multiple inclusion */
#define _bsm_tcp_msg_enc

// Char struct of BSM and application data .. to be used with connected ROS SOCKET
// Size: 436 bytes
typedef struct {
		char latitude[16];
		char longitude[16];
		char elevation[8];
		char speed[8];
		char heading[8];
		char gpsstatus[4];
		char speed_confidence[8];
		char head_confidence[8];
		char throtl_confidence[8];
		char elev_confidence[8];
		char time_confidence[8];
		char pos_confidence[8];
		char radius[16];
		char confidence[16];
		char count[8];
		char length[8];
		char width[8];
		char vehicleheight[8];
		char vehiclemass[4];
		char basicvehicleclass[4];
		char vehicletype[4];
		char bumperheight_front[8];
		char bumperheight_rear[8];
		char auxbrakes[4];
		char wheelbrake[4];
		char wheelbrakeavailable[4];
		char sparebit[4];
		char abs[4];
		char brakeboost[4];
		char stabilitycontrol[4];
		char traction[4];
		char angle[16];
		char longaccel[16];
		char lataccel[16];
		char vertaccel[16];
		char yawrate[16];
		char event_hazardlights[4];
		char event_absactivate[4];
		char event_tractionctrlloss[4];
		char event_stabilityctrlactivated[4];
		char event_hazardbraking[4];
		char event_airbag[4];
		char throttle_pos[16];
		char siren_in_use[4];
		char highbeam[2];
		char lowbeam[2];
		char rightturnsignal[2];
		char leftturnsignal[2];
		char hazardlights[2];
		char autolightcontrol[2];
		char dtimerunlights[2];
		char foglights[2];
		char parkinglights[2];
		char lightbarinuse[2];
		char wipers_swfnt[2];
		char wipers_rtfnt[2];
		char wipers_swrear[2];
		char wipers_rtrear[2];
		char positionalaccuracy0[8]; //SemiMajorAxisAccuracy
		char positionalaccuracy1[8]; //SemiMinorAxisAccuracy
		char positionalaccuracy2[8]; //SemiMajorAxisOrientation
		char transmissionstate0[2];
		char transmissionstate1[2];
		char transmissionstate2[2];
		char transmissionstate3[2];
		char event_descr_len[4];
		char bsm_interval[8]; //Contains dynamic beaconing rate (100-1000 ms)
		char distance[8]; //distance (m) between pair of vehicles
}bsm_data_enc;



#endif /* ndef _bsm_tcp_msg_enc */
