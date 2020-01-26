/* `_msg_conversion_handler.h'
 * 	2019 - Foad Hajiaghajani - foadhaji [at] buffalo.edu
 * 	Compiled by and verified on SAVARI SDK v5.10.1.7
 *	Compiled by and verified on C++11-ROS_Kinetic-Ubuntu_16.04
 *	This header file must be loaded in /savari_ros/include/
 *
 *	All files are subject to further change or update
 *	Last update: Jan 2020
 */

#ifndef _msg_conversion_handler /* Prevent multiple inclusion */
#define _msg_conversion_handler

#include <inttypes.h>
#include <bsm_tcp_msg_enc.h>
#include <bsm_tcp_msg_dec.h>


// To store raw data [char] received in DSRC payload for self and ego vehicles
typedef struct bsm_tcp_data_enc{
	bsm_data_enc self;
	bsm_data_enc ego;
}bsm_tcp_data_enc_t;

// To store raw data [numerical] received in DSRC payload for self and ego vehicles
typedef struct bsm_tcp_data_dec{
	bsm_data_dec self;
	bsm_data_dec ego;
}bsm_tcp_data_dec_t;

/*
 * Converts a struct of char to a struct of numerical values
 */
bsm_data_dec cstr_2_num (bsm_data_enc data_enc){
	bsm_data_dec data_dec = {0};

	data_dec.latitude = (double)atof(data_enc.latitude);
	data_dec.longitude = (double)atof(data_enc.longitude);
	data_dec.elevation = (double)atof(data_enc.elevation);
	data_dec.speed = (double)atof(data_enc.speed);
	data_dec.heading = (double)atof(data_enc.heading);
	data_dec.gpsstatus = (uint32_t)atoi(data_enc.gpsstatus);
	data_dec.speed_confidence = (uint32_t)atoi(data_enc.speed_confidence);
	data_dec.head_confidence = (uint32_t)atoi(data_enc.head_confidence);
	data_dec.throtl_confidence = (uint32_t)atoi(data_enc.throtl_confidence);
	data_dec.elev_confidence = (uint32_t)atoi(data_enc.elev_confidence);
	data_dec.time_confidence = (uint32_t)atoi(data_enc.time_confidence);
	data_dec.pos_confidence = (uint32_t)atoi(data_enc.pos_confidence);
	data_dec.radius = (double)atof(data_enc.radius);
	data_dec.confidence = (double)atof(data_enc.confidence);
	data_dec.count = (uint32_t)atoi(data_enc.count);
	data_dec.length = (double)atof(data_enc.length);
	data_dec.width = (double)atof(data_enc.width);
	data_dec.vehicleheight = (double)atof(data_enc.vehicleheight);
	data_dec.vehiclemass = (int)atoi(data_enc.vehiclemass);
	data_dec.basicvehicleclass = (uint32_t)atoi(data_enc.basicvehicleclass);
	data_dec.vehicletype = (uint32_t)atoi(data_enc.vehicletype);
	data_dec.bumperheight_front = (double)atof(data_enc.bumperheight_front);
	data_dec.bumperheight_rear = (double)atof(data_enc.bumperheight_rear);
	data_dec.auxbrakes = (uint32_t)atoi(data_enc.auxbrakes);
	data_dec.wheelbrake = (uint32_t)atoi(data_enc.wheelbrake);
	data_dec.wheelbrakeavailable = (uint32_t)atoi(data_enc.wheelbrakeavailable);
	data_dec.sparebit = (uint32_t)atoi(data_enc.sparebit);
	data_dec.brakeboost = (uint32_t)atoi(data_enc.brakeboost);
	data_dec.abs = (int)atoi(data_enc.abs);
	data_dec.stabilitycontrol = (int)atoi(data_enc.stabilitycontrol);
	data_dec.traction = (int)atoi(data_enc.traction);
	data_dec.angle = (double)atof(data_enc.angle);
	data_dec.longaccel = (double)atof(data_enc.longaccel);
	data_dec.lataccel = (double)atof(data_enc.lataccel);
	data_dec.vertaccel = (double)atof(data_enc.vertaccel);
	data_dec.yawrate = (double)atof(data_enc.yawrate);
	data_dec.throttle_pos = (double)atof(data_enc.throttle_pos);
	data_dec.event_hazardlights = (uint32_t)atoi(data_enc.event_hazardlights);
	data_dec.event_absactivate = (uint32_t)atoi(data_enc.event_absactivate);
	data_dec.event_tractionctrlloss = (uint32_t)atoi(data_enc.event_tractionctrlloss);
	data_dec.event_stabilityctrlactivated = (uint32_t)atoi(data_enc.event_stabilityctrlactivated);
	data_dec.event_hazardbraking = (uint32_t)atoi(data_enc.event_hazardbraking);
	data_dec.event_airbag = (uint32_t)atoi(data_enc.event_airbag);
	data_dec.siren_in_use = (uint32_t)atoi(data_enc.siren_in_use);
	data_dec.highbeam = (uint32_t)atoi(data_enc.highbeam);
	data_dec.lowbeam = (uint32_t)atoi(data_enc.lowbeam);
	data_dec.rightturnsignal = (uint32_t)atoi(data_enc.rightturnsignal);
	data_dec.leftturnsignal = (uint32_t)atoi(data_enc.leftturnsignal);
	data_dec.hazardlights = (uint32_t)atoi(data_enc.hazardlights);
	data_dec.autolightcontrol = (uint32_t)atoi(data_enc.autolightcontrol);
	data_dec.dtimerunlights = (uint32_t)atoi(data_enc.dtimerunlights);
	data_dec.foglights = (uint32_t)atoi(data_enc.foglights);
	data_dec.parkinglights = (uint32_t)atoi(data_enc.parkinglights);
	data_dec.lightbarinuse = (uint32_t)atoi(data_enc.lightbarinuse);
	data_dec.wipers_swfnt = (int)atoi(data_enc.wipers_swfnt);
	data_dec.wipers_rtfnt = (int)atoi(data_enc.wipers_rtfnt);
	data_dec.wipers_swrear = (int)atoi(data_enc.wipers_swrear);
	data_dec.wipers_rtrear = (int)atoi(data_enc.wipers_rtrear);
	data_dec.positionalaccuracy0 = (double)atof(data_enc.positionalaccuracy0);
	data_dec.positionalaccuracy1 = (double)atof(data_enc.positionalaccuracy1);
	data_dec.positionalaccuracy2 = (double)atof(data_enc.positionalaccuracy2);
	data_dec.transmissionstate0 = (uint32_t)atoi(data_enc.transmissionstate0);
	data_dec.transmissionstate1 = (uint32_t)atoi(data_enc.transmissionstate1);
	data_dec.transmissionstate2 = (uint32_t)atoi(data_enc.transmissionstate2);
	data_dec.transmissionstate3 = (uint32_t)atoi(data_enc.transmissionstate3);
	data_dec.event_descr_len = (int)atoi(data_enc.event_descr_len);
	data_dec.bsm_interval = (double)atof(data_enc.bsm_interval);
	data_dec.distance = (double)atof(data_enc.distance);

	return (data_dec);
}

/*
 * Converts a received payload struct [char] to decoded payload struct
 */
bsm_tcp_data_dec decode_tcp_payload (bsm_tcp_data_enc p_enc){
	bsm_tcp_data_dec p_dec = {0};
	p_dec.self = cstr_2_num(p_enc.self);
	p_dec.ego = cstr_2_num(p_enc.ego);
	return (p_dec);
}


#endif /* ndef _msg_conversion_handler */
