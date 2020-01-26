/* `_ros_handler.h'
 * 	2019 - Foad Hajiaghajani - foadhaji [at] buffalo.edu
 * 	Compiled by and verified on SAVARI SDK v5.10.1.7
 *	Compiled by and verified on C++11-ROS_Kinetic-Ubuntu_16.04
 *	This header file must be loaded in /savari_ros/include/
 *
 *	All files are subject to further change or update
 *	Last update: Jan 2020
 */

#ifndef _ros_handler /* Prevent multiple inclusion */
#define _ros_handler
#include "ros/ros.h"
#include <iostream>

#include <sensor_msgs/NavSatFix.h>
#include <savari_msgs/Gps.h>
#include <savari_msgs/Path.h>
#include <savari_msgs/Vehicle.h>

#include <bsm_tcp_msg_enc.h>
#include <bsm_tcp_msg_dec.h>


class savariRosNode
{
public:
	ros::NodeHandle nh;

	sensor_msgs::NavSatFix self_fix_;
	savari_msgs::Gps self_gps_;
	savari_msgs::Vehicle self_vehicle_;
	savari_msgs::Path self_path_;
	sensor_msgs::NavSatFix ego_fix_;
	savari_msgs::Gps ego_gps_;
	savari_msgs::Vehicle ego_vehicle_;
	savari_msgs::Path ego_path_;

	bsm_tcp_data_enc_t payload_enc;
	bsm_tcp_data_dec_t payload_dec;

	// List of topics advertising self vehicle data
	ros::Publisher pub_self_fix = this->nh.advertise <sensor_msgs::NavSatFix>("/savari_ros/self/fix", 1000);
	ros::Publisher pub_self_gps = this->nh.advertise <savari_msgs::Gps>("/savari_ros/self/gps", 1000);
	ros::Publisher pub_self_path = this->nh.advertise <savari_msgs::Path>("/savari_ros/self/path", 1000);
	ros::Publisher pub_self_vehicle = this->nh.advertise <savari_msgs::Vehicle>("/savari_ros/self/vehicle", 1000);

	// List of topics advertising remote vehicle data (leader-follower application)
	ros::Publisher pub_ego_fix = this->nh.advertise <sensor_msgs::NavSatFix>("/savari_ros/ego/fix", 1000);
	ros::Publisher pub_ego_gps = this->nh.advertise <savari_msgs::Gps>("/savari_ros/ego/gps", 1000);
	ros::Publisher pub_ego_path = this->nh.advertise <savari_msgs::Path>("/savari_ros/ego/path", 1000);
	ros::Publisher pub_ego_vehicle = this->nh.advertise <savari_msgs::Vehicle>("/savari_ros/ego/vehicle", 1000);


	savariRosNode(){
	}
	/**
	 * Publish and advertise all available ROS topics
	 */
	void publish(){
		this->pub_self_fix.publish(this->self_fix_);
		this->pub_self_gps.publish(this->self_gps_);
		this->pub_self_vehicle.publish(this->self_vehicle_);
		this->pub_self_path.publish(this->self_path_);
		this->pub_ego_fix.publish(this->ego_fix_);
		this->pub_ego_gps.publish(this->ego_gps_);
		this->pub_ego_vehicle.publish(this->ego_vehicle_);
		this->pub_ego_path.publish(this->ego_path_);
	}

	/**
	 * Fill out the content of "savari_msgs/Gps" ROS topic related to self vehicle
	 * NOTE: "self vehicle" corresponds to the DSRC OBU which is phisically
	 * connected to machine running this ROS driver
	 */
	void fill_self_gps (){
		self_gps_.header.stamp = ros::Time::now();
		self_gps_.latitude = payload_dec.self.latitude;
		self_gps_.longitude = payload_dec.self.longitude;
		self_gps_.elevation = payload_dec.self.elevation;
		self_gps_.speed = payload_dec.self.speed;
		self_gps_.heading = payload_dec.self.heading;
		self_gps_.gpsstatus = payload_dec.self.gpsstatus;
	}

	/**
	 * Fill out the content of "savari_msgs/Vehicle" ROS topic related to self vehicle
	 * NOTE: "self vehicle" corresponds to the DSRC OBU which is phisically
	 * connected to machine running this ROS driver
	 */
	void fill_self_vehicle (){
		self_vehicle_.header.stamp = ros::Time::now();
		self_vehicle_.length = payload_dec.self.length;
		self_vehicle_.width = payload_dec.self.width;
		self_vehicle_.vehicleheight = payload_dec.self.vehicleheight;
		self_vehicle_.vehiclemass = payload_dec.self.vehiclemass;
		self_vehicle_.basicvehicleclass = payload_dec.self.basicvehicleclass;
		self_vehicle_.vehicletype = payload_dec.self.vehicletype;
		self_vehicle_.bumperheight_front = payload_dec.self.bumperheight_front;
		self_vehicle_.bumperheight_rear = payload_dec.self.bumperheight_rear;
		self_vehicle_.auxbrakes = payload_dec.self.auxbrakes;
		self_vehicle_.wheelbrake = payload_dec.self.wheelbrake;
		self_vehicle_.wheelbrakeavailable = payload_dec.self.wheelbrakeavailable;
		self_vehicle_.sparebit = payload_dec.self.sparebit;
		self_vehicle_.abs = payload_dec.self.abs;
		self_vehicle_.brakeboost = payload_dec.self.brakeboost;
		self_vehicle_.stabilitycontrol = payload_dec.self.stabilitycontrol;
		self_vehicle_.traction = payload_dec.self.traction;
		self_vehicle_.angle = payload_dec.self.angle;
		self_vehicle_.longaccel = payload_dec.self.longaccel;
		self_vehicle_.lataccel = payload_dec.self.lataccel;
		self_vehicle_.vertaccel = payload_dec.self.vertaccel;
		self_vehicle_.yawrate = payload_dec.self.yawrate;
		self_vehicle_.event_hazardlights = payload_dec.self.event_hazardlights;
		self_vehicle_.event_absactivate = payload_dec.self.event_absactivate;
		self_vehicle_.event_tractionctrlloss = payload_dec.self.event_tractionctrlloss;
		self_vehicle_.event_stabilityctrlactivated = payload_dec.self.event_stabilityctrlactivated;
		self_vehicle_.event_hazardbraking = payload_dec.self.event_hazardbraking;
		self_vehicle_.event_airbag = payload_dec.self.event_airbag;
		self_vehicle_.throttle_pos = payload_dec.self.throttle_pos;
		self_vehicle_.siren_in_use = payload_dec.self.siren_in_use;
		self_vehicle_.highbeam = payload_dec.self.highbeam;
		self_vehicle_.lowbeam = payload_dec.self.lowbeam;
		self_vehicle_.rightturnsignal = payload_dec.self.rightturnsignal;
		self_vehicle_.leftturnsignal = payload_dec.self.leftturnsignal;
		self_vehicle_.hazardlights = payload_dec.self.hazardlights;
		self_vehicle_.autolightcontrol = payload_dec.self.autolightcontrol;
		self_vehicle_.dtimerunlights = payload_dec.self.dtimerunlights;
		self_vehicle_.foglights = payload_dec.self.foglights;
		self_vehicle_.parkinglights = payload_dec.self.parkinglights;
		self_vehicle_.lightbarinuse = payload_dec.self.lightbarinuse;
		self_vehicle_.wipers_swfnt = payload_dec.self.wipers_swfnt;
		self_vehicle_.wipers_rtfnt = payload_dec.self.wipers_rtfnt;
		self_vehicle_.wipers_swrear = payload_dec.self.wipers_swrear;
		self_vehicle_.wipers_rtrear = payload_dec.self.wipers_rtrear;
		self_vehicle_.positionalaccuracy[0] = payload_dec.self.positionalaccuracy0;
		self_vehicle_.positionalaccuracy[1] = payload_dec.self.positionalaccuracy1;
		self_vehicle_.positionalaccuracy[2] = payload_dec.self.positionalaccuracy2;
		self_vehicle_.transmissionstate0 = payload_dec.self.transmissionstate0;
		self_vehicle_.transmissionstate1 = payload_dec.self.transmissionstate1;
		self_vehicle_.transmissionstate2 = payload_dec.self.transmissionstate2;
		self_vehicle_.transmissionstate3 = payload_dec.self.transmissionstate3;
		self_vehicle_.event_descr_len = payload_dec.self.event_descr_len;
		self_vehicle_.bsm_interval = payload_dec.self.bsm_interval;
		self_vehicle_.distance = payload_dec.self.distance;
	}

	/**
	 * Fill content of "savari_msgs/Path" ROS topic related to self vehicle
	 * NOTE: "self vehicle" corresponds to the DSRC OBU which is phisically
	 * connected to machine running this ROS driver
	 */
	void fill_self_path (){
		self_path_.header.stamp = ros::Time::now();
		self_path_.speed_confidence = payload_dec.self.speed_confidence;
		self_path_.head_confidence = payload_dec.self.head_confidence;
		self_path_.throtl_confidence = payload_dec.self.throtl_confidence;
		self_path_.elev_confidence = payload_dec.self.elev_confidence;
		self_path_.time_confidence = payload_dec.self.time_confidence;
		self_path_.pos_confidence = payload_dec.self.pos_confidence;
		self_path_.radius = payload_dec.self.radius;
		self_path_.confidence = payload_dec.self.confidence;
		self_path_.count = payload_dec.self.count;
	}

	/**
	 * Fill content of "sensor_msgs/NavSatFix" ROS topic related to self vehicle
	 * NOTE: "self vehicle" corresponds to the DSRC OBU which is phisically
	 * connected to machine running this ROS driver
	 */
	void fill_self_fix(){
		self_fix_.header.stamp = ros::Time::now();
		self_fix_.latitude = payload_dec.self.latitude;
		self_fix_.longitude = payload_dec.self.longitude;
		self_fix_.altitude = payload_dec.self.elevation;
		/*
		 * --------- Not defined as of now --------
			self_fix_.position_covariance
			self_fix_.position_covariance.assign
			self_fix_.position_covariance_type
			---- ---- ---- ---- ---- ---- ---- ----
		 */
	}

	/**
	 * Fill content of "sensor_msgs/NavSatFix" ROS topic related to remote vehicle
	 * NOTE: "remote vehicle" corresponds to the DSRC OBU(s) from which DSRC packets
	 * are received periodically.
	 */
	void fill_ego_fix(){
		ego_fix_.header.stamp = ros::Time::now();
		ego_fix_.latitude = payload_dec.ego.latitude;
		ego_fix_.longitude = payload_dec.ego.longitude;
		ego_fix_.altitude = payload_dec.ego.elevation;
		/*
		 * --------- Not defined as of now --------
			ego_fix_.position_covariance
			ego_fix_.position_covariance.assign
			ego_fix_.position_covariance_type
			---- ---- ---- ---- ---- ---- ---- ----
		 */
	}

	/**
	 * Fill content of "savari_msgs/Gps" ROS topic related to remote vehicle
	 * NOTE: "remote vehicle" corresponds to the DSRC OBU(s) from which DSRC packets
	 * are received periodically.
	 */
	void fill_ego_gps(){
		ego_gps_.header.stamp = ros::Time::now();
		ego_gps_.latitude = payload_dec.ego.latitude;
		ego_gps_.longitude = payload_dec.ego.longitude;
		ego_gps_.elevation = payload_dec.ego.elevation;
		ego_gps_.speed = payload_dec.ego.speed;
		ego_gps_.heading = payload_dec.ego.heading;
		ego_gps_.gpsstatus = payload_dec.ego.gpsstatus;
	}

	/**
	 * Fill content of "savari_msgs/Vehicle" ROS topic related to remote vehicle
	 * NOTE: "remote vehicle" corresponds to the DSRC OBU(s) from which DSRC packets
	 * are received periodically.
	 */
	void fill_ego_vehicle(){
		ego_vehicle_.header.stamp = ros::Time::now();
		ego_vehicle_.length = payload_dec.ego.length;
		ego_vehicle_.width = payload_dec.ego.width;
		ego_vehicle_.vehicleheight = payload_dec.ego.vehicleheight;
		ego_vehicle_.vehiclemass = payload_dec.ego.vehiclemass;
		ego_vehicle_.basicvehicleclass = payload_dec.ego.basicvehicleclass;
		ego_vehicle_.vehicletype = payload_dec.ego.vehicletype;
		ego_vehicle_.bumperheight_front = payload_dec.ego.bumperheight_front;
		ego_vehicle_.bumperheight_rear = payload_dec.ego.bumperheight_rear;
		ego_vehicle_.auxbrakes = payload_dec.ego.auxbrakes;
		ego_vehicle_.wheelbrake = payload_dec.ego.wheelbrake;
		ego_vehicle_.wheelbrakeavailable = payload_dec.ego.wheelbrakeavailable;
		ego_vehicle_.sparebit = payload_dec.ego.sparebit;
		ego_vehicle_.abs = payload_dec.ego.abs;
		ego_vehicle_.brakeboost = payload_dec.ego.brakeboost;
		ego_vehicle_.stabilitycontrol = payload_dec.ego.stabilitycontrol;
		ego_vehicle_.traction = payload_dec.ego.traction;
		ego_vehicle_.angle = payload_dec.ego.angle;
		ego_vehicle_.longaccel = payload_dec.ego.longaccel;
		ego_vehicle_.lataccel = payload_dec.ego.lataccel;
		ego_vehicle_.vertaccel = payload_dec.ego.vertaccel;
		ego_vehicle_.yawrate = payload_dec.ego.yawrate;
		ego_vehicle_.event_hazardlights = payload_dec.ego.event_hazardlights;
		ego_vehicle_.event_absactivate = payload_dec.ego.event_absactivate;
		ego_vehicle_.event_tractionctrlloss = payload_dec.ego.event_tractionctrlloss;
		ego_vehicle_.event_stabilityctrlactivated = payload_dec.ego.event_stabilityctrlactivated;
		ego_vehicle_.event_hazardbraking = payload_dec.ego.event_hazardbraking;
		ego_vehicle_.event_airbag = payload_dec.ego.event_airbag;
		ego_vehicle_.throttle_pos = payload_dec.ego.throttle_pos;
		ego_vehicle_.siren_in_use = payload_dec.ego.siren_in_use;
		ego_vehicle_.highbeam = payload_dec.ego.highbeam;
		ego_vehicle_.lowbeam = payload_dec.ego.lowbeam;
		ego_vehicle_.rightturnsignal = payload_dec.ego.rightturnsignal;
		ego_vehicle_.leftturnsignal = payload_dec.ego.leftturnsignal;
		ego_vehicle_.hazardlights = payload_dec.ego.hazardlights;
		ego_vehicle_.autolightcontrol = payload_dec.ego.autolightcontrol;
		ego_vehicle_.dtimerunlights = payload_dec.ego.dtimerunlights;
		ego_vehicle_.foglights = payload_dec.ego.foglights;
		ego_vehicle_.parkinglights = payload_dec.ego.parkinglights;
		ego_vehicle_.lightbarinuse = payload_dec.ego.lightbarinuse;
		ego_vehicle_.wipers_swfnt = payload_dec.ego.wipers_swfnt;
		ego_vehicle_.wipers_rtfnt = payload_dec.ego.wipers_rtfnt;
		ego_vehicle_.wipers_swrear = payload_dec.ego.wipers_swrear;
		ego_vehicle_.wipers_rtrear = payload_dec.ego.wipers_rtrear;
		ego_vehicle_.positionalaccuracy[0] = payload_dec.ego.positionalaccuracy0;
		ego_vehicle_.positionalaccuracy[1] = payload_dec.ego.positionalaccuracy1;
		ego_vehicle_.positionalaccuracy[2] = payload_dec.ego.positionalaccuracy2;
		ego_vehicle_.transmissionstate0 = payload_dec.ego.transmissionstate0;
		ego_vehicle_.transmissionstate1 = payload_dec.ego.transmissionstate1;
		ego_vehicle_.transmissionstate2 = payload_dec.ego.transmissionstate2;
		ego_vehicle_.transmissionstate3 = payload_dec.ego.transmissionstate3;
		ego_vehicle_.event_descr_len = payload_dec.ego.event_descr_len;
		ego_vehicle_.bsm_interval = payload_dec.ego.bsm_interval;
		ego_vehicle_.distance = payload_dec.ego.distance;
	}

	/**
	 * Fill content of "savari_msgs/Path" ROS topic related to remote vehicle
	 * NOTE: "remote vehicle" corresponds to the DSRC OBU(s) from which DSRC packets
	 * are received periodically.
	 */
	void fill_ego_path(){
		ego_path_.header.stamp = ros::Time::now();
		ego_path_.speed_confidence = payload_dec.ego.speed_confidence;
		ego_path_.head_confidence = payload_dec.ego.head_confidence;
		ego_path_.throtl_confidence = payload_dec.ego.throtl_confidence;
		ego_path_.elev_confidence = payload_dec.ego.elev_confidence;
		ego_path_.time_confidence = payload_dec.ego.time_confidence;
		ego_path_.pos_confidence = payload_dec.ego.pos_confidence;
		ego_path_.radius = payload_dec.ego.radius;
		ego_path_.confidence = payload_dec.ego.confidence;
		ego_path_.count = payload_dec.ego.count;
	}

	/**
	 * Function call bridge to fill out the content of ROS messages to be later published
	 */
	void fill_ros_msg (){
		// fill msg for self vehicle
		fill_self_fix();
		fill_self_gps();
		fill_self_vehicle();
		fill_self_path();

		// fill msg for remote vehicle
		fill_ego_fix();
		fill_ego_gps();
		fill_ego_vehicle();
		fill_ego_path();
	}

	/**
	 * This fuction returns the boolean state of ROS node-handle
	 */
	bool rosOK(){
		return (this->nh.ok());
	}


private:
};

#endif /* ndef _ros_handler */
