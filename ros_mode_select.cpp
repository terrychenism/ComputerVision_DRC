#include <ros/ros.h>
#include <ros/topic.h>
#include <rosbag/bag.h>
#include <multisense_ros/DeviceInfo.h>
#include <multisense_ros/RawCamConfig.h>
#include <multisense_ros/RawCamCal.h>
#include <multisense_ros/RawCamData.h>
#include <multisense_ros/RawLidarData.h>
#include <multisense_ros/RawLidarCal.h>
#include <stdio.h>
#include <ros/callback_queue.h>

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/StrParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#define TOPIC_DEVICE_INFO     "/multisense_sl/calibration/device_info"
#define TOPIC_RAW_CAM_CAL     "/multisense_sl/calibration/raw_cam_cal"
#define TOPIC_RAW_CAM_CONFIG  "/multisense_sl/calibration/raw_cam_config"
#define TOPIC_RAW_CAM_DATA    "/multisense_sl/calibration/raw_cam_data"
#define TOPIC_RAW_LIDAR       "/laser/calibration/raw_lidar_data"
#define TOPIC_RAW_LIDAR_CAL   "/laser/calibration/raw_lidar_cal"

class ModeSelect{

poblic 

    ModeSelect (ros::NodeHandle nh)
    {
    	std::string r; //maybe int? 
   		double s;

    	double speed = nh.getParm("speed", s);
    	string reso = nh.getParm("resolution", r);
    }

    void setConf(const dynamic_reconfigure::Config& conf)
	{
    dynamic_reconfigure::ReconfigureRequest  srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;

    srv_req.config = conf;

    ros::service::call("/multisense_sl/set_parameters", srv_req, srv_resp);
	}

	void setMotorSpeed(double radPerSec)
	{
	    dynamic_reconfigure::DoubleParameter double_param;
	    dynamic_reconfigure::Config          conf;

	    double_param.name  = "motor_speed";
	    double_param.value = radPerSec;
	    conf.doubles.push_back(double_param);

	    setConf(conf);
	}

	void setResolution(const std::string& res)
	{
	    dynamic_reconfigure::StrParameter str_param;
	    dynamic_reconfigure::Config       conf;

	    str_param.name  = "resolution";
	    str_param.value = res;
	    conf.strs.push_back(str_param);
	    
	    setConf(conf);
	}
	void setHighMode(string res, double speed){
		setMotorSpeed(speed);
		setResolution(res);
	}

	void setLowMode(string res, double speed){
		setMotorSpeed(speed);
		setResolution(res);
	}


    ~ModeSelect();


};
