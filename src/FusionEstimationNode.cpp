#include <ros/ros.h>
#include "FusionEstimation.h"
#include "Sensors.h"

#include <dynamic_reconfigure/server.h>         //to timely modify parameters
#include <fusion_estimator/EstimatorConfig.h>   //to timely modify parameters
double ReconfigData[16];
bool ReconfigFlag = false;
void ROS_FunConfigCallback(fusion_estimator::EstimatorConfig &config, uint32_t level)
{
    if(config.ResetFlag)
    {
      config.ResetFlag = false;
    }

    ReconfigData[0] = config.EstimatorPar1;
    ReconfigData[1] = config.EstimatorPar2;
    ReconfigData[2] = config.EstimatorPar3;
    ReconfigData[3] = config.EstimatorPar4;
    ReconfigData[4] = config.EstimatorPar5;
    ReconfigData[5] = config.EstimatorPar6;
    ReconfigData[6] = config.EstimatorPar7;
    ReconfigData[7] = config.EstimatorPar8;
    ReconfigData[8] = config.EstimatorPar9;
    ReconfigData[9] = config.EstimatorPar10;
    ReconfigData[10] = config.EstimatorPar11;
    ReconfigData[11] = config.EstimatorPar12;
    ReconfigData[12] = config.EstimatorPar13;
    ReconfigData[13] = config.EstimatorPar14;
    ReconfigData[14] = config.EstimatorPar15;
    ReconfigData[15] = config.EstimatorPar16;

    ReconfigFlag = true;
}

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "fusion_estimator_operating_node");
    ros::NodeHandle nh("~");

    double frequency;

    nh.param<double>("fusion_estimation/frequency", frequency, 400.0);

    ros::Rate rate(frequency);

    // Modify parameters
    std::shared_ptr<dynamic_reconfigure::Server<fusion_estimator::EstimatorConfig>> ReconfigServer;
    dynamic_reconfigure::Server<fusion_estimator::EstimatorConfig>::CallbackType ReconfigCallback;
    ReconfigServer = std::make_shared<dynamic_reconfigure::Server<fusion_estimator::EstimatorConfig>>(nh);
    ReconfigCallback = boost::bind(&ROS_FunConfigCallback, _1, _2);
    ReconfigServer->setCallback(ReconfigCallback);

    // Construct estimator
    DataFusion::SensorIMU SensorIMUData{};
    DataFusion::SensorHipRF SensorRFHipData{};
    DataFusion::SensorHipLF SensorLFHipData{};
    DataFusion::SensorHipRH SensorRHHipData{};
    DataFusion::SensorHipLH SensorLHHipData{};
    DataFusion::SensorLidar SensorLidarData{};

    ROS_INFO("Fusion Estimator Node Start...");
    while (ros::ok())
    {
        ros::spinOnce();

        SensorIMUData.Est_Estimation();
        SensorRFHipData.Est_Estimation();
        SensorLFHipData.Est_Estimation();
        SensorRHHipData.Est_Estimation();
        SensorLHHipData.Est_Estimation();
        SensorLidarData.Est_Estimation();

        SensorIMUData.Est_FunOutput();

        if(ReconfigFlag)
        {
            ReconfigFlag = false;
            SensorRFHipData.OffsetReconfig(ReconfigData);
            SensorLFHipData.OffsetReconfig(ReconfigData);
            SensorRHHipData.OffsetReconfig(ReconfigData);
            SensorLHHipData.OffsetReconfig(ReconfigData);
        }

        rate.sleep();
    }

    return 0;
}