/*
Author：Sun Minxing
School: Institute of Optics And Electronics, Chinese Academy of Science
Email： 401435318@qq.com
*/
#pragma once

#include <ros/ros.h>
#include <ros/package.h>
#include <urdf/model.h>
#include "FusionEstimation.h"

// #include <ocs2_quadruped_msgs/LowState.h>
#include <fusion_estimator/LowState.h>
#define FusionEstimator_Package_LowState fusion_estimator::LowState
#include <nav_msgs/Odometry.h>
#define FusionEstimator_Package_Odometry nav_msgs::Odometry

namespace DataFusion
{
  class SensorIMU : public FusionEstimation
  {
    public:
      SensorIMU()
      : FusionEstimation{}
      {
        // Obtain siganl topic name from yaml
        ROS_nh_FusionEstimator.getParam("fusion_estimator/Sensor_Signal_Source_1", SignalSourceTopic);
        ROS_sub_FusionEstimatorIMU = ROS_nh_FusionEstimator.subscribe<FusionEstimator_Package_LowState>(SignalSourceTopic, 1, boost::bind(&SensorIMU::IMUCallback, this, _1));
        ROS_INFO("IMU subscribe from: %s", SignalSourceTopic.c_str());

        SensorErrorCode = 101;
        xyz_Available[2] = 1;
        rpy_Available[0] = 1;
        rpy_Available[1] = 1;
      }

    private:
      std::string SignalSourceTopic;
      FusionEstimator_Package_LowState SignalSourceData;
      ros::Subscriber ROS_sub_FusionEstimatorIMU;
      void IMUCallback(const FusionEstimator_Package_LowState::ConstPtr& msg)
      {
          SignalSourceData = *msg;
          SignalSourceDataRecieved = true;
      }

      void Est_DataObtain() override;
      void FurtherHandleXYZ2() override;
      void FurtherHandleRPY0() override;
  };

  class SensorHip : public FusionEstimation
  {
    public:
      SensorHip(const std::string& hipName)
      : FusionEstimation{}, Par_HipName(hipName)
      {
        // Obtain siganl topic name and urdf name from yaml
        ROS_nh_FusionEstimator.getParam("fusion_estimator/urdf_file_name", urdfFileName);
        ROS_nh_FusionEstimator.getParam("fusion_estimator/urdf_file_package", urdfFilePackage);
        std::string package_path = ros::package::getPath(urdfFilePackage);
        urdfFilePath = package_path + "/" + urdfFileName;
        ROS_INFO("urdf path: %s", urdfFilePath.c_str());
        ROS_nh_FusionEstimator.getParam("fusion_estimator/urdf_BaseName", Par_BaseName);
        ROS_nh_FusionEstimator.getParam("fusion_estimator/urdf_HipJointName", Par_HipJointName);
        ROS_nh_FusionEstimator.getParam("fusion_estimator/urdf_HipLengthName", Par_HipLengthName);
        ROS_nh_FusionEstimator.getParam("fusion_estimator/urdf_ThighLengthName", Par_ThighLengthName);
        ROS_nh_FusionEstimator.getParam("fusion_estimator/urdf_CalfLengthName", Par_CalfLengthName);
        ROS_nh_FusionEstimator.getParam("fusion_estimator/urdf_FootLengthName", Par_FootLengthName);
        ROS_nh_FusionEstimator.getParam("fusion_estimator/Sensor_Signal_Source_1", SignalSourceTopic);
        ROS_sub_FusionEstimatorHip = ROS_nh_FusionEstimator.subscribe<FusionEstimator_Package_LowState>(SignalSourceTopic, 1, boost::bind(&SensorHip::HipCallback, this, _1));
        ROS_INFO("Hip subscribe from: %s", SignalSourceTopic.c_str());

        Hip_BodyParameterObtain(); //Include sensor position obtaining

        xyz_Available[0] = 1;
        xyz_Available[1] = 1;
        XYZ_R[1] = {1}; // Set velocity noise to be biger to decrease peak
        XYZ_R[4] = {1}; // Set velocity noise to be biger to decrease peak
        XYZ_R[7] = {100}; // Set velocity noise to be biger to decrease peak

        if(Par_HipName == "RF")
        {
          JointDataOrder = 0;
          FeetNumber = 1;
        }
        if(Par_HipName == "LF")
        {
          JointDataOrder = 3;
          FeetNumber = 2;
        }
        if(Par_HipName == "LH")
        {
          JointDataOrder = 9;
          FeetNumber = 3;
        }
        if(Par_HipName == "RH")
        {
          JointDataOrder = 6;
          FeetNumber = 4;
        }
        SensorErrorCode = 200 + FeetNumber;
      }

      double FootHipCorrectPar[6] = {-0.9, 1, 1, -1, 1, 1};  //xyz0_Data and xyz1_Data will seperately multiply these number to resize
      // Gazebo Simulation {-0.9, 1, 1, -1, 1, 1};

      void OffsetReconfig(double Data[16]) override;

    protected:
      std::string SignalSourceTopic;
      std::string urdfFilePackage;
      std::string urdfFileName;
      std::string urdfFilePath;
      FusionEstimator_Package_LowState SignalSourceData;
      ros::Subscriber ROS_sub_FusionEstimatorHip;
      void HipCallback(const FusionEstimator_Package_LowState::ConstPtr& msg)
      {
          SignalSourceData = *msg;
          SignalSourceDataRecieved = true;
      }

      std::string Par_HipName;
      std::string Par_BaseName;
      std::string Par_HipJointName;
      std::string Par_HipLengthName;
      std::string Par_ThighLengthName;
      std::string Par_CalfLengthName;
      std::string Par_FootLengthName;

      double Par_HipLength, Par_ThighLength, Par_CalfLength, Par_FootLength, Par_BodyLength, Par_BodyWidth, Par_BodyHeight, Par_HipPosition[3];

      int JointDataOrder, FeetNumber;
      double LatestFeetEffort;
      double LatestJointAngle[3], LatestJointVelocity[3];
      double FootEffortThreshold = 60;
      bool FootIsOnGround = true, FootWasOnGround = true, FootLanding = false;
      int FootfallPositionRecordIsInitiated = 0;
      double FootfallPositionRecord[3] = {0};

      void Est_DataObtain() override;
      void FurtherHandleXYZ0() override;
      void FurtherHandleXYZ1() override;
      void Hip_BodyParameterObtain();
      void Hip_FootHipPosition();
      void Hip_LegJoint2HipFoot(double *Angle, double *AngleVelocity, double *HipFootPosition, double *HipFootVelocity, double SideSign, double Par_HipLength, double Par_ThighLength, double Par_CalfLength, double Par_FootLength);
      void Hip_FootFallPositionCorrect();

      // Cubature Kalman Estimator
      bool CKF6d_IsInitialized = false;
      bool CKF6d_Enable = true;
      const int CKF6d_StateDimension = 6;
      const int CKF6d_ObserDimension = 6;
      double Est_LegPar_hip, Est_LegPar_thigh, Est_LegPar_calf, Est_LegPar_foot;
      double CKF6d_LegPar_hip, CKF6d_LegPar_thigh, CKF6d_LegPar_calf, CKF6d_LegPar_foot;
      Eigen::MatrixXd CKF6d_CPoints0, CKF6d_CPoints1, CKF6d_CPoints2, CKF6d_SqrtP, CKF6d_State, CKF6d_Observation, CKF6d_ObserError;
      Eigen::MatrixXd CKF6d_Ppre, CKF6d_Pzz, CKF6d_Pxz, CKF6d_KlmGain;
      Eigen::MatrixXd CKF6d_X11, CKF6d_XX1, CKF6d_Z11, CKF6d_Output;
      Eigen::LLT<Eigen::MatrixXd> SqrtPinfo;
      Eigen::MatrixXd CKF6d_ParZ, CKF6d_ParX, CKF6d_ParP, CKF6d_ParQ, CKF6d_ParR;
      double CKF6d_Parameter[3] = {0,0,1}, CKF6d_InitialState[6] = {0,0,-0.4,0,0,0};
      void Hip_CKF6dFunEstimation(Eigen::MatrixXd& Observation, Eigen::MatrixXd& Output, Eigen::MatrixXd& State, Eigen::MatrixXd& Matrix_P, Eigen::MatrixXd& Matrix_Q, Eigen::MatrixXd& Matrix_R, double (&Parameter)[3]);
      void Hip_CKF6dFunInitiation();
      void Hip_CKF6dFunTransition(Eigen::MatrixXd& Input, Eigen::MatrixXd& Output, double(&Parameter)[3]);
      void Hip_CKF6dFunObservation(Eigen::MatrixXd& Input, Eigen::MatrixXd& Output, double(&Parameter)[3]);
  };

  class SensorHipRF : public SensorHip
  {
    public:
      SensorHipRF()
      : SensorHip("RF")
      {
      }
  };

  class SensorHipLF : public SensorHip
  {
    public:
      SensorHipLF()
      : SensorHip("LF")
      {
      }
  };

  class SensorHipRH : public SensorHip
  {
    public:
      SensorHipRH()
      : SensorHip("LH")
      {
      }
  };

  class SensorHipLH : public SensorHip
  {
    public:
      SensorHipLH()
      : SensorHip("RH")
      {
      }
  };

  class SensorLidar : public FusionEstimation
  {
    public:
      SensorLidar()
      : FusionEstimation{}
      {
        // Obtain siganl topic name from yaml
        ROS_nh_FusionEstimator.getParam("fusion_estimator/Sensor_Signal_Source_2", SignalSourceTopic);
        ROS_sub_FusionEstimatorLidar = ROS_nh_FusionEstimator.subscribe<FusionEstimator_Package_Odometry>(SignalSourceTopic, 1, boost::bind(&SensorLidar::LidarCallback, this, _1));
        ROS_INFO("Lidar subscribe from: %s", SignalSourceTopic.c_str());

        SensorErrorCode = 301;
        xyz_Available[0] = 1;
      }

    private:
      std::string SignalSourceTopic;
      FusionEstimator_Package_Odometry SignalSourceData;
      ros::Subscriber ROS_sub_FusionEstimatorLidar;
      void LidarCallback(const FusionEstimator_Package_Odometry::ConstPtr& msg)
      {
          SignalSourceData = *msg;
          SignalSourceDataRecieved = true;
      }

      void Est_DataObtain() override;
      void FurtherHandleXYZ0() override;
  };

} // namespace legged
