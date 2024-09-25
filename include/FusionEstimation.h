/*
Author：Sun Minxing
School: Institute of Optics And Electronics, Chinese Academy of Science
Email： 401435318@qq.com
*/
#pragma once

#include <ros/ros.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cmath>
#include <functional>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <fusion_estimator/FusionEstimatorTest.h>
#define FusionEstimator_Package_FusionEstimatorTest fusion_estimator::FusionEstimatorTest
#include <ocs2_quadruped_msgs/RobotState.h>
#define FusionEstimator_Package_RobotState ocs2_quadruped_msgs::RobotState

namespace DataFusion
{
  #ifndef M_PI
  #define M_PI 3.14159265358979323846
  #endif

  // Initial states
  extern double InitialState[6];               // [Xposition Yposition Zposition RollOrientation PitchOrientation YawOrientation]

  // Public variables for all sensors
  extern Eigen::MatrixXd Est_StateXYZ;         // Estimated States [Xposition Xvelocity Xacceleration Yp Yv Ya Zp Zv Za]
  extern Eigen::MatrixXd Est_ProXYZ;           // Estimation Error Covariance
  extern double Est_Par_XYZ_Q[9];              // Process Noise Covariance
  extern double Est_XYZTimeRecord;             // Record last estimation time for calculating interval
  extern double Est_PositionRecord[3];         // Record last postion for reset
  extern double Est_OutputedPointPosition[3];  // Maybe you want to output other position like dog head
  extern double MapHeightStore[3][100];        // Store steps height. Each row seperately stored Z, confidence, stored time

  extern Eigen::MatrixXd Est_StateRPY;         // Estimated States [ROLLorientation ROLLvelocity ROLLacceleration Po Pv Pa Yo Yv Ya]
  extern Eigen::MatrixXd Est_ProRPY;           // Estimation Error Covariance
  extern double Est_Par_RPY_Q[9];              // Process Noise Covariance
  extern double Est_RPYTimeRecord;             // Record last estimation time for calculating interval
  extern double Est_Orientation[3];            // Body Orientation
  extern Eigen::Quaterniond Est_Quaternion;    // Body Orientation Quaternion
  extern Eigen::Quaterniond Est_QuaternionInv; // Body Orientation Quaternion Inverse
  extern double Est_OrientationRecord[3];      // For Reset

  extern double Est_FeetIsContact[4];          // Estimated States
  extern double Est_JointAngle[12];            // Estimated States
  extern double Est_JointAngleVelocity[12];    // Estimated States

  // Communication variables
  extern ros::Time ROS_nh_CurrentTime;
  extern bool Est_IsTopicInitialized;          // Initiate the topic to publish estimation result
  extern int Est_ErrorCount;
  extern FusionEstimator_Package_FusionEstimatorTest ROS_MsgTest;  // The topic to publish estimation result

  class FusionEstimation
  {
  public:
    FusionEstimation();

    void Est_FunOutput();

    void Est_Estimation()
    {
      if(SignalSourceDataRecieved)
      {
        Est_DataObtain();
        Est_DataPreHandle();
        Est_DataEstimation();
      }
    }

    virtual void OffsetReconfig(double Data[16])
    {
      xyz0_DataOffset[0] = Data[0];
      xyz0_DataOffset[1] = Data[1];
      xyz0_DataOffset[2] = Data[2];
      xyz1_DataOffset[0] = Data[3];
      xyz1_DataOffset[1] = Data[4];
      xyz1_DataOffset[2] = Data[5];
      xyz2_DataOffset[0] = Data[6];
      xyz2_DataOffset[1] = Data[7];
      xyz2_DataOffset[2] = Data[8];
    }

  protected:

    // Communication
    ros::NodeHandle ROS_nh_FusionEstimator;
    ros::Publisher ROS_pub_FusionEstimatorOutput;
    ros::Publisher ROS_pub_FusionEstimatedState;
    void ROS_FunInitTopic();
    bool SignalSourceDataRecieved = false;

    //Universal variables
    bool Est_IsInitialized = false;
    double Est_CurrentTime;

    //Intermediate variable
    Eigen::Quaterniond Est_QuaternionTemp1, Est_QuaternionTemp2, Est_QuaternionTemp3;
    double  Est_AdjustedObservation[3];
    Eigen::Vector3d Est_BodyAngleVel, Est_SensorWorldPosition, Est_SensorWorldVelocity, Est_SensorPosition, Est_Vector3dTemp1, Est_Vector3dTemp2;
    double Est_InputX[3] = {0,0,0}, Est_InputY[3] = {0,0,0}, Est_InputZ[3] = {0,0,0};
    double Est_InputRoll[3] = {0,0,0}, Est_InputPitch[3] = {0,0,0}, Est_InputYaw[3] = {0,0,0};

    //Core functions
    void Est_FunInitiation(double EstimatorInitialPosition_x, double EstimatorInitialPosition_y, double EstimatorInitialPosition_z, double EstimatorInitialOrientationRoll, double EstimatorInitialOrientationPitch, double EstimatorInitialOrientationYaw);

    void Est_FunDataHandle(double Observation[], Eigen::Quaterniond SensorOriQuat, Eigen::Quaterniond SensorOriQuatInv, int BinaryMark, double SensorPosition[], double DataOffset[], std::function<void()> FunctionHandler);

    void Est_FunEstimation(Eigen::MatrixXd* EstimatedState, Eigen::MatrixXd* EstimationPro, double ObserX[], double ObserY[], double ObserZ[], int BinaryMark, double NoiseR[], double NoiseQ[], double* LastEstimationTime, double ProcedureMark);

    // Kalman Estiamtor
    const int Kalman9d_StateDimension = 9;

    double Par_Kalman9d_F[9] = {1,1,1,1,1,1,1,1,1}; // diagonal element
    double Par_Kalman9d_G[9] = {1,1,1,1,1,1,1,1,1};
    double Par_Kalman9d_Q[9] = {1,1,1,1,1,1,1,1,1};
    double Par_Kalman9d_H[9] = {1,1,1,1,1,1,1,1,1};
    double Par_Kalman9d_R[9] = {1,1,1,1,1,1,1,1,1};
    double Par_Kalman9d_Xe[9] = {0,0,0,0,0,0,0,0,0};

    Eigen::MatrixXd Kalman9d_F, Kalman9d_FT, Kalman9d_G, Kalman9d_H, Kalman9d_HT, Kalman9d_Q, Kalman9d_R, Kalman9d_Xe, Kalman9d_Z, Kalman9d_Ze, Kalman9d_P, Kalman9d_P_pre, Kalman9d_K,
        Kalman9d_111, Kalman9d_1Z1, Kalman9d_1Z2, Kalman9d_XX1, Kalman9d_XX2, Kalman9d_ZX1, Kalman9d_XZ1, Kalman9d_XZp1, Kalman9d_ZZ1, Kalman9d_ZZ2, Kalman9d_X11, Kalman9d_X12, Kalman9d_Z11, Kalman9d_Z12, Kalman9d_eyeX;
    void Kalman9d_FunInitiation();
    void Kalman9d_FunEstimation();

    //Sensors variables
    virtual void Est_DataObtain() {}
    void Est_DataPreHandle();
    virtual void FurtherHandleXYZ0() {}
    virtual void FurtherHandleXYZ1() {}
    virtual void FurtherHandleXYZ2() {}
    virtual void FurtherHandleRPY0() {}
    virtual void FurtherHandleRPY1() {}
    virtual void FurtherHandleRPY2() {}
    void Est_DataEstimation();

    bool EstimationFlag = false;

    int SensorErrorCode = 0; // A casual sensor serial number for error location

    double SensorPosition[3] = {0,0,0};

    Eigen::Quaterniond SensorQuaternion;
    Eigen::Quaterniond SensorQuaternionInv;

    int xyz_Available[3] = {0,0,0}; // [position, linear velocity, linear acceleration] ~ avaliable
    int rpy_Available[3] = {0,0,0}; // [orientation, angular velocity, angular acceleration] ~ avaliable

    double xyz0_Data[3] = {0,0,0}; // position
    double xyz0_DataOffset[3] = {0,0,0};

    double xyz1_Data[3] = {0,0,0}; // linear velocity
    double xyz1_DataOffset[3] = {0,0,0};

    double xyz2_Data[3] = {0,0,0}; // linear acceleration
    double xyz2_DataOffset[3] = {0,0,0};

    double rpy0_Data[3] = {0,0,0}; // orientation
    double rpy0_DataOffset[3] = {0,0,0};

    double rpy1_Data[3] = {0,0,0}; // angular velocity
    double rpy1_DataOffset[3] = {0,0,0};

    double rpy2_Data[3] = {0,0,0}; //angular acceleration
    double rpy2_DataOffset[3] = {0,0,0};

    double XYZ_R[9] = {1,1,1,1,1,1,1,1,1}; // sensor XYZ noise covariance diagonal
    double RPY_R[9] = {1,1,1,1,1,1,1,1,1}; // sensor RPY noise covariance diagonal

  };
} // namespace legged
