#include "FusionEstimation.h"

namespace DataFusion
{
  double InitialState[6] = {0, 0, 0.1, 0, 0, 0};     // [Xposition Yposition Zposition RollOrientation PitchOrientation YawOrientation]

  // Public variables for all sensors
  Eigen::MatrixXd Est_StateXYZ;                     // Estimated States
  Eigen::MatrixXd Est_ProXYZ;                       // Estimation Error Covariance
  double Est_Par_XYZ_Q[9] = {1,1,1,1,1,1,1,1,1};    // Process Noise Covariance
  double Est_XYZTimeRecord = 0;                     // Record last estimation time for calculating interval
  double Est_PositionRecord[3] = {0, 0, 0};         // Record last postion for reset
  double Est_OutputedPointPosition[3] = {0, 0, 0};  // Maybe you want the position of dog head
  Eigen::MatrixXd Est_StateRPY;                     // Estimated States
  Eigen::MatrixXd Est_ProRPY;                       // Estimation Error Covariance
  double MapHeightStore[3][100] = {0};              // Store steps height. Each row seperately stored Z, confidence, stored time

  double Est_Par_RPY_Q[9] = {1,1,1,1,1,1,1,1,1};    // Process Noise Covariance
  double Est_RPYTimeRecord = 0;                     // Record last estimation time for calculating interval
  double Est_Orientation[3] = {0, 0, 0};            // Body Orientation
  Eigen::Quaterniond Est_Quaternion;                // Body Orientation Quaternion
  Eigen::Quaterniond Est_QuaternionInv;             // Body Orientation Quaternion Inverse
  double Est_OrientationRecord[3] = {0, 0, 0};      // For Reset

  double Est_FeetIsContact[4] = {0, 0, 0, 0};       // Estimated States
  double Est_JointAngle[12];                        // Estimated States
  double Est_JointAngleVelocity[12];                // Estimated States

  // Debug Tools
  ros::Time ROS_nh_CurrentTime;
  bool Est_IsTopicInitialized = false;              // Initiate the topic to publish estimation result
  int Est_ErrorCount = 0;
  FusionEstimator_Package_FusionEstimatorTest ROS_MsgTest;  // The topic to publish estimation result


  FusionEstimation::FusionEstimation()
  {
    ROS_nh_CurrentTime = ros::Time::now();
    
    Est_Quaternion = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    Est_QuaternionInv = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);

    SensorQuaternion = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    SensorQuaternionInv = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);

    if (!Est_IsTopicInitialized)
    {
      ROS_FunInitTopic();
      Est_IsTopicInitialized = true;
    }

    if (!Est_IsInitialized)
    {
      ROS_nh_FusionEstimator.getParam("fusion_estimator/Initial_State_X", InitialState[0]);
      ROS_nh_FusionEstimator.getParam("fusion_estimator/Initial_State_Y", InitialState[1]);
      ROS_nh_FusionEstimator.getParam("fusion_estimator/Initial_State_Z", InitialState[2]);
      ROS_nh_FusionEstimator.getParam("fusion_estimator/Initial_State_Roll", InitialState[3]);
      ROS_nh_FusionEstimator.getParam("fusion_estimator/Initial_State_Pitch", InitialState[4]);
      ROS_nh_FusionEstimator.getParam("fusion_estimator/Initial_State_Yaw", InitialState[5]);
      Est_FunInitiation(InitialState[0], InitialState[1], InitialState[2], InitialState[3], InitialState[4], InitialState[5]);
    }
  }

  void FusionEstimation::Est_FunInitiation(double EstimatorInitialPosition_x, double EstimatorInitialPosition_y, double EstimatorInitialPosition_z, double EstimatorInitialOrientationRoll, double EstimatorInitialOrientationPitch, double EstimatorInitialOrientationYaw)
  {
    int i, j;

    //Input check
    if(std::isnan(EstimatorInitialPosition_x)||std::isnan(EstimatorInitialPosition_y)||std::isnan(EstimatorInitialPosition_z))
    {
      EstimatorInitialPosition_x = 0;
      EstimatorInitialPosition_y = 0;
      EstimatorInitialPosition_z = 0;
      ROS_WARN("Position initiation Nan error and reset with 0,0,0 \n");
    }

    if(std::isnan(EstimatorInitialOrientationRoll)||std::isnan(EstimatorInitialOrientationPitch)||std::isnan(EstimatorInitialOrientationYaw))
    {
      EstimatorInitialOrientationRoll = 0;
      EstimatorInitialOrientationPitch = 0;
      EstimatorInitialOrientationYaw = 0;
      ROS_WARN("Orientation initiation Nan error and reset with 0,0,0 \n");
    }

    //Parameters
    Est_PositionRecord[0] = EstimatorInitialPosition_x;
    Est_PositionRecord[1] = EstimatorInitialPosition_y;
    Est_PositionRecord[2] = EstimatorInitialPosition_z;

    Est_StateXYZ.resize(Kalman9d_StateDimension, 1);
    Est_StateXYZ.setZero();
    Est_StateXYZ(0, 0) = EstimatorInitialPosition_x;
    Est_StateXYZ(3, 0) = EstimatorInitialPosition_y;
    Est_StateXYZ(6, 0) = EstimatorInitialPosition_z;

    Est_ProXYZ.resize(Kalman9d_StateDimension, Kalman9d_StateDimension);
    Est_ProXYZ.setIdentity();
    Est_OrientationRecord[0] = EstimatorInitialOrientationRoll;
    Est_OrientationRecord[1] = EstimatorInitialOrientationPitch;
    Est_OrientationRecord[2] = EstimatorInitialOrientationYaw;

    Est_StateRPY.resize(Kalman9d_StateDimension, 1);
    Est_StateRPY.setZero();
    Est_StateRPY(0, 0) = EstimatorInitialOrientationRoll;
    Est_StateRPY(3, 0) = EstimatorInitialOrientationPitch;
    Est_StateRPY(6, 0) = EstimatorInitialOrientationYaw;

    Est_ProRPY.resize(Kalman9d_StateDimension, Kalman9d_StateDimension);
    Est_ProRPY.setIdentity();
    Kalman9d_FunInitiation();
  }

  void FusionEstimation::ROS_FunInitTopic()
  {
    ROS_pub_FusionEstimatorOutput = ROS_nh_FusionEstimator.advertise<FusionEstimator_Package_FusionEstimatorTest>("FusionEstimator" , 10);
    ROS_pub_FusionEstimatedState = ROS_nh_FusionEstimator.advertise<FusionEstimator_Package_RobotState>("FusionEstimatedState", 10); ///state_estimation_lkf/state FusionEstimatedState
  }

  void FusionEstimation::Est_DataPreHandle()
  {
    if(EstimationFlag)
    {
      for(int k; k < 3; k++)
      {
        Est_InputX[k] = 0;
        Est_InputY[k] = 0;
        Est_InputZ[k] = 0;
        Est_InputRoll[k] = 0;
        Est_InputPitch[k] = 0;
        Est_InputYaw[k] = 0;
      }
      if(xyz_Available[0]) //Process linear Position
        Est_FunDataHandle(xyz0_Data, SensorQuaternion, SensorQuaternionInv, 1, SensorPosition, xyz0_DataOffset, [this]() { this->FurtherHandleXYZ0(); });
      if(xyz_Available[1]) //Process linear Velocity
        Est_FunDataHandle(xyz1_Data, SensorQuaternion, SensorQuaternionInv, 2, SensorPosition, xyz1_DataOffset, [this]() { this->FurtherHandleXYZ1(); });
      if(xyz_Available[2]) //Process linear Acceleration
        Est_FunDataHandle(xyz2_Data, SensorQuaternion, SensorQuaternionInv, 4, SensorPosition, xyz2_DataOffset, [this]() { this->FurtherHandleXYZ2(); });
      if(rpy_Available[0]) //Process Orientation
        Est_FunDataHandle(rpy0_Data, SensorQuaternion, SensorQuaternionInv, 8, SensorPosition, rpy0_DataOffset, [this]() { this->FurtherHandleRPY0(); });
      if(rpy_Available[1]) //Process Angular Velocity
        Est_FunDataHandle(rpy1_Data, SensorQuaternion, SensorQuaternionInv, 16, SensorPosition, rpy1_DataOffset, [this]() { this->FurtherHandleRPY1(); });
      if(rpy_Available[2]) //Process Angular Acceleration
        Est_FunDataHandle(rpy2_Data, SensorQuaternion, SensorQuaternionInv, 32, SensorPosition, rpy2_DataOffset, [this]() { this->FurtherHandleRPY2(); });
    }
  }

  //Sensor Data HandleFunc
  void FusionEstimation::Est_FunDataHandle(double Observation[], Eigen::Quaterniond SensorOriQuat, Eigen::Quaterniond SensorOriQuatInv, int BinaryMark, double SensorPosition[], double DataOffset[], std::function<void()> FurtherHandle)
  {
    int i, DataDim[6];
    for(i = 0; i < 6; i++)
    {
      if((int)((BinaryMark%(int)pow(2,i+1))/(int)pow(2,i)))
        DataDim[i] = 1;
      else
        DataDim[i] = 0;
    }

    if (DataDim[0])//Position Data
    {
      //Data Quaternion Adjust
      Est_QuaternionTemp1 = Eigen::Quaterniond(0, Observation[0] + DataOffset[0], Observation[1] + DataOffset[1], Observation[2] + DataOffset[2]);

      //Data Quaternion in Body Frame with Sensor Quaternion Modify
      Est_QuaternionTemp1 = SensorOriQuat * Est_QuaternionTemp1 * SensorOriQuatInv;

      //Data Quaternion in World Frame with Body Quaternion Modify
      Est_QuaternionTemp1 = Est_Quaternion * Est_QuaternionTemp1 * Est_QuaternionInv;

      //1. Calculate Sensor Position Compared to Body in World Frame
      Est_QuaternionTemp3 = Eigen::Quaterniond(0, SensorPosition[0], SensorPosition[1], SensorPosition[2]);
      Est_QuaternionTemp3 = Est_Quaternion * Est_QuaternionTemp3 * Est_QuaternionInv;

      //2. Calculate Foot Position Compared to Body in World Frame
      Est_AdjustedObservation[0] = Est_QuaternionTemp1.x() + Est_QuaternionTemp3.x();
      Est_AdjustedObservation[1] = Est_QuaternionTemp1.y() + Est_QuaternionTemp3.y();
      Est_AdjustedObservation[2] = Est_QuaternionTemp1.z() + Est_QuaternionTemp3.z();

      Est_InputX[0] = Est_AdjustedObservation[0];
      Est_InputY[0] = Est_AdjustedObservation[1];
      Est_InputZ[0] = Est_AdjustedObservation[2];
    }
    else if (DataDim[1])//Linear Velocity Data
    {
      //Data Orientation Adjust
      Est_QuaternionTemp1 = Eigen::Quaterniond(0, Observation[0] + DataOffset[0], Observation[1] + DataOffset[1], Observation[2] + DataOffset[2]);

      //Sensor Quaternion in Body Frame
      Est_QuaternionTemp1 = SensorOriQuat * Est_QuaternionTemp1 * SensorOriQuatInv;

      //Adjust with Sensor Angular Velocity in World Frame
      Est_QuaternionTemp1 = Est_Quaternion * Est_QuaternionTemp1 * Est_QuaternionInv;

      //1. Calculate Sensor to Body Distance in World Frame
      Est_QuaternionTemp3 = Eigen::Quaterniond(0, SensorPosition[0], SensorPosition[1], SensorPosition[2]);
      Est_QuaternionTemp3 = Est_Quaternion * Est_QuaternionTemp3 * Est_QuaternionInv;
      Est_SensorWorldPosition(0, 0) = Est_QuaternionTemp1.x();
      Est_SensorWorldPosition(1, 0) = Est_QuaternionTemp1.y();
      Est_SensorWorldPosition(2, 0) = Est_QuaternionTemp1.z();

      //2. Calculate Sensor Angular Velocity in World Frame
      Est_BodyAngleVel(0, 0) = Est_StateRPY(1, 0);
      Est_BodyAngleVel(1, 0) = Est_StateRPY(4, 0);
      Est_BodyAngleVel(2, 0) = Est_StateRPY(7, 0);
      Est_SensorWorldVelocity = Est_BodyAngleVel.cross(Est_SensorWorldPosition);

      //3. Calculate Body Velocity with  Angular Velocity  Compensation

      Est_QuaternionTemp1.x() -= Est_SensorWorldVelocity(0, 0);
      Est_QuaternionTemp1.y() -= Est_SensorWorldVelocity(1, 0);
      Est_QuaternionTemp1.z() -= Est_SensorWorldVelocity(2, 0);

      Est_AdjustedObservation[0] = - Est_QuaternionTemp1.x();
      Est_AdjustedObservation[1] = - Est_QuaternionTemp1.y();
      Est_AdjustedObservation[2] = - Est_QuaternionTemp1.z();

      Est_InputX[1] = Est_AdjustedObservation[0];
      Est_InputY[1] = Est_AdjustedObservation[1];
      Est_InputZ[1] = Est_AdjustedObservation[2];

      }
    else if (DataDim[2])//Linear Acceleration Data
    {
      //Data Orientation Adjust
      Est_QuaternionTemp1 = Eigen::Quaterniond(0, Observation[0] + DataOffset[0], Observation[1] + DataOffset[1], Observation[2] + DataOffset[2]);

      //Sensor Quaternion in Body Frame
      Est_QuaternionTemp1 = SensorOriQuat * Est_QuaternionTemp1 * SensorOriQuatInv;

      //Sensor acceleration in World Frame
      Est_QuaternionTemp1 = Est_Quaternion * Est_QuaternionTemp1 * Est_QuaternionInv;

      //1. Calculate Centrifugal Acceleration
      Est_SensorPosition << SensorPosition[0], SensorPosition[1], SensorPosition[2];
      Est_Vector3dTemp1 << Est_StateRPY(1, 0), Est_StateRPY(4, 0), Est_StateRPY(7, 0);
      Est_Vector3dTemp2 = Est_Vector3dTemp1.cross(Est_SensorPosition);
      Est_Vector3dTemp1 = Est_Vector3dTemp1.cross(Est_Vector3dTemp2);

      //3. Calculate Body Acceleration with Centrifugal Acceleration Compensation
      Est_QuaternionTemp1.x() -= Est_Vector3dTemp1(0);
      Est_QuaternionTemp1.y() -= Est_Vector3dTemp1(1);
      Est_QuaternionTemp1.z() -= Est_Vector3dTemp1(2);

      Est_AdjustedObservation[0] = Est_QuaternionTemp1.x();
      Est_AdjustedObservation[1] = Est_QuaternionTemp1.y();
      Est_AdjustedObservation[2] = Est_QuaternionTemp1.z();

      Est_InputX[2] = Est_AdjustedObservation[0];
      Est_InputY[2] = Est_AdjustedObservation[1];
      Est_InputZ[2] = Est_AdjustedObservation[2];
    }
    else if (DataDim[3])//Orientation Data
    {
      double Temp[3], Q[4];
      Temp[0] = Observation[0] + DataOffset[0];
      Temp[1] = Observation[1] + DataOffset[1];
      Temp[2] = Observation[2] + DataOffset[2];

      //Obtain Sensor Quaternion in World Frame
      Est_QuaternionTemp1 = Eigen::AngleAxisd(Temp[2], Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(Temp[1], Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(Temp[0], Eigen::Vector3d::UnitX());
      Est_QuaternionTemp2 = Est_Quaternion.inverse();

      //Obtain Body Quaternion in World Frame
      Est_QuaternionTemp2 = Est_QuaternionTemp1 * SensorOriQuatInv;

      //Obtain Body Orientation in World Frame
      Q[0] = Est_QuaternionTemp2.w();
      Q[1] = Est_QuaternionTemp2.x();
      Q[2] = Est_QuaternionTemp2.y();
      Q[3] = Est_QuaternionTemp2.z();

      Est_AdjustedObservation[0] = atan2(2 * (Q[0] * Q[1] + Q[2] * Q[3]), (1 - 2 * (Q[1] * Q[1] + Q[2] * Q[2])));
      Est_AdjustedObservation[1] = asin(2 * (Q[0] * Q[2] - Q[3] * Q[1]));
      Est_AdjustedObservation[2] = atan2(2 * (Q[0] * Q[3] + Q[1] * Q[2]), (1 - 2 * (Q[2] * Q[2] + Q[3] * Q[3])));

      Est_InputRoll[0] = Est_AdjustedObservation[0];
      Est_InputPitch[0] = Est_AdjustedObservation[1];
      Est_InputYaw[0] = Est_AdjustedObservation[2];
    }
    else if (DataDim[4])//Angular Velocity Data
    {
      //Data Orientation Adjust
      Est_QuaternionTemp1 = Eigen::Quaterniond(0, Observation[0] + DataOffset[0], Observation[1] + DataOffset[1], Observation[2] + DataOffset[2]);

      //Sensor Quaternion in Body Frame
      Est_QuaternionTemp1 = SensorOriQuat * Est_QuaternionTemp1 * SensorOriQuatInv;

      //Sensor Quaternion in World Frame
      Est_QuaternionTemp1 = Est_Quaternion * Est_QuaternionTemp1 * Est_QuaternionInv;

      Est_AdjustedObservation[0] = Est_QuaternionTemp1.x();
      Est_AdjustedObservation[1] = Est_QuaternionTemp1.y();
      Est_AdjustedObservation[2] = Est_QuaternionTemp1.z();

      Est_InputRoll[1]  = Est_AdjustedObservation[0];
      Est_InputPitch[1] = Est_AdjustedObservation[1];
      Est_InputYaw[1]   = Est_AdjustedObservation[2];
    }
    else if (DataDim[5])//Angular Acceleration Data
    {
      //Data Orientation Adjust
      Est_QuaternionTemp1 = Eigen::Quaterniond(0, Observation[0] + DataOffset[0], Observation[1] + DataOffset[1], Observation[2] + DataOffset[2]);

      //Sensor Quaternion in Body Frame
      Est_QuaternionTemp1 = SensorOriQuat * Est_QuaternionTemp1 * SensorOriQuatInv;

      //Sensor Quaternion in World Frame
      Est_QuaternionTemp1 = Est_Quaternion * Est_QuaternionTemp1 * Est_QuaternionInv;

      Est_AdjustedObservation[0] = Est_QuaternionTemp1.x();
      Est_AdjustedObservation[1] = Est_QuaternionTemp1.y();
      Est_AdjustedObservation[2] = Est_QuaternionTemp1.z();

      Est_InputRoll[2] = Est_AdjustedObservation[0];
      Est_InputPitch[2] = Est_AdjustedObservation[1];
      Est_InputYaw[2] = Est_AdjustedObservation[2];
    }

    FurtherHandle();
  }

  void FusionEstimation::Est_DataEstimation()
  {
    if (!Est_IsInitialized)
    {
      Est_IsInitialized = true;
      Est_RPYTimeRecord = ROS_nh_CurrentTime.toSec();
      Est_XYZTimeRecord = ROS_nh_CurrentTime.toSec();
    }
    if(EstimationFlag)
    {
      EstimationFlag = false;

      Est_CurrentTime = ROS_nh_CurrentTime.toSec();

      if(xyz_Available[0]+xyz_Available[1]+xyz_Available[2]) //Estimate XYZ axis
          Est_FunEstimation(&Est_StateXYZ, &Est_ProXYZ, Est_InputX, Est_InputY, Est_InputZ, xyz_Available[0] + 2 * xyz_Available[1] + 4 * xyz_Available[2], XYZ_R, Est_Par_XYZ_Q, &Est_XYZTimeRecord, 1 + 0.01*(xyz_Available[0] + 2 * xyz_Available[1] + 4 * xyz_Available[2]));
      if(rpy_Available[0]+rpy_Available[1]+rpy_Available[2]) //Estimate RPY axis
          Est_FunEstimation(&Est_StateRPY, &Est_ProRPY, Est_InputRoll, Est_InputPitch, Est_InputYaw, rpy_Available[0] + 2 * rpy_Available[1] + 4 * rpy_Available[2], RPY_R, Est_Par_RPY_Q, &Est_RPYTimeRecord, 2 + 0.01*(xyz_Available[0] + 2 * xyz_Available[1] + 4 * xyz_Available[2]));

      // Update body Quaternion
      if(rpy_Available[0]+rpy_Available[1]+rpy_Available[2])
      {
        Est_Quaternion = Eigen::AngleAxisd(Est_StateRPY(0, 0), Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(Est_StateRPY(3, 0), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(Est_StateRPY(6, 0), Eigen::Vector3d::UnitZ());
        Est_QuaternionInv = Est_Quaternion.inverse();
      }
    }
  }

  //Sensor Data Estimation
  void FusionEstimation::Est_FunEstimation(Eigen::MatrixXd* EstimatedState, Eigen::MatrixXd* EstimationPro, double ObserX[], double ObserY[], double ObserZ[], int BinaryMark, double NoiseR[], double NoiseQ[], double* LastEstimationTime, double ProcedureMark)
  {
    int i, DataDim[3];
    for(i = 0; i < 3; i++)
    {
      if((int)((BinaryMark%(int)pow(2,i+1))/(int)pow(2,i)))
        DataDim[i] = 1;
      else
        DataDim[i] = 0;
    }
    //1. State Transition Matrix F
    for(i = 0; i < 3; i++)
    {
        Kalman9d_F(i * 3, i * 3 + 1) = Est_CurrentTime - *LastEstimationTime;
        Kalman9d_F(i * 3 + 1, i * 3 + 2) = Est_CurrentTime - *LastEstimationTime;
        Kalman9d_F(i * 3, i * 3 + 2) = std::pow(Est_CurrentTime - *LastEstimationTime, 2) / 2;
    }
    //2. Prior State Matrix Xe
    Kalman9d_Xe = *EstimatedState;
    //3. Prior State Estimation Error Covariance Matrix P_Pre
    Kalman9d_P = *EstimationPro;
    //4.1 Measurement Matrix H
    Kalman9d_H.setZero();
    for(i = 0; i < 3; i++)
      if(DataDim[i])
      {
        Kalman9d_H(0 + i, 0 + i) = 1;
        Kalman9d_H(3 + i, 3 + i) = 1;
        Kalman9d_H(6 + i, 6 + i) = 1;
      }
    Kalman9d_HT = Kalman9d_H.transpose();
    //4.2 Observed Noise Covariance Matrix R
    Kalman9d_R.setZero();
    for (i = 0; i < Kalman9d_StateDimension; ++i)
    {
      Kalman9d_R(i, i) = NoiseR[i];
    }
    //4.3 State Transition Noise Covariance Matrix Q
    Kalman9d_Q.setZero();
    for (int i = 0; i < Kalman9d_StateDimension; ++i)
    {
      Kalman9d_Q(i, i) = NoiseQ[i];
    }
    Kalman9d_Z.setZero();
    //5. Observation Matrix Z
    for(i = 0; i < Kalman9d_StateDimension; i++)
      if(DataDim[i])
      {
        Kalman9d_Z(0 + i, 0) = ObserX[i];
        Kalman9d_Z(3 + i, 0) = ObserY[i];
        Kalman9d_Z(6 + i, 0) = ObserZ[i];

      }
    //6. Last Estimation Time
    *LastEstimationTime = Est_CurrentTime;
    //7. Estimation
    Kalman9d_FunEstimation();
    //8. Estimation Fault Detect
    for (i = 0; i < Kalman9d_StateDimension; ++i)
    {
      if (std::isnan(Kalman9d_Xe(i))||abs(Kalman9d_Xe(i))>10000)
      {
        if(Est_ErrorCount < 100)
        {
          std::cout <<"Sensor estiamtion error number count: "<<Est_ErrorCount<< " with state " << Kalman9d_Xe.transpose() <<" in procedure "<<ProcedureMark <<" on sensor "<< SensorErrorCode << std::endl;
          std::cout <<"Data Input XYZ0: "<< xyz0_Data[0] << ' ' << xyz0_Data[1] << ' ' << xyz0_Data[2] << std::endl;
          std::cout <<"Data Input XYZ1: "<< xyz1_Data[0] << ' ' << xyz1_Data[1] << ' ' << xyz1_Data[2] << std::endl;
          std::cout <<"Data Input XYZ2: "<< xyz2_Data[0] << ' ' << xyz2_Data[1] << ' ' << xyz2_Data[2] << std::endl;
          std::cout <<"Data Input RPY0: "<< rpy0_Data[0] << ' ' << rpy0_Data[1] << ' ' << rpy0_Data[2] << std::endl;
          std::cout <<"Data Input RPY1: "<< rpy1_Data[0] << ' ' << rpy1_Data[1] << ' ' << rpy1_Data[2] << std::endl;
          std::cout <<"Data Input RPY2: "<< rpy2_Data[0] << ' ' << rpy2_Data[1] << ' ' << rpy2_Data[2] << std::endl;
          std::cout <<"Estimator Observation: "<<Kalman9d_Z.transpose() << std::endl;
        }
        Est_ErrorCount++;

        Est_FunInitiation(Est_PositionRecord[0], Est_PositionRecord[1], Est_PositionRecord[2],
          Est_OrientationRecord[0], Est_OrientationRecord[1], Est_OrientationRecord[2]);

        break;
      }
    }
    //9. Estimation Parameter Store
    *EstimatedState = Kalman9d_Xe;
    *EstimationPro = Kalman9d_P;
  }

  void FusionEstimation::Est_FunOutput()
  {
    int i, j;
    Eigen::Quaterniond Est_InterestedPointQ, Est_LinearVelocityBaseFrameQ, Est_LinearAccelerationBaseFrameQ;
    Eigen::Quaterniond Est_AngularVelocityBaseFrameQ, Est_AngularAccelerationBaseFrameQ;

    //Record
    for (i = 0; i < 3; ++i)
    {
      if (!std::isnan(Est_StateXYZ(i*3)))
      {
        Est_PositionRecord[0] = Est_StateXYZ(i*3, 0);
      }
      if (!std::isnan(Est_StateRPY(i*3)))
      {
        Est_OrientationRecord[0] = Est_StateRPY(i*3, 0);
      }
    }

    //Interested point transformation
    Est_InterestedPointQ = Eigen::Quaterniond(0, Est_OutputedPointPosition[0], Est_OutputedPointPosition[1], Est_OutputedPointPosition[2]);
    Est_InterestedPointQ = Est_Quaternion * Est_InterestedPointQ * Est_QuaternionInv;

    //Angular velocity in base frame
    Est_AngularVelocityBaseFrameQ = Eigen::Quaterniond(0, Est_StateRPY(1, 0), Est_StateRPY(4, 0), Est_StateRPY(7, 0));
    Est_AngularVelocityBaseFrameQ = Est_QuaternionInv * Est_AngularVelocityBaseFrameQ * Est_Quaternion;

    //Angular acceleration in base frame
    Est_AngularAccelerationBaseFrameQ = Eigen::Quaterniond(0, Est_StateRPY(2, 0), Est_StateRPY(5, 0), Est_StateRPY(8, 0));
    Est_AngularAccelerationBaseFrameQ = Est_QuaternionInv * Est_AngularAccelerationBaseFrameQ * Est_Quaternion;

    //Linear velocity in base frame
    Est_LinearVelocityBaseFrameQ = Eigen::Quaterniond(0, Est_StateXYZ(1, 0), Est_StateXYZ(4, 0), Est_StateXYZ(7, 0));
    Est_LinearVelocityBaseFrameQ = Est_QuaternionInv * Est_LinearVelocityBaseFrameQ * Est_Quaternion;

    //Linear acceleration in base frame
    Est_LinearAccelerationBaseFrameQ = Eigen::Quaterniond(0, Est_StateXYZ(2, 0), Est_StateXYZ(5, 0), Est_StateXYZ(8, 0));
    Est_LinearAccelerationBaseFrameQ = Est_QuaternionInv * Est_LinearAccelerationBaseFrameQ * Est_Quaternion;

    //Public estimation result
    ROS_MsgTest.stamp = ROS_nh_CurrentTime;

    ROS_MsgTest.EstimatedXYZ[0] = Est_StateXYZ(0, 0) + Est_InterestedPointQ.x();
    ROS_MsgTest.EstimatedXYZ[1] = Est_StateXYZ(3, 0) + Est_InterestedPointQ.y();
    ROS_MsgTest.EstimatedXYZ[2] = Est_StateXYZ(6, 0) + Est_InterestedPointQ.z();
    ROS_MsgTest.EstimatedXYZ[3] = Est_StateXYZ(1, 0);
    ROS_MsgTest.EstimatedXYZ[4] = Est_StateXYZ(4, 0);
    ROS_MsgTest.EstimatedXYZ[5] = Est_StateXYZ(7, 0);
    ROS_MsgTest.EstimatedXYZ[6] = Est_StateXYZ(2, 0);
    ROS_MsgTest.EstimatedXYZ[7] = Est_StateXYZ(5, 0);
    ROS_MsgTest.EstimatedXYZ[8] = Est_StateXYZ(8, 0);

    ROS_MsgTest.EstimatedRPY[0] = Est_StateRPY(0, 0);
    ROS_MsgTest.EstimatedRPY[1] = Est_StateRPY(3, 0);
    ROS_MsgTest.EstimatedRPY[2] = Est_StateRPY(6, 0);
    ROS_MsgTest.EstimatedRPY[3] = Est_StateRPY(1, 0);
    ROS_MsgTest.EstimatedRPY[4] = Est_StateRPY(4, 0);
    ROS_MsgTest.EstimatedRPY[5] = Est_StateRPY(7, 0);
    ROS_MsgTest.EstimatedRPY[6] = Est_StateRPY(2, 0);
    ROS_MsgTest.EstimatedRPY[7] = Est_StateRPY(5, 0);
    ROS_MsgTest.EstimatedRPY[8] = Est_StateRPY(8, 0);

    //Public estimated result
    FusionEstimator_Package_RobotState Est_OutputState;

    Est_OutputState.stamp = ROS_nh_CurrentTime;

    Est_OutputState.contacts.resize(4);
    for (int i = 0; i < 4; ++i) {
        Est_OutputState.contacts[i] = Est_FeetIsContact[i];
    }

    Est_OutputState.x.reserve(3 + 3 + 3 + 3 + 12);
    Est_OutputState.u.reserve(12);

    Est_OutputState.x.push_back(Est_StateRPY(0, 0));
    Est_OutputState.x.push_back(Est_StateRPY(3, 0));
    Est_OutputState.x.push_back(Est_StateRPY(6, 0));

    Est_OutputState.x.push_back(Est_StateXYZ(0, 0) + Est_InterestedPointQ.x());
    Est_OutputState.x.push_back(Est_StateXYZ(3, 0) + Est_InterestedPointQ.y());
    Est_OutputState.x.push_back(Est_StateXYZ(6, 0) + Est_InterestedPointQ.z());

    Est_OutputState.x.push_back(Est_AngularVelocityBaseFrameQ.x());
    Est_OutputState.x.push_back(Est_AngularVelocityBaseFrameQ.y());
    Est_OutputState.x.push_back(Est_AngularVelocityBaseFrameQ.z());

    Est_OutputState.x.push_back(Est_LinearVelocityBaseFrameQ.x());
    Est_OutputState.x.push_back(Est_LinearVelocityBaseFrameQ.y());
    Est_OutputState.x.push_back(Est_LinearVelocityBaseFrameQ.z());

    // Set state: Joint angles
    for(int i = 0; i < 12; ++i) {
      Est_OutputState.x.push_back(Est_JointAngle[i]);
    }

    // Set input: Joint velocities
    for(int i = 0; i< 12; ++i) {
      Est_OutputState.u.push_back(Est_JointAngleVelocity[i]);
    }

    if(SignalSourceDataRecieved)
    {
      ROS_pub_FusionEstimatorOutput.publish(ROS_MsgTest);
      ROS_pub_FusionEstimatedState.publish(Est_OutputState);
    }
  }

  void FusionEstimation::Kalman9d_FunInitiation()
  {
    int i, j;

    Kalman9d_F.resize(Kalman9d_StateDimension, Kalman9d_StateDimension);
    Kalman9d_FT.resize(Kalman9d_StateDimension, Kalman9d_StateDimension);
    Kalman9d_G.resize(Kalman9d_StateDimension, Kalman9d_StateDimension);
    Kalman9d_H.resize(Kalman9d_StateDimension, Kalman9d_StateDimension);
    Kalman9d_HT.resize(Kalman9d_StateDimension, Kalman9d_StateDimension);
    Kalman9d_Q.resize(Kalman9d_StateDimension, Kalman9d_StateDimension);
    Kalman9d_R.resize(Kalman9d_StateDimension, Kalman9d_StateDimension);
    Kalman9d_Xe.resize(Kalman9d_StateDimension, 1);
    Kalman9d_Z.resize(Kalman9d_StateDimension, 1);
    Kalman9d_Ze.resize(Kalman9d_StateDimension, 1);
    Kalman9d_K.resize(Kalman9d_StateDimension, Kalman9d_StateDimension);
    Kalman9d_P.resize(Kalman9d_StateDimension, Kalman9d_StateDimension);
    Kalman9d_P_pre.resize(Kalman9d_StateDimension, Kalman9d_StateDimension);
    Kalman9d_XX1.resize(Kalman9d_StateDimension, Kalman9d_StateDimension);
    Kalman9d_ZZ1.resize(Kalman9d_StateDimension, Kalman9d_StateDimension);
    Kalman9d_ZZ2.resize(Kalman9d_StateDimension, Kalman9d_StateDimension);
    Kalman9d_Z11.resize(Kalman9d_StateDimension, 1);
    Kalman9d_eyeX.resize(Kalman9d_StateDimension, Kalman9d_StateDimension);

    Kalman9d_F.setZero();
    Kalman9d_FT.setZero();
    Kalman9d_G.setZero();
    Kalman9d_H.setZero();
    Kalman9d_HT.setZero();
    Kalman9d_Q.setZero();
    Kalman9d_R.setZero();
    Kalman9d_Xe.setZero();
    Kalman9d_Z.setZero();
    Kalman9d_Ze.setZero();
    Kalman9d_K.setZero();
    Kalman9d_P.setIdentity();
    Kalman9d_P_pre.setIdentity();
    Kalman9d_XX1.setZero();
    Kalman9d_ZZ1.setZero();
    Kalman9d_ZZ2.setZero();
    Kalman9d_Z11.setZero();
    Kalman9d_eyeX.setIdentity();

    for (i = 0; i < Kalman9d_StateDimension; ++i)
    {
      Kalman9d_Xe(i, 0) = Par_Kalman9d_Xe[i];
      Kalman9d_R(i, i) = Par_Kalman9d_R[i];
      Kalman9d_H(i, j) = Par_Kalman9d_H[i];
      Kalman9d_F(i, i) = Par_Kalman9d_F[i];
      Kalman9d_G(i, i) = Par_Kalman9d_G[i];
      Kalman9d_Q(i, i) = Par_Kalman9d_Q[i];
    }

    Kalman9d_FT = Kalman9d_F.transpose();
    Kalman9d_HT = Kalman9d_H.transpose();
  }

  void FusionEstimation::Kalman9d_FunEstimation()
  {
    //State Prediction Kalman9d_Xe
    Kalman9d_Xe = Kalman9d_F * Kalman9d_Xe;

    //Covariance Prediction Kalman9d_P_pre
    Kalman9d_P_pre = Kalman9d_F * Kalman9d_P * Kalman9d_FT + Kalman9d_Q;

    //Calculate Kalman Gain Kalman9d_K
    Kalman9d_ZZ1 = Kalman9d_H * Kalman9d_P_pre * Kalman9d_HT + Kalman9d_R;
    Kalman9d_ZZ2 = Kalman9d_ZZ1.inverse();
    Kalman9d_K = Kalman9d_P_pre * Kalman9d_HT * Kalman9d_ZZ2;

    //Update The State Kalman9d_Xe
    Kalman9d_Z11 = Kalman9d_Z - Kalman9d_H * Kalman9d_Xe;
    Kalman9d_Xe = Kalman9d_Xe + Kalman9d_K * Kalman9d_Z11;

    //Covariance Updating Kalman9d_P
    Kalman9d_XX1 = Kalman9d_eyeX - Kalman9d_K * Kalman9d_H;
    Kalman9d_P = Kalman9d_XX1 * Kalman9d_P_pre;
  }
}