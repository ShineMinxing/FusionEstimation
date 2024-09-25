#include "Sensors.h"

namespace DataFusion
{
  void SensorIMU::Est_DataObtain()
  {
    int i;
    double Temp1, Temp2;
    double Q[4];

    static double LastData1[3], LastData2[3], LastData3[3];

    //Get latest Orientation data: from IMU, maybe based on magnetometer
    Q[0] = SignalSourceData.imu.orientation.w;
    Q[1] = SignalSourceData.imu.orientation.x;
    Q[2] = SignalSourceData.imu.orientation.y;
    Q[3] = SignalSourceData.imu.orientation.z;
    ROS_nh_CurrentTime = SignalSourceData.header.stamp;

    rpy0_Data[0] = atan2(2 * (Q[0] * Q[1] + Q[2] * Q[3]), (1 - 2 * (Q[1] * Q[1] + Q[2] * Q[2])));
    rpy0_Data[1] = asin(2 * (Q[0] * Q[2] - Q[3] * Q[1]));
    rpy0_Data[2] = atan2(2 * (Q[0] * Q[3] + Q[1] * Q[2]), (1 - 2 * (Q[2] * Q[2] + Q[3] * Q[3])));

    Temp1 = rpy0_Data[0] * cos(rpy0_Data[2]) - rpy0_Data[1] * sin(rpy0_Data[2]);
    Temp2 = rpy0_Data[1] * cos(rpy0_Data[2]) + rpy0_Data[0] * sin(rpy0_Data[2]);

    rpy0_Data[0] = Temp1;
    rpy0_Data[1] = Temp2;

    //Get latest AngleVelocity data: from IMU,
    rpy1_Data[0] = SignalSourceData.imu.angular_velocity.x;
    rpy1_Data[1] = SignalSourceData.imu.angular_velocity.y;
    rpy1_Data[2] = SignalSourceData.imu.angular_velocity.z;

    //Get latest Acceleration data: from IMU,
    xyz2_Data[0] = SignalSourceData.imu.linear_acceleration.x;
    xyz2_Data[1] = - SignalSourceData.imu.linear_acceleration.y;
    xyz2_Data[2] = - SignalSourceData.imu.linear_acceleration.z;

    //Data check
    for (i = 0; i < 3; i++)
    {
      if (rpy0_Data[i] != LastData1[i]&&!std::isnan(rpy0_Data[i]))
      {
        LastData1[i] = rpy0_Data[i];
        EstimationFlag = 1;
      }
      else if(std::isnan(rpy0_Data[i]))
      {
        EstimationFlag = 0;
        break;
      }
      if (rpy1_Data[i] != LastData2[i]&&!std::isnan(rpy1_Data[i]))
      {
        LastData2[i] = rpy1_Data[i];
        EstimationFlag = 1;
      }
      else if(std::isnan(rpy1_Data[i]))
      {
        EstimationFlag = 0;
        break;
      }
      if (xyz2_Data[i] != LastData3[i]&&!std::isnan(xyz2_Data[i]))
      {
        LastData3[i] = xyz2_Data[i];
        EstimationFlag = 1;
      }
      else if(std::isnan(xyz2_Data[i]))
      {
        EstimationFlag = 0;
        break;
      }
    }
  }

  void SensorIMU::FurtherHandleXYZ2()
  {
    Est_InputZ[2] += 9.8;
  }

  void SensorIMU::FurtherHandleRPY0()
  {
    static int Est_OriRollTurningTimesRecord = 0, Est_OriYawTurningTimesRecord = 0, Est_OriPitchTurningTimesRecord = 0;

    if (sin(Est_OrientationRecord[0]) >= 0 && Est_AdjustedObservation[0] < -0.9 * M_PI)
      Est_OriRollTurningTimesRecord += 1;
    if (sin(Est_OrientationRecord[0]) < 0 && Est_AdjustedObservation[0] > 0.9 * M_PI)
      Est_OriRollTurningTimesRecord -= 1;
    Est_OrientationRecord[0] = Est_AdjustedObservation[0];

    if (sin(Est_OrientationRecord[1]) >= 0 && Est_AdjustedObservation[1] < -0.9 * M_PI)
      Est_OriPitchTurningTimesRecord += 1;
    if (sin(Est_OrientationRecord[1]) < 0 && Est_AdjustedObservation[1] > 0.9 * M_PI)
      Est_OriPitchTurningTimesRecord -= 1;
    Est_OrientationRecord[1] = Est_AdjustedObservation[1];

    if (sin(Est_OrientationRecord[2]) >= 0 && Est_AdjustedObservation[2] < -0.9 * M_PI)
      Est_OriYawTurningTimesRecord += 1;
    if (sin(Est_OrientationRecord[2]) < 0 && Est_AdjustedObservation[2]  > 0.9 * M_PI)
      Est_OriYawTurningTimesRecord -= 1;
    Est_OrientationRecord[2] = Est_AdjustedObservation[2];

    Est_InputRoll[0] = (Est_AdjustedObservation[0] + Est_OriRollTurningTimesRecord * 2 * M_PI);
    Est_InputPitch[0] = (Est_AdjustedObservation[1] + Est_OriPitchTurningTimesRecord * 2 * M_PI);
    Est_InputYaw[0] = (Est_AdjustedObservation[2] + Est_OriYawTurningTimesRecord * 2 * M_PI);
  }

  void SensorHip::Hip_BodyParameterObtain()
  {
    double x,y,z;

    //Obtain parameters from urdf
    urdf::Model model;
    if (!model.initFile(urdfFilePath)) {
      ROS_INFO("Loading %s Hip Parameter from URDF file: %s", Par_HipName.c_str(), urdfFilePath.c_str());
      ROS_WARN("Failed to parse urdf file, check if the path is correct");
      return;
    }

    // base link
    urdf::LinkConstSharedPtr BaseLink = model.getLink(Par_BaseName);

    if (!BaseLink || !BaseLink->collision || BaseLink->collision->geometry->type != urdf::Geometry::BOX)
    {
      ROS_WARN("%s is not found or its collision geometry is not a box.", Par_BaseName.c_str());
      Par_BodyLength = 0.12345;  // Special number for error detect
      Par_BodyWidth  = 0.12345;
      Par_BodyHeight = 0.12345;
    }
    else
    {
      auto box = std::static_pointer_cast<const urdf::Box>(BaseLink->collision->geometry);
      Par_BodyLength = box->dim.x;
      Par_BodyWidth  = box->dim.y;
      Par_BodyHeight = box->dim.z;
    }

    // hip position
    std::string JointName = Par_HipName + Par_HipJointName;

    urdf::JointConstSharedPtr HipJoint = model.getJoint(JointName);

    if (!HipJoint)
    {
      ROS_WARN("%s joint not found.", JointName.c_str());
      Par_HipPosition[0] = Par_BodyLength / 2;
      Par_HipPosition[1] = Par_BodyWidth / 2;
      Par_HipPosition[2] = 0;
    }
    else
    {
      Par_HipPosition[0] = HipJoint->parent_to_joint_origin_transform.position.x;
      Par_HipPosition[1] = HipJoint->parent_to_joint_origin_transform.position.y;
      Par_HipPosition[2] = HipJoint->parent_to_joint_origin_transform.position.z;
    }

    SensorPosition[0] = Par_HipPosition[0];
    SensorPosition[1] = Par_HipPosition[1];
    SensorPosition[2] = Par_HipPosition[2];

    // hip length
    JointName = Par_HipName + Par_HipLengthName;
    urdf::JointConstSharedPtr HipLength = model.getJoint(JointName);
    if (!HipLength) {
      ROS_WARN("%s joint not found.", JointName.c_str());
      Par_HipLength = 0.12345;  // Special number for error detect
    }
    else
    {
      x = HipLength->parent_to_joint_origin_transform.position.x;
      y = HipLength->parent_to_joint_origin_transform.position.y;
      z = HipLength->parent_to_joint_origin_transform.position.z;
      Par_HipLength =  std::sqrt(x*x + y*y + z*z);
    }

    // thigh length
    JointName = Par_HipName + Par_ThighLengthName;
    urdf::JointConstSharedPtr ThighLength = model.getJoint(JointName);
    if (!ThighLength) {
      ROS_WARN("%s joint not found.", JointName.c_str());
      Par_ThighLength = 0.12345;  // Special number for error detect
    }
    else
    {
      x = ThighLength->parent_to_joint_origin_transform.position.x;
      y = ThighLength->parent_to_joint_origin_transform.position.y;
      z = ThighLength->parent_to_joint_origin_transform.position.z;
      Par_ThighLength =  std::sqrt(x*x + y*y + z*z);
    }

    // calf length
    JointName = Par_HipName + Par_ThighLengthName;
    urdf::JointConstSharedPtr CalfLength = model.getJoint(JointName);
    if (!CalfLength) {
      ROS_WARN("%s joint not found.", JointName.c_str());
      Par_CalfLength = 0.12345;  // Special number for error detect
    }
    else
    {
      x = CalfLength->parent_to_joint_origin_transform.position.x;
      y = CalfLength->parent_to_joint_origin_transform.position.y;
      z = CalfLength->parent_to_joint_origin_transform.position.z;
      Par_CalfLength =  std::sqrt(x*x + y*y + z*z);
    }

    //foot length
    JointName = Par_HipName + Par_FootLengthName;
    urdf::LinkConstSharedPtr FootLength = model.getLink(JointName);
    if (!FootLength || !FootLength->collision || FootLength->collision->geometry->type != urdf::Geometry::SPHERE) {
      ROS_WARN("%s joint not found or its collision geometry is not a sphere.", JointName.c_str());
      Par_FootLength = 0;
    }
    else
    {
      auto RF_FOOT = std::static_pointer_cast<const urdf::Sphere>(FootLength->collision->geometry);
      Par_FootLength = RF_FOOT->radius;
    }

    ROS_INFO("The position of %s hip are %.2f, %.2f, %.2f, length of hip, thigh, calf, foot are %.2f, %.2f, %.2f, %.2f", Par_HipName.c_str(), Par_HipPosition[0], Par_HipPosition[1], Par_HipPosition[2], Par_HipLength, Par_ThighLength, Par_CalfLength, Par_FootLength);
  }

  void SensorHip::Est_DataObtain()
  {
    int i;
    static double LastData1[3], LastData2[3];

    ROS_nh_CurrentTime = SignalSourceData.header.stamp;
    //Get latest FeetEffort data, in the order of rightfront leftfront leftback rightback legs
    LatestFeetEffort = SignalSourceData.foot_force[JointDataOrder / 3];

    //Get Joint Angle data, in the order of rightfront leftfront leftback rightback legs
    for (i = 0; i < 3; i++)
    {
      LatestJointAngle[i] = SignalSourceData.joint_states[JointDataOrder + i].q;
      Est_JointAngle[JointDataOrder + i] = LatestJointAngle[i];
    }

    //Get Joint Velocity data, in the order of rightfront leftfront leftback rightback legs
    for (i = 0; i < 3; i++)
    {
      LatestJointVelocity[i] = SignalSourceData.joint_states[JointDataOrder + i].dq;
      Est_JointAngleVelocity[JointDataOrder + i] = LatestJointVelocity[i];
    }

    //Data check
    EstimationFlag = false;
    FootIsOnGround = false;
    for (i = 0; i < 3; i++)
    {
      if (LatestJointAngle[i] != LastData1[i]&&!std::isnan(LatestJointAngle[i]))
      {
        LastData1[i] = LatestJointAngle[i];
        EstimationFlag = true;
      }
      else if(std::isnan(LatestJointAngle[i]))
      {
        EstimationFlag = false;
        break;
      }

      if (LatestJointVelocity[i] != LastData2[i]&&!std::isnan(LatestJointVelocity[i]))
      {
        LastData2[i] = LatestJointVelocity[i];
        EstimationFlag = true;
      }
      else if(std::isnan(LatestJointVelocity[i]))
      {
        EstimationFlag = false;
        break;
      }
    }

    //Obtain foot hip relative position and velocity
    if(EstimationFlag)
      Hip_FootHipPosition();

    //Detect the moment of foot falling on the ground
    if(LatestFeetEffort >= FootEffortThreshold)
    {
      Est_FeetIsContact[JointDataOrder / 3] = 1;
      FootIsOnGround = true;
    }
    else
    {
      Est_FeetIsContact[JointDataOrder / 3] = 0;
      FootIsOnGround = false;
    }

    if(FootIsOnGround && !FootWasOnGround) //1 vs 0
      FootLanding = true;
    else
      FootLanding = false;

    FootWasOnGround = FootIsOnGround;

    //Estimation flag
    if(EstimationFlag && FootIsOnGround)
      EstimationFlag = true;
    else
      EstimationFlag = false;

  }

  void SensorHip::Hip_FootHipPosition()
  {
    double SideSign;

    //Calculate relative position and velocity between hip and foot
    if (Par_HipName == "RF" || Par_HipName == "RH")
      SideSign = -1;
    else
      SideSign = 1;

    Hip_LegJoint2HipFoot(LatestJointAngle, LatestJointVelocity, xyz0_Data, xyz1_Data, SideSign, Par_HipLength, Par_ThighLength, Par_CalfLength, Par_FootLength);

    ROS_MsgTest.DataCheckA[(SensorErrorCode-201)*3+0] = xyz0_Data[0];
    ROS_MsgTest.DataCheckA[(SensorErrorCode-201)*3+1] = xyz0_Data[1];
    ROS_MsgTest.DataCheckA[(SensorErrorCode-201)*3+2] = xyz0_Data[2];
    ROS_MsgTest.DataCheckB[(SensorErrorCode-201)*3+0] = xyz1_Data[0];
    ROS_MsgTest.DataCheckB[(SensorErrorCode-201)*3+1] = xyz1_Data[1];
    ROS_MsgTest.DataCheckB[(SensorErrorCode-201)*3+2] = xyz1_Data[2];

    xyz0_Data[0] = FootHipCorrectPar[0]*xyz0_Data[0];
    xyz0_Data[1] = FootHipCorrectPar[1]*xyz0_Data[1];
    xyz0_Data[2] = FootHipCorrectPar[2]*xyz0_Data[2];
    xyz1_Data[0] = FootHipCorrectPar[3]*xyz1_Data[0];
    xyz1_Data[1] = FootHipCorrectPar[4]*xyz1_Data[1];
    xyz1_Data[2] = FootHipCorrectPar[5]*xyz1_Data[2];
  }

  void SensorHip::Hip_LegJoint2HipFoot(double *Angle, double *AngleVelocity, double *HipFootPosition, double *HipFootVelocity, double SideSign, double Par_HipLength, double Par_ThighLength, double Par_CalfLength, double Par_FootLength)
  {
    double s1, s2, s3, c1, c2, c3, c23, s23, dq1, dq2, dq3;

    s1 = sin(Angle[0]);
    s2 = sin(Angle[1]);
    s3 = sin(Angle[2]);
    c1 = cos(Angle[0]);
    c2 = cos(Angle[1]);
    c3 = cos(Angle[2]);
    dq1 = AngleVelocity[0];
    dq2 = AngleVelocity[1];
    dq3 = AngleVelocity[2];
    c23 = c2 * c3 - s2 * s3;
    s23 = s2 * c3 + c2 * s3;

    HipFootPosition[0] = (Par_CalfLength + Par_FootLength)  * s23 + Par_ThighLength * s2;
    HipFootPosition[1] = Par_HipLength * SideSign * c1 + (Par_CalfLength + Par_FootLength) * (s1 * c23) + Par_ThighLength * c2 * s1;
    HipFootPosition[2] = Par_HipLength * SideSign * s1 - (Par_CalfLength + Par_FootLength) * (c1 * c23) - Par_ThighLength * c1 * c2;

    HipFootVelocity[0] = ((Par_CalfLength + Par_FootLength) *c23 + Par_ThighLength * c2)*dq2 + ((Par_CalfLength + Par_FootLength) *c23)*dq3;
    HipFootVelocity[1] = ((Par_CalfLength + Par_FootLength) *c1*c23 + Par_ThighLength * c1*c2 - Par_HipLength*SideSign*s1)*dq1\
      + (-(Par_CalfLength + Par_FootLength)  * s1*s23 - Par_ThighLength * s1*s2)*dq2\
      + (-(Par_CalfLength + Par_FootLength)  * s1*s23)*dq3;
    HipFootVelocity[2] = ((Par_CalfLength + Par_FootLength) *s1*c23 + Par_ThighLength * c2*s1 + Par_HipLength*SideSign*c1)*dq1\
      + ((Par_CalfLength + Par_FootLength) *c1*s23 + Par_ThighLength * c1*s2)*dq2\
      + ((Par_CalfLength + Par_FootLength) *c1*s23)*dq3;
  }

  void SensorHip::FurtherHandleXYZ0()
  {
    ROS_MsgTest.Parameter[SensorErrorCode-201] = FootLanding+(1-FootfallPositionRecordIsInitiated);

    if(FootLanding||!FootfallPositionRecordIsInitiated)
    {
      FootfallPositionRecordIsInitiated = 1;
      FootLanding= 0;
      FootfallPositionRecord[0] = Est_StateXYZ(0, 0) + Est_InputX[0];
      FootfallPositionRecord[1] = Est_StateXYZ(3, 0) + Est_InputY[0];
      FootfallPositionRecord[2] = Est_StateXYZ(6, 0) + Est_InputZ[0];

      Hip_FootFallPositionCorrect();
    }

    Est_InputX[0] = FootfallPositionRecord[0] - Est_InputX[0];
    Est_InputY[0] = FootfallPositionRecord[1] - Est_InputY[0];
    Est_InputZ[0] = FootfallPositionRecord[2] - Est_InputZ[0];

    ROS_MsgTest.FeetBasedPosition[(SensorErrorCode-201)*3+0] = Est_InputX[0];
    ROS_MsgTest.FeetBasedPosition[(SensorErrorCode-201)*3+1] = Est_InputY[0];
    ROS_MsgTest.FeetBasedPosition[(SensorErrorCode-201)*3+2] = Est_InputZ[0];

  }

  void SensorHip::Hip_FootFallPositionCorrect()
  {
    double Scope = 0.1, DataAvailablePeriod = 60;
    int i = 0;
    double distance = 0, Zdifference = 0, Temp[4] = {0,0,0,0};
    double AngleA = atan(abs(Est_InputY[0]) / abs(Est_InputX[0]));

    distance = std::sqrt(std::pow(Est_InputX[0],2) + std::pow(Est_InputY[0],2) + std::pow(Est_InputZ[0],2));

    // Discard too old record
    for(i = 0; i < 100; i++)
    {
      if(MapHeightStore[2][i] != 0 && abs(Est_CurrentTime-MapHeightStore[2][i]) > DataAvailablePeriod)
      {
        MapHeightStore[0][i] = 0;
        MapHeightStore[1][i] = 0;
        MapHeightStore[2][i] = 0;
        // std::cout <<"One old step cleared" << std::endl;
      }
    }

    Zdifference = 0;
    // MapHeightStore and claculate z difference
    if(FootfallPositionRecord[2] <= Scope)
    {
      Zdifference = FootfallPositionRecord[2];
    }
    else
    {
      for(i = 0; i < 100; i++)
      {
        if(abs(MapHeightStore[0][i] - FootfallPositionRecord[2]) <= Scope)
        {
          MapHeightStore[1][i] *= exp(- (Est_CurrentTime - MapHeightStore[2][i]) / (DataAvailablePeriod / 3)); //Confidence fading
          MapHeightStore[0][i] = (MapHeightStore[0][i] * MapHeightStore[1][i] + FootfallPositionRecord[2]) / (MapHeightStore[1][i] + 1);
          MapHeightStore[1][i] += 1;
          MapHeightStore[2][i] = Est_CurrentTime;
          Zdifference = FootfallPositionRecord[2] - MapHeightStore[0][i];
          // std::cout <<"New height confidence is " << MapHeightStore[1][i] << std::endl;
          break;
        }
      }
      if(Zdifference == 0)
      {
        for(i = 0; i < 100; i++)
        {
          if(MapHeightStore[2][i] == 0)
          {
            MapHeightStore[0][i] = FootfallPositionRecord[2];
            MapHeightStore[1][i] = 1;
            MapHeightStore[2][i] = Est_CurrentTime;
            break;
          }
        }
      }
    }

    // Modify x y z according to z difference
    FootfallPositionRecord[2] = FootfallPositionRecord[2] - Zdifference;
    // std::cout <<"The corrected Footfall position on Z is substracted: "<< Zdifference << std::endl;

  }

  void SensorHip::FurtherHandleXYZ1()
  {
    ROS_MsgTest.FeetBasedVelocity[(SensorErrorCode-201)*3+0] = Est_InputX[1];
    ROS_MsgTest.FeetBasedVelocity[(SensorErrorCode-201)*3+1] = Est_InputY[1];
    ROS_MsgTest.FeetBasedVelocity[(SensorErrorCode-201)*3+2] = Est_InputZ[1];
  }

  void SensorHip::OffsetReconfig(double Data[16])
  {
  }

  void SensorLidar::Est_DataObtain()
  {
    int i;
    static double LastData1[3] = {0,0,0};

    xyz0_Data[0] = SignalSourceData.pose.pose.position.x;
    xyz0_Data[1] = SignalSourceData.pose.pose.position.y;
    xyz0_Data[2] = SignalSourceData.pose.pose.position.z;

    for(i = 0; i < 3; i++)
    {
      XYZ_R[i] = SignalSourceData.pose.covariance[i * 6 + i];
    }

    //Data check
    for (i = 0; i < 3; i++)
    {
      if (xyz0_Data[i] != LastData1[i]&&!std::isnan(xyz0_Data[i]))
      {
        LastData1[i] = xyz0_Data[i];
        EstimationFlag = 1;
      }
      else if(std::isnan(xyz0_Data[i]))
      {
        EstimationFlag = 0;
        break;
      }
      if (std::isnan(XYZ_R[i]))
      {
        EstimationFlag = 0;
        break;
      }
      else if(XYZ_R[i] < 0.01)
      {
        XYZ_R[i] = 0.01;
      }
      else if(XYZ_R[i] > 10000)
      {
        EstimationFlag = 0;
        break;
      }
    }
  }

  void SensorLidar::FurtherHandleXYZ0()
  {
    static double LastData[3] = {0,0,0};
    static double LastEstimation[3] = {0,0,0};

    Est_InputX[0] = LastEstimation[0] + (xyz0_Data[0] - LastData[0]);
    Est_InputY[0] = LastEstimation[1] + (xyz0_Data[1] - LastData[1]);
    Est_InputZ[0] = LastEstimation[2] + (xyz0_Data[2] - LastData[2]);

    LastData[0] = xyz0_Data[0];
    LastData[1] = xyz0_Data[1];
    LastData[2] = xyz0_Data[2];

    LastEstimation[0] = Est_StateXYZ(0, 0);
    LastEstimation[1] = Est_StateXYZ(3, 0);
    LastEstimation[2] = Est_StateXYZ(6, 0);
  }
}