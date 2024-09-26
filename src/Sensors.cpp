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
    LatestFeetEffort = SignalSourceData.contacts[JointDataOrder / 3];

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

    if(CKF6d_Enable)
    {
      if (!CKF6d_IsInitialized)
      {
		    CKF6d_IsInitialized = true;
        Est_CurrentTime = SignalSourceData.header.stamp.toSec();
        CKF6d_Parameter[1] = Est_CurrentTime;
        CKF6d_InitialState[0] = xyz0_Data[0];
        CKF6d_InitialState[1] = xyz0_Data[1];
        CKF6d_InitialState[2] = xyz0_Data[2];
        CKF6d_InitialState[3] = xyz1_Data[0];
        CKF6d_InitialState[4] = xyz1_Data[1];
        CKF6d_InitialState[5] = xyz1_Data[2];
        Hip_CKF6dFunInitiation();
      }

      for (int i = 0; i < 3; i++)
      {
        CKF6d_ParZ(i, 0) = LatestJointAngle[i];
        CKF6d_ParZ(3 + i, 0) = LatestJointVelocity[i];
      }

      CKF6d_Parameter[0] = Est_CurrentTime;
      CKF6d_Parameter[2] = SideSign;

      Hip_CKF6dFunEstimation(CKF6d_ParZ, CKF6d_Output, CKF6d_ParX, CKF6d_ParP, CKF6d_ParQ, CKF6d_ParR, CKF6d_Parameter);

      xyz1_Data[0] = CKF6d_Output(3 , 0);
      xyz1_Data[1] = CKF6d_Output(4 , 0);
      xyz1_Data[2] = CKF6d_Output(5 , 0);

      for (int i = 0; i < 6; i++)
      {
        if (std::isnan(CKF6d_Output(i , 0))||abs(CKF6d_Output(i , 0))>10000)
        {
          Est_ErrorCount++;
          if(!(Est_ErrorCount>=0&&Est_ErrorCount<10000))
          {
            CKF6d_Enable = false;
            std::cout <<"CKF has been disabled..."<< std::endl;
          }
          std::cout <<"CKF estiamtion error number count: "<<Est_ErrorCount<< " with state " << CKF6d_Output.transpose() <<" on sensor "<< SensorErrorCode << std::endl;

          Hip_LegJoint2HipFoot(LatestJointAngle, LatestJointVelocity, xyz0_Data, xyz1_Data, SideSign, Par_HipLength, Par_ThighLength, Par_CalfLength, Par_FootLength);
          for (int i = 0; i < 3; i++)
          {
            CKF6d_InitialState[i] = xyz0_Data[i];
            CKF6d_InitialState[3 + i] = xyz1_Data[i];
          }
          std::cout << "CKF6d Initial State: ";
          for (int i = 0; i < 6; ++i) {
              std::cout << CKF6d_InitialState[i] << " ";
          }
          std::cout << std::endl;
          Hip_CKF6dFunInitiation();
          break;
        }
      }
    }

    ROS_MsgTest.DataCheckC[(SensorErrorCode-201)*3+0] = xyz0_Data[0];
    ROS_MsgTest.DataCheckC[(SensorErrorCode-201)*3+1] = xyz0_Data[1];
    ROS_MsgTest.DataCheckC[(SensorErrorCode-201)*3+2] = xyz0_Data[2];
    ROS_MsgTest.DataCheckD[(SensorErrorCode-201)*3+0] = xyz1_Data[0];
    ROS_MsgTest.DataCheckD[(SensorErrorCode-201)*3+1] = xyz1_Data[1];
    ROS_MsgTest.DataCheckD[(SensorErrorCode-201)*3+2] = xyz1_Data[2];

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

  void SensorHip::Hip_CKF6dFunInitiation()
	{
    int i;
    double Par_PQ = 0.0001;

		CKF6d_CPoints0.resize(CKF6d_StateDimension, CKF6d_StateDimension * 2);
    CKF6d_CPoints0.block(0, 0, CKF6d_StateDimension, CKF6d_StateDimension) = Eigen::MatrixXd::Identity(CKF6d_StateDimension, CKF6d_StateDimension);
    CKF6d_CPoints0.block(0, CKF6d_StateDimension, CKF6d_StateDimension, CKF6d_StateDimension) = -Eigen::MatrixXd::Identity(CKF6d_StateDimension, CKF6d_StateDimension);

		CKF6d_CPoints1.resize(CKF6d_StateDimension, CKF6d_StateDimension * 2);
		CKF6d_CPoints1.setZero();

		CKF6d_CPoints2.resize(CKF6d_StateDimension, CKF6d_StateDimension * 2);
		CKF6d_CPoints2.setZero();

		CKF6d_SqrtP.resize(CKF6d_StateDimension, CKF6d_StateDimension);
		CKF6d_SqrtP.setZero();

		CKF6d_State.resize(CKF6d_StateDimension, 1);
		CKF6d_State.setZero();

		CKF6d_Observation.resize(CKF6d_ObserDimension, 1);
		CKF6d_Observation.setZero();

		CKF6d_ObserError.resize(CKF6d_ObserDimension, 1);
		CKF6d_ObserError.setZero();

		CKF6d_Ppre.resize(CKF6d_StateDimension, CKF6d_StateDimension);
		CKF6d_Ppre.setZero();

		CKF6d_Pzz.resize(CKF6d_ObserDimension, CKF6d_ObserDimension);
		CKF6d_Pzz.setZero();

		CKF6d_Pxz.resize(CKF6d_StateDimension, CKF6d_ObserDimension);
		CKF6d_Pxz.setZero();

		CKF6d_KlmGain.resize(CKF6d_StateDimension, CKF6d_ObserDimension);
		CKF6d_KlmGain.setZero();

		CKF6d_X11.resize(CKF6d_StateDimension, 1);
		CKF6d_X11.setZero();

		CKF6d_XX1.resize(CKF6d_StateDimension, CKF6d_StateDimension);
		CKF6d_XX1.setZero();

		CKF6d_Z11.resize(CKF6d_ObserDimension, 1);
		CKF6d_Z11.setZero();

		CKF6d_LegPar_hip   = Par_HipLength;
		CKF6d_LegPar_thigh = Par_ThighLength;
		CKF6d_LegPar_calf  = Par_CalfLength;
		CKF6d_LegPar_foot  = Par_FootLength;

    CKF6d_Output.resize(CKF6d_StateDimension, 1);
    CKF6d_Output.setZero();

    CKF6d_ParZ.resize(CKF6d_ObserDimension, 1);
    CKF6d_ParX.resize(CKF6d_StateDimension, 1);
    CKF6d_ParP.resize(CKF6d_StateDimension, CKF6d_StateDimension);
    CKF6d_ParQ.resize(CKF6d_StateDimension, CKF6d_StateDimension);
    CKF6d_ParR.resize(CKF6d_ObserDimension, CKF6d_ObserDimension);
    CKF6d_ParZ.setZero();
    CKF6d_ParX.setZero();
    CKF6d_ParP.setIdentity();
    CKF6d_ParP = Par_PQ * CKF6d_ParP;
    CKF6d_ParQ.setIdentity();
    CKF6d_ParQ.diagonal().head(3) << 0.0001, 0.0001, 0.0001;
    CKF6d_ParQ.diagonal().tail(3) << 0.001, 0.001, 0.001;
    CKF6d_ParR.setZero();
    CKF6d_ParR.diagonal().head(3) << 0.01, 0.01, 0.01;
    CKF6d_ParR.diagonal().tail(3) << 1, 1, 1;
    CKF6d_ParX(0, 0) = CKF6d_InitialState[0];
    CKF6d_ParX(1, 0) = CKF6d_InitialState[1];
    CKF6d_ParX(2, 0) = CKF6d_InitialState[2];
    CKF6d_ParX(3, 0) = CKF6d_InitialState[3];
    CKF6d_ParX(4, 0) = CKF6d_InitialState[4];
    CKF6d_ParX(5, 0) = CKF6d_InitialState[5];
  }

	void SensorHip::Hip_CKF6dFunTransition(Eigen::MatrixXd& Input, Eigen::MatrixXd& Output, double(&Parameter)[3])
	{
		int i;
		double SideSign;

		SideSign = Parameter[2];

		Output = Input;
		for (i = 0; i < 3; i++)
			Output(i, 0) = Output(i, 0) + (Parameter[0] - Parameter[1]) * Output(i + 3, 0);

		Output(1, 0) = SideSign * abs(Output(1, 0));
	}

	void SensorHip::Hip_CKF6dFunObservation(Eigen::MatrixXd& Input, Eigen::MatrixXd& Output, double(&Parameter)[3])
	{
		double x = Input(0, 0), y = Input(1, 0), z = Input(2, 0), vx = Input(3, 0), vy = Input(4, 0), vz = Input(5, 0);
		double theta1, theta2, theta3, Vtheta1, Vtheta2, Vtheta3;
		double r, r_bar, y_bar, z_bar, alpha, beta;
		double SideSign;

		SideSign = Parameter[2];

    if((SideSign == 1 && y > SideSign*CKF6d_LegPar_hip)||(SideSign == -1 && y < SideSign*CKF6d_LegPar_hip))
    {
      theta1 = 2 * CKF6d_LegPar_hip * z + sqrt(0.000001 + 4 * CKF6d_LegPar_hip*CKF6d_LegPar_hip * z*z - 4 * (z*z + y*y)*(CKF6d_LegPar_hip*CKF6d_LegPar_hip - y*y));
      theta1 = SideSign * asin( theta1 / (2*(z*z + y*y)) );
    }
    else if((SideSign == 1 && y > 0)||(SideSign == -1 && y < 0))
    {
      theta1 = 2 * CKF6d_LegPar_hip * z + sqrt(0.000001 + 4 * CKF6d_LegPar_hip*CKF6d_LegPar_hip * z*z - 4 * (z*z + y*y)*(CKF6d_LegPar_hip*CKF6d_LegPar_hip - y*y));
      theta1 = SideSign * asin( theta1 / (2*(z*z + y*y)) );
    }
    else
    {
      y=0;
      theta1 = 2 * CKF6d_LegPar_hip * z + sqrt(0.000001 + 4 * CKF6d_LegPar_hip*CKF6d_LegPar_hip * z*z - 4 * (z*z + y*y)*(CKF6d_LegPar_hip*CKF6d_LegPar_hip - y*y));
      theta1 = SideSign * asin( theta1 / (2*(z*z + y*y)) );
    }

		z_bar = z - SideSign * sin(theta1) * CKF6d_LegPar_hip;
		y_bar = y - SideSign * cos(theta1) * CKF6d_LegPar_hip;
		r_bar = sqrt(z_bar*z_bar + y_bar*y_bar);
		r = sqrt(r_bar*r_bar + x*x);
		theta3 = - M_PI + acos((CKF6d_LegPar_thigh*CKF6d_LegPar_thigh + (CKF6d_LegPar_calf + CKF6d_LegPar_foot)*(CKF6d_LegPar_calf + CKF6d_LegPar_foot) - r * r) / (2 * CKF6d_LegPar_thigh * (CKF6d_LegPar_calf + CKF6d_LegPar_foot)));

		alpha = atan2(x, r_bar);
		beta = acos((r*r + CKF6d_LegPar_thigh*CKF6d_LegPar_thigh - (CKF6d_LegPar_calf+CKF6d_LegPar_foot)*(CKF6d_LegPar_calf+CKF6d_LegPar_foot))/(2*r*CKF6d_LegPar_thigh));
		theta2 = alpha + beta;

		double J[3][3];

		J[0][0] = 0;
		J[0][1] = (CKF6d_LegPar_calf + CKF6d_LegPar_foot) * (cos(theta2) * cos(theta3) - sin(theta2) * sin(theta3)) + CKF6d_LegPar_thigh * cos(theta2);
		J[0][2] = (CKF6d_LegPar_calf + CKF6d_LegPar_foot) * (cos(theta2) * cos(theta3) - sin(theta2) * sin(theta3));

		J[1][0] = -CKF6d_LegPar_hip * SideSign * sin(theta1) + (CKF6d_LegPar_calf + CKF6d_LegPar_foot) * cos(theta1) * (cos(theta2) * cos(theta3) - sin(theta2) * sin(theta3)) + CKF6d_LegPar_thigh * cos(theta2) * cos(theta1);
		J[1][0] = -CKF6d_LegPar_hip * SideSign * sin(theta1) + (CKF6d_LegPar_calf + CKF6d_LegPar_foot) * cos(theta1) * (cos(theta2) * cos(theta3) - sin(theta2) * sin(theta3)) + CKF6d_LegPar_thigh * cos(theta2) * cos(theta1);
		J[1][1] = (CKF6d_LegPar_calf + CKF6d_LegPar_foot) * sin(theta1) * (-sin(theta2) * cos(theta3) - cos(theta2) * sin(theta3)) + CKF6d_LegPar_thigh * sin(theta1) * (-sin(theta2));
		J[1][2] = (CKF6d_LegPar_calf + CKF6d_LegPar_foot) * sin(theta1) * (-cos(theta2) * sin(theta3) - sin(theta2) * cos(theta3));

		J[2][0] = CKF6d_LegPar_hip * SideSign * cos(theta1) -	(CKF6d_LegPar_calf + CKF6d_LegPar_foot) * (-sin(theta1) * (cos(theta2) * cos(theta3) - sin(theta2) * sin(theta3))) - CKF6d_LegPar_thigh * cos(theta2) * (-sin(theta1));
		J[2][0] = CKF6d_LegPar_hip * SideSign * cos(theta1) -	(CKF6d_LegPar_calf + CKF6d_LegPar_foot) * (-sin(theta1) * (cos(theta2) * cos(theta3) - sin(theta2) * sin(theta3))) - CKF6d_LegPar_thigh * cos(theta2) * (-sin(theta1));
		J[2][1] = -(CKF6d_LegPar_calf + CKF6d_LegPar_foot) * cos(theta1) * (-sin(theta2) * cos(theta3) - cos(theta2) * sin(theta3)) - CKF6d_LegPar_thigh * cos(theta1) * -sin(theta2);
		J[2][2] = -(CKF6d_LegPar_calf + CKF6d_LegPar_foot) * cos(theta1) * (- cos(theta2) * sin(theta3) - sin(theta2) * cos(theta3));

		double detJ = J[0][0] * (J[1][1] * J[2][2] - J[1][2] * J[2][1]) - J[0][1] * (J[1][0] * J[2][2] - J[1][2] * J[2][0]) + J[0][2] * (J[1][0] * J[2][1] - J[1][1] * J[2][0]);

		double invJ[3][3] = {
		{(J[1][1] * J[2][2] - J[1][2] * J[2][1]) / detJ, -(J[0][1] * J[2][2] - J[0][2] * J[2][1]) / detJ, (J[0][1] * J[1][2] - J[0][2] * J[1][1]) / detJ},
		{-(J[1][0] * J[2][2] - J[1][2] * J[2][0]) / detJ, (J[0][0] * J[2][2] - J[0][2] * J[2][0]) / detJ, -(J[0][0] * J[1][2] - J[0][2] * J[1][0]) / detJ},
		{(J[1][0] * J[2][1] - J[1][1] * J[2][0]) / detJ, -(J[0][0] * J[2][1] - J[0][1] * J[2][0]) / detJ, (J[0][0] * J[1][1] - J[0][1] * J[1][0]) / detJ}
		};

		Vtheta1 = invJ[0][0] * vx + invJ[0][1] * vy + invJ[0][2] * vz;
		Vtheta2 = invJ[1][0] * vx + invJ[1][1] * vy + invJ[1][2] * vz;
		Vtheta3 = invJ[2][0] * vx + invJ[2][1] * vy + invJ[2][2] * vz;

		Output(0, 0) = theta1;
		Output(1, 0) = theta2;
		Output(2, 0) = theta3;
		Output(3, 0) = Vtheta1;
		Output(4, 0) = Vtheta2;
		Output(5, 0) = Vtheta3;
	}

	void SensorHip::Hip_CKF6dFunEstimation(Eigen::MatrixXd& Observation, Eigen::MatrixXd& Output, Eigen::MatrixXd& State, Eigen::MatrixXd& Matrix_P, Eigen::MatrixXd& Matrix_Q, Eigen::MatrixXd& Matrix_R, double(&Parameter)[3])
	{
		int i;

    if( !(abs(Parameter[0] - Parameter[1]) < 0.02) )
    {
      Parameter[1] = Parameter[0];
    }

		// LLT decompose

		SqrtPinfo.compute(Matrix_P);
		CKF6d_SqrtP = SqrtPinfo.matrixL();

		// Calculate cubature points

		CKF6d_CPoints2 = std::sqrt(CKF6d_StateDimension) * CKF6d_SqrtP * CKF6d_CPoints0 + State.replicate(1, 2 * CKF6d_StateDimension);

		for (i = 0; i < 2 * CKF6d_StateDimension; ++i)
		{
			CKF6d_State = CKF6d_CPoints2.col(i);
			Hip_CKF6dFunTransition(CKF6d_State, CKF6d_X11, Parameter);
			CKF6d_CPoints1.col(i) = CKF6d_X11;
		}

		// State prediction

		CKF6d_State.setZero();
		for (i = 0; i < 2 * CKF6d_StateDimension; ++i)
		{
			CKF6d_State = CKF6d_State + CKF6d_CPoints1.col(i);
		}
		CKF6d_State = CKF6d_State / (2 * CKF6d_StateDimension);

		// Prediction error covariance

		CKF6d_Ppre.setZero();
		for (i = 0; i < 2 * CKF6d_StateDimension; ++i)
		{
			CKF6d_Ppre = CKF6d_Ppre + (CKF6d_CPoints1.col(i) - CKF6d_State) *  (CKF6d_CPoints1.col(i) - CKF6d_State).transpose();
		}
		CKF6d_Ppre = CKF6d_Ppre / (2 * CKF6d_StateDimension) + Matrix_Q;

		// Estimation error covariance

		SqrtPinfo.compute(CKF6d_Ppre);
		CKF6d_SqrtP = SqrtPinfo.matrixL();

		// Calculate cubature points

		CKF6d_CPoints2 = std::sqrt(CKF6d_StateDimension) * CKF6d_SqrtP * CKF6d_CPoints0 + CKF6d_State.replicate(1, 2 * CKF6d_StateDimension);

		// Observation prediction, Error covariance, Cross covariance

		CKF6d_Observation.setZero();
		CKF6d_Pzz.setZero();
		CKF6d_Pxz.setZero();

		for (i = 0; i < 2 * CKF6d_StateDimension; ++i)
		{
			CKF6d_X11 = CKF6d_CPoints2.col(i);
			Hip_CKF6dFunObservation(CKF6d_X11, CKF6d_Z11, Parameter);
			CKF6d_Observation = CKF6d_Observation + CKF6d_Z11;
			CKF6d_Pzz = CKF6d_Pzz + CKF6d_Z11 * CKF6d_Z11.transpose();
			CKF6d_Pxz = CKF6d_Pxz + CKF6d_X11 * CKF6d_Z11.transpose();
		}
		CKF6d_Observation = CKF6d_Observation / (2 * CKF6d_StateDimension);
		CKF6d_Pzz = CKF6d_Pzz / (2 * CKF6d_StateDimension) - CKF6d_Observation * CKF6d_Observation.transpose() + Matrix_R;
		CKF6d_Pxz = CKF6d_Pxz / (2 * CKF6d_StateDimension) - CKF6d_State * CKF6d_Observation.transpose();

		// Kalman gain

		CKF6d_ObserError = Observation - CKF6d_Observation;

		CKF6d_KlmGain = CKF6d_Pxz * CKF6d_Pzz.inverse();

		// State update

		State = CKF6d_State + CKF6d_KlmGain * CKF6d_ObserError;

		// Estimation error covariance update

		Matrix_P = CKF6d_Ppre - CKF6d_KlmGain * CKF6d_Pzz * CKF6d_KlmGain.transpose();

		// Time Update
		Parameter[1] = Parameter[0];

		// Estimation output
		Output = State;

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
    // Temp[0] = abs(Est_InputZ[0]) - Zdifference;
    // Temp[1] = std::sqrt(std::pow(distance,2) - std::pow(Temp[0],2));
    // if(std::pow(distance,2) < std::pow(Temp[0],2))
    //   Temp[1] = 0;
    // Temp[2] = Temp[1]; //cos(AngleA) * Temp[1];
    // Temp[3] = sin(AngleA) * Temp[1];
    // FootfallPositionRecord[0] = FootfallPositionRecord[0] + (abs(Temp[2]) - abs(Est_InputX[0])) / abs(Est_InputX[0]) * Est_InputX[0];
    // FootfallPositionRecord[1] = FootfallPositionRecord[1] + (abs(Temp[3]) - abs(Est_InputY[0])) / abs(Est_InputY[0]) * Est_InputY[0];
    std::cout <<"The corrected Footfall position on Z is substracted: "<< Zdifference << std::endl;

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