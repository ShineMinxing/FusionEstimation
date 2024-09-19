# FunsionEstimationNode Instruction


## Overview

The `FunsionEstimator` includes 3 parts:

1. **`FunsionEstimationNode.cpp`** - Handles ROS operations and communication.

2. **`FusionEstimation.cpp` and related headers** - Serve as the fusion estimator's base, providing shared data handling functions for different sensors.

3. **`Sensors.cpp` and related headers** - Designed for different sensors to obtain and uniquely process data.



## Quick Start
1. Correct initial states, urdf file path, link/joint name, signal topics source, node spin frequency in `fusion_estimator/cfg/fusion_estimator_config.yaml`.

2. **catkin build -DCMAKE_BUILD_TYPE=Release** twice.

3. Run the node:

    ```bash
    roslaunch fusion_estimator FusionEstimator.launch
    ```

4. Subscribe to data from the topic `FusionEstimatedState`, which is based on `fusion_estimator/msg/RobotState.msg`.

5. .xml files are stored in `fusion_estimator/MultiplotConfig`, `fusion_estimator/MultiplotConfig/Comprison.xml` is recommanded.

## Quick Demo

after cd src/fusion_estimator

    ```bash
    roscore
    rosrun rqt_multiplot rqt_multiplot
    roslaunch fusion_estimator FusionEstimator.launch
    rosbag play smx_c_1.bag
    ```

## Quick Modify
1. **`FusionEstimationNode.cpp` generally needn't modify**:

	If you want to add or disable some sensors, add or delete relative instances construction and circulation:
	
	```bash
    DataFusion::SensorLidar SensorLidarData{};
    ```
	```bash
    SensorLidarData.Est_Estimation();
    ```
	If you want to reconfigure some parameters, use relative `.OffsetReconfig(ReconfigData)` function. `OffsetReconfig` has its default form, but you can override it. 
	
2. **`FusionEstimation.h`, `FusionEstimation.cpp` generally needn't modify**:

	If you want to output estimation result to diffent topic, correct:
    ```bash
    #include <fusion_estimator/RobotState.h>
	#define FusionEstimator_Package_RobotState fusion_estimator::RobotState
    ```
	```bash
    FusionEstimation::Est_FunOutput()
    ```
	and CMakeLists.txt, package.xml.

3. **Sensor Setup in `Sensors.h`**:

	a. Define subscribed signal topic h file:
	```bash
	#include <fusion_estimator/LowState.h>
	#define FusionEstimator_Package_LowState fusion_estimator::LowState
	#include <nav_msgs/Odometry.h>
	#define FusionEstimator_Package_Odometry nav_msgs::Odometry
    ```
	b. Define your own sensor class imitating `class SensorIMU : public FusionEstimation`.

	b.1. Obtain signal topic name from `fusion_estimator_config.yaml` and bind it to **SignalSourceDataRecieved**:
	
	```bash
	ROS_nh_FusionEstimator.getParam("fusion_estimator/Sensor_Signal_Source_1", SignalSourceTopic);
	ROS_sub_FusionEstimatorIMU = ROS_nh_FusionEstimator.subscribe<FusionEstimator_Package_LowState>(SignalSourceTopic, 1, boost::bind(&SensorIMU::IMUCallback, this, _1));
	ROS_INFO("IMU subscribe from: %s", SignalSourceTopic.c_str());
    ```
	b.2. Give a casual serial number to your sensor like "SensorErrorCode = 101".

	b.3. Give TRUE to array xyz_Available and rpy_Available, according to signal type.
	
	```bash
    int xyz_Available[3] = {0,0,0};  #[position, linear velocity, linear acceleration] ~ avaliable
    int rpy_Available[3] = {0,0,0};  #[orientation, angular velocity, angular acceleration] ~ avaliable
	```

	b.4. Give needed value to other succeeded parameters, succeeded from "Sensors variables" part in `FusionEstimation.h`.

	```bash
	double SensorPosition[3] = {0,0,0};
    Eigen::Quaterniond SensorQuaternion;
    Eigen::Quaterniond SensorQuaternionInv;
    double xyz0_DataOffset[3] = {0,0,0};
    double xyz1_DataOffset[3] = {0,0,0};
    double xyz2_DataOffset[3] = {0,0,0};
    double rpy0_DataOffset[3] = {0,0,0};
    double rpy1_DataOffset[3] = {0,0,0};
    double rpy2_DataOffset[3] = {0,0,0};
    double XYZ_R[9] = {1,1,1,1,1,1,1,1,1}; // sensor XYZ noise covariance diagonal
    double RPY_R[9] = {1,1,1,1,1,1,1,1,1}; // sensor RPY noise covariance diagonal
    ```
	
	b.5. Declare the override of `Est_DataObtain()` to transfer siganl to xyz0_Data/xyz1_Data/xyz2_Data/rpy0_Data/rpy1_Data/rpy2_Data.
	
	b.6. Declare the override of other functions if needed, including:

	```bash
	virtual void OffsetReconfig(double Data[16])
	virtual void Est_DataObtain() {}
    virtual void FurtherHandleXYZ0() {}
    virtual void FurtherHandleXYZ1() {}
    virtual void FurtherHandleXYZ2() {}
    virtual void FurtherHandleRPY0() {}
    virtual void FurtherHandleRPY1() {}
    virtual void FurtherHandleRPY2() {}
    ```

	b.7. Declare extra needed variables and functions for your sensor.

4. **Sensor Setup in `Sensors.cpp`**:

	a. Define `Est_DataObtain()` to transfer siganl to succeeded array xyz0_Data/xyz1_Data/xyz2_Data/rpy0_Data/rpy1_Data/rpy2_Data. rpy0_Data (Orientation) must be world frame based. rpy1_Data (Angular Velocity), rpy2_Data (Angular Acceleration), xyz0_Data (Position), xyz1_Data (Linear Velocity), xyz2_Data (Linear Acceleration) must be sensor frame based.

	b. Define other functions you want to override.

	c. Define other needed functions for your sensors.


# Detailed Explain

## Sensor Usage:

1. **Instantiate Sensor Objects**:

   In `FusionEstimationNode.cpp`, create instances of sensor classes defined in `sensor.h` based on the specific sensor being used. For example: `DataFusion::SensorIMU SensorIMUData{}; `



2. **Perform Fusion Estimation**:

   Within the processing loop, use the `Estimation` function of each sensor to perform the fusion estimation. For example:

   `SensorIMUData.Estimation()`



3. **Publish and Retrieve Estimation Results**:

   Call the `Est_FunOutput` function of any sensor object to publish the estimation results to the `FusionEstimator` topic FusionEstimator and FusionEstimatedState. For example:

	 `SensorIMUData.Est_FunOutput()`


## Sensor Parameter Configuration



### 1. Predefined Parameters:

The parameters for each sensor's derived class are predefined in the ‘Sensors variables’ part within ‘FusionEstimation.h’ and come with default values.



### 2. ‘DataObtain()’ Function:

The ‘DataObtain()’ function is responsible for reading sensor signals. It must be overridden in ‘Sensors.h’ as `void DataObtain() override;`, and the actual implementation should be provided in ‘Sensors.cpp’ based on the specific method used to read the sensor data.



### 3. Signal Availability Flags:

The arrays `xyz_Available[3]` and `rpy_Available[3]` are flags that indicate the availability of sensor signals. The elements correspond to the availability of [position, linear velocity, linear acceleration] and [orientation, angular velocity, angular acceleration], respectively. Corresponding flags should be set to ‘true’ in ‘Sensors.h’ based on the types of signals actually provided by the sensor.



### 4. Sensor Position and Orientation:

The `SensorPosition[3]` array defines the sensor's position in the body coordinate system. `SensorQuaternion` and `SensorQuaternionInv` are the rotation quaternions and their inverses, used to convert from the sensor's coordinate frame to the body coordinate frame. These parameters should be assigned values based on the actual setup.



### 5. Sensor Error Code:

`SensorErrorCode` is a user-defined code for the sensor, which can be set according to preference. It is used for error reporting and debugging in case of estimator crashes.



### 6. Data Storage:

The data obtained by ‘DataObtain()’ must be stored in the corresponding arrays: `xyz0_Data[3]`, `xyz1_Data[3]`, `xyz2_Data[3]`, `rpy0_Data[3]`, `rpy1_Data[3]`, and `rpy2_Data[3]`, which hold the fusion estiamtor's observed position, linear velocity, linear acceleration, orientation, angular velocity, and angular acceleration data, respectively.



### 7. Observation Error Covariance:

`XYZ_R` and `RPY_R` are the diagonal elements of the observation error covariance for the vectors `[x0, x1, x2, y0, y1, y2, z0, z1, z2]` and `[roll0, roll1, roll2, pitch0, pitch1, pitch2, yaw0, yaw1, yaw2]`. The indices 0, 1, and 2 represent the differentiation order, corresponding to position, velocity, and acceleration.



### 8. Data Offset Correction:

The arrays `xyz0_DataOffset[3]` to `rpy2_DataOffset[3]` store the offset correction values for the sensor data in `xyz0_Data[3]` to `rpy2_Data[3]`. Note that deviations related to orientation, such as gravitational acceleration, should not be corrected by `_DataOffset`. Correct it in `DataPreHandle()` after signal coordinate transformation.



### 9. Additional Correction Functions:

`FurtherHandleXYZ0()` to `FurtherHandleRPY2()` are additional correction functions applied to the results of `DataPreHandle()`. If the coordinate transformation results obtained from `DataPreHandle()` are not accurate, these functions can be used for further adjustments, such as subtracting gravitational acceleration.



##  Fusion Estimation Overview



### 1. Global Variables

Global variables defined within the ‘DataFusion’ namespace are shared across all sensor-derived classes, enabling fusion estimation.



###  2. OffsetReconfig

This function, based on ‘EstimatorConfig.cfg’, is designed to correct the ‘DataOffset’ variables of each sensor to compensate for drift. It is defined in the public class and is called within the loop in FusionEstimationNode.cpp to achieve real-time parameter adjustments.



###  3. Est_FunDataHandle

This function processes different types of observation data from various sensors according to their data types:



1. **Position Data:** Transforms the target position data from the sensor's coordinate frame to the world coordinate frame, relative to the body. This includes coordinate frame rotation and sensor position correction. (If the target is not the body itself but a footfall point or landmark, FurtherHandleXYZ0() should be used to compute the body's position based on the target's position relative to the body. The resulting values should be assigned to Est_InputX[0], Est_InputY[0], and Est_InputZ[0].)



2. **Linear Velocity Data:** Converts the sensor's linear velocity data from the sensor's coordinate frame to the world coordinate frame, representing the body's linear velocity. This involves coordinate rotation and sensor angular velocity correction.



3. **Linear Acceleration Data:** Converts the sensor's linear acceleration data from the sensor's coordinate frame to the world coordinate frame, representing the body's linear acceleration. This includes coordinate rotation and centrifugal acceleration correction.



4. **Orientation Data:** Converts the sensor's orientation data from the sensor's coordinate frame to the world coordinate frame, representing the body's orientation. This process involves coordinate rotation.



5. **Angular Velocity Data:** Converts the sensor's angular velocity data from the sensor's coordinate frame to the world coordinate frame, representing the body's angular velocity. This includes coordinate rotation.



6. **Angular Acceleration Data:** Converts the sensor's angular acceleration data from the sensor's coordinate system to the body’s angular acceleration in the world coordinate system. This process includes coordinate rotation.



###  4. Est_FunEstimation

This function begins by using BinaryMark to determine the type of observation data and sets the observation matrix H and observation vector Kalman9d_Z accordingly. The state transition matrix F is configured based on the time interval between the current and previous estimates. The matrices Kalman9d_Xe, Kalman9d_P, Kalman9d_R, and Kalman9d_Q are defined based on the function's inputs. After calling Kalman9d_FunEstimation(), Kalman9d_Xe and Kalman9d_P are updated. Finally, the updated values are transmitted externally via pointers EstimatedState and EstimationPro.



### 5. Est_FunOutput

If the estimated result is required not at the origin of the robot's body coordinate frame but at another location, such as the head for comparing with SLAM, set Est_OutputedPointPosition[3] and use Est_QuaternionTemp3.x(), Est_QuaternionTemp3.y(), and Est_QuaternionTemp3.z(). If the angular velocity and angular acceleration outputs are needed in the body coordinate frame instead of the world coordinate frame, use Est_QuaternionTemp1 and Est_QuaternionTemp2.
