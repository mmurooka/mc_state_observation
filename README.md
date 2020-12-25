# mc_state_observation

This package implements additional state observers for `mc_rtc`.
Some will be considered for inclusion in `mc_rtc` once they have been fully battle tested.

Here is an overview of the various observers implemented:

## Observers

### AttitudeObserver (estimation of IMU orientation)

This observer is directly inspired by the [AttitudeEstimator](https://github.com/isri-aist/hrpsys-state-observation/blob/master/include/hrpsys-state-observation/AttitudeEstimator.h) of [hrpsys-state-observation](https://github.com/isri-aist/hrpsys-state-observation) (that provides an improved replacement for the deprecated [`KalmanFilter`](https://github.com/isri-aist/hrpsys-private/tree/master/KalmanFilter) component of [hrpsys-private](https://github.com/isri-aist/hrpsys-private)). The `AttitudeEstimator` component has been heavily used on `HRP-5P`. 

Configuration options:

```yaml
robot: JVRC1                # robot to observe (defaults to the main robot)
imuSensor: Accelerometer    # sensor providing the IMU measurements (defaults to the first bodysensor)
updateSensor: Accelerometer # name of the sensor in which to write the estimated orientation (defaults to imuSensor)
log_kf: false               # whether to log the kalman filter parameters (default: false)
init_from_control: true     # whether to initialize the kalman filter's orientation from the control robot state (default: true)
KamanFilter:                # configuration of the kalman filter (default values should be reasonable in most cases)
  compensateMode: true
  offset: [0,0,0]           # Apply an orientation offset to the estimation result (rpy or matrix)
  acc_cov: 0.003
  gyr_cov: 1e-10
  ori_acc_cov: 0.003
  lin_acc_cov: 1e-13
  state_cov: 3e-14
  state_init_cov: 1e-8
```


### MocapObserverROS (estimation of the floating base from MOCAP data)

Example configuration (updates main real robot instance from MOCAP data). Note that this requires calibration of the mocap marker wrt to the robot body:
- `Calibrate`: Measures the marker frame to robot body transformation (calibration of the MOCAP markers). This assumes that the initial robot position is very well known in the controller
- `Initialize`: Establishes the link between robot map and mocap origin.

```
ObserverPipelines:
- name: MocapPipeline
  gui: true
  observers:
    - type: Encoder
    - type: MocapObserverROS
      update: true
      config:
        updateRobot: hrp5_p
        marker_tf: HRP5P 
        marker_origin_tf: mocap 
        body: Chest_Link2
```

## Dependencies

- [state-observation](https://github.com/jrl-umi3218/state-observation) > 1.3.3
- [mc_rtc](https://github.com/jrl-umi3218/mc_rtc)
- Eigen3
- Boost
