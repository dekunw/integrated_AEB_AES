# integrated_AEB_AES
This repository is the source code of a paper "Integrated Control of Steering and Braking for Effective Collision Avoidance with Autonomous Emergency Braking in Automated Driving" published at the 30th Mediterranean Conference on Control and Automation (MED), 2022 (https://ieeexplore.ieee.org/document/9837228).

To use all of these codes and simulink models, make sure you have installed carsim, ROS extension to matlab and prepare a board like Nvidia Jetson Nano.

There are four file folders in this repository. Descriptions are down below:

## ModelWithCameraRadar
"lane_following","4sensors" and "8sensors_MAY15th" are almost the same. The only difference is the sensor settings that the first uses default one-front-camera and one-radar settings, the second also add extra one-rear camera and one-rear-radar, the last includes four groups of one-camera one-radar combination. However, the last one may not be appropriate for its heavy computation burden in ordinary PC.

Files that are named similarly to ["LFACC_01_DoubleCurve_DecelTarget.mat"](https://github.com/dekunw/integrated_AEB_AES/blob/main/ModelWithCameraRadar/4sensors/lane_following/LFACC_01_DoubleCurve_DecelTarget.mat) are DrivingScenarios predefined. Some of them are from vehicle safety testing standards(ISO or European Standards, could not recall). Some are created by myself to test the behavior of the controlled system.

Open "helpLFSetUp.m" to load parameters needed. Then open "Integrated_controlled_system.slx" then it is ready to run. It should be noted that "LateralDynamicsCT05.m" and "LateralDynamicsDT05" is discrete-time(DT) and continuous-time(CT) dynamics model of the vehicle. "Controller.slx" is needed in Integrated_controlled_system.slx" which is the main contribution of mine.


## [ZOH and some FOH](https://github.com/dekunw/integrated_AEB_AES/tree/main/ZOH)
"ZOH" means the model with fixed sample time discretized by Zero-Order-Hold(ZOH). It was built before implementing First-Order-Hold(FOH). There are several files of great interest.

1. "LateralController1.slx" is a model directly adapted from some official example where an Adaptive MPC block is used.
2. "LateralControllerDIY.slx" replaced the adaptive MPC block with a self-written matlab function as lateral controller. It includes two controllers if you open the file you will see. The top one is ZOH controller while the bottom one is newly-made FOH controller.
3. "IntegratedMPCcontroller5states.slx" includes ZOH model from "LateralControllerDIY.slx" which is in the bottom and also a ZOH integrated controller in the top. The ZOH integrated controller also controls the longitudinal forces input to the vehicle model which also adds an extra state "longitudinal velocity".
4. "ZOHcontroller.slx" is controller block with ROS communication blocks prepared for HiL testing.

## controller_for_carsim
The plant model we used before is from matlab source or made by myself(quite simple model, the same model I used for designing the MPC controller). In real cases, the model on which the controller is designed is far simpler than the dynamics of real plants which cannot predict behaviors of real controlled system in many cases. Therefore, to test the performance of the designed controller, we need experiments carried out on real plants or simulations on more sophisticated plant model. In this regard, carsim is introduced and a more complicated vehicle model is used to test the designed controller.
1.withROS: it includes ROS blocks prepared for HiL testing.
2.withoutROS: it does not include ROS blocks and can be considered as a version of controlled system with vehicle model in matlab replaced by carsim model.

## [FOH and other files](https://github.com/dekunw/integrated_AEB_AES/tree/main/FOH)
This is the final version of the integrated AEB+AES(Autonomous Emergency Steering) controller. There are some files related to Stochastic MPC controller but not all of them.

1. "controller_for_carsim" is a version of controller(ZOH) that does not control longitudinal velocity and is prepared for testing its performance on carsim's vehicle model. (adaptive MPC blocks)
2."controller_for_carsimDIY" replaces the adaptive MPC controller with the self-wrote controller code. The rest is the same as "controller_for_carsim".
3."controller_for_carsimDIYROS" introduces ROS blocks which recieves control inputs calculated from the controller running on Jetson nano and sends states of vehicle model from carsim into ROS master.
4."DIYcontroller" introduces ROS blocks into controller-only model. It will be translated into c++ and deployed on Jetson nano. It recieves states of vehicle model and sends out calculated inputs.
5."ROScontroller" is similar to "DIYcontroller". But its controller has two seperated parts, one lateral MPC controller, one longitudinal P controller.

There are a lot of other models coming from carsim and not created by myself. Be careful to make changes to them.

##Publication
