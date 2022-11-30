# integrated_AEB_AES
This repository is some of the remaining source code of my master thesis at KTH(http://kth.diva-portal.org/smash/record.jsf?pid=diva2%3A1602269&dswid=-9052) as well as a conference paper (https://ieeexplore.ieee.org/document/9837228). Unfortunately, some of the source codes are lost over the years and long travel during covid pandemic.

There are four file folders in this repository. Descriptions are down below:
## ModelWithCameraRadar
"lane_following","4sensors" and "8sensors_MAY15th" are almost the same. The only difference is the sensor settings that the first uses default one-front-camera and one-radar settings, the second also add extra one-rear camera and one-rear-radar, the last includes four groups of one-camera one-radar combination. However, the last one may not be appropriate for its heavy computation burden in ordinary PC.
Files that are named similarly to LFACC_01_DoubleCurve_DecelTarget.mat are DrivingScenarios predefined. Some of them are from vehicle safety testing standards(ISO or European Standards, could not recall). Some are created by myself to test the behavior of the controlled system.
