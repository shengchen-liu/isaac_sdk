# segway_rmp_lawson

[//]: # "Image References"

[image_lawson_iso]: ./images/lawson_iso.jpg "lawson_iso_view"
[image_rmp210]: ./images/rmp210.jpg "rmp210"
[image_kalman_filter]: ./images/Basic_concept_of_Kalman_filtering.svg.png "Kalman Filter Concept"
[]: 

[TOC]



## Hardware configuration

1. (iso view)

   ![alt text][image_lawson_iso]

2. (RMP 210)

   ![alt text][image_rmp210]

   (source: https://www.segway.com/robotics/commercial/rmp210/)

3. (Side view)

## IMU

An **inertial measurement unit** (**IMU**) is an electronic device that measures and reports a body's [specific force](https://en.wikipedia.org/wiki/Specific_force), angular rate, and sometimes the [orientation](https://en.wikipedia.org/wiki/Orientation_(geometry)) of the body, using a combination of [accelerometers](https://en.wikipedia.org/wiki/Accelerometer), [gyroscopes](https://en.wikipedia.org/wiki/Gyroscope), and sometimes [magnetometers](https://en.wikipedia.org/wiki/Magnetometers). IMUs are typically used to maneuver [aircraft](https://en.wikipedia.org/wiki/Aircraft) (an [attitude and heading reference system](https://en.wikipedia.org/wiki/Attitude_and_heading_reference_system)), including [unmanned aerial vehicles](https://en.wikipedia.org/wiki/Unmanned_aerial_vehicle) (UAVs), among many others, and [spacecraft](https://en.wikipedia.org/wiki/Spacecraft), including [satellites](https://en.wikipedia.org/wiki/Satellite) and [landers](https://en.wikipedia.org/wiki/Lander_(spacecraft)). 

(source: https://en.wikipedia.org/wiki/Inertial_measurement_unit)

### LSM6DS33

LSM6DS33 is a 6 DOF imu sensor that combines a digital 3-axis accelerometer and 3-axis gyroscope into a single package.  The sensor provides six independent acceleration and rotation rate readings whose sensitivities can be set in the ranges of ±2 g to ±16 g and ±125°/s to ±2000°/s, available through I²C and SPI interfaces.   Spec details can be found in this document: https://www.st.com/resource/en/datasheet/lsm6ds33.pdf.



## lawson_sim

### lawson_sim_third_person

1. start application

   ```bash
   shengchen@shengchen-HP:~/isaac/isaac-sdk-2019.2-30e21124$ bazel run apps/segway_rmp_lawson/lawson_sim:lawson_sim -- --config="apps/assets/maps/carter_warehouse_p.config.json" --graph="apps/assets/maps/carter_warehouse_p.graph.json"
   ```

   

2. Start Unreal Engine

   ```bash
   shengchen@shengchen-HP:~/isaac/UnrealEngine-IsaacSim_1.2$ ./Engine/Binaries/Linux/UE4Editor IsaacSimProject CarterWarehouse_P -vulkan -isaac_sim_config_json="/home/shengchen/isaac/isaac-sdk-2019.2-30e21124/apps/segway_rmp_lawson/lawson_sim/bridge_config/lawson_full.json"
   ```

3. (video here)

### lawson_sim_joystick

1. start application

   ```bash
   shengchen@shengchen-HP:~/isaac/isaac-sdk-2019.2-30e21124$ bazel run apps/segway_rmp_lawson/lawson_sim:lawson_sim -- --config="apps/assets/maps/carter_warehouse_p.config.json" --graph="apps/assets/maps/carter_warehouse_p.graph.json"
   ```

2. start Unreal Engine

   ```bash
shengchen@shengchen-HP:~/isaac/UnrealEngine-IsaacSim_1.2$ ./Engine/Binaries/Linux/UE4Editor IsaacSimProject CarterWarehouse_P -vulkan -isaac_sim_config_json="/home/shengchen/isaac/isaac-sdk-2019.2-30e21124/apps/segway_rmp_lawson/lawson_sim/bridge_config/lawson_full.json"
   ```
   
3. (video here)

### lawson_sim_mapping

1. start application

   ```bash
   shengchen@shengchen-HP:~/isaac/isaac-sdk-2019.2-30e21124$ bazel run apps/segway_rmp_lawson/lawson_sim:lawson_sim_mapping
   ```

   

2. start Unreal Engine

   ```bash
   shengchen@shengchen-HP:~/isaac/UnrealEngine-IsaacSim_1.2$ ./Engine/Binaries/Linux/UE4Editor IsaacSimProject CarterWarehouse_P -vulkan -isaac_sim_config_json="/home/shengchen/isaac/isaac-sdk-2019.2-30e21124/apps/segway_rmp_lawson/lawson_sim/bridge_config/lawson_full.json"
   ```

3. (video here)

## lawson

1. start application

   ```bash
   shengchen@shengchen-HP:~/isaac/isaac-sdk-2019.2-30e21124$ bazel run apps/segway_rmp_lawson:lawson
   ```

   