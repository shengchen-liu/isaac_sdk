# segway_rmp_lawson

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

## simple_robot

1. start application

   ```bash
   shengchen@shengchen-HP:~/isaac/isaac-sdk-2019.2-30e21124$ bazel run apps/segway_rmp_lawson/simple_robot:simple_robot
   ```

   

