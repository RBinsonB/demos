# lunar_pole_exploration_rover

This package (and associated assets in the simulation repository) provides a Gazebo simulation of a lunar south pole exploration mission inspired by the real NASA VIPER mission.

It includes a realistic environment, the Mons Mouton, where the VIPER rover was planned to land, and a fully controllable rover model. 

The key elements of the simulation is the integration of specifically developped Gazebo plugins. A solar panel plugin and associated battery and power consumption plugins aim to simulate the power generation challenge of the lunar south pole.

This package and associated plugins and models were developped by [Robin Baran](https://github.com/RBinsonB) and [Stevedan Ogochukwu Omodolor Omodia](https://github.com/stevedanomodolor) for the NASA Space ROS Sim Summer Sprint Challenge.

    Challenge Name: NASA Space ROS Sim Summer Sprint Challenge
    Team lead Freelancer name: @RBinsonB
    Submission title: Lunar Pole Exploration Rover & Plugins

## Running the demo
### Building the docker
To build the docker image, go to your ROS2 workspace. Make a spaceros workspace if you don't already have one:
```bash
mkdir -p ~spaceros_ws/src
cd ~spaceros_ws
```

Clone the demos and simulation repos. Checkout both repos to the feature branch `feat/lunar_pole_exploration_rover`:
```bash
cd ~spaceros_ws
git clone git@github.com:RBinsonB/demos.git
git checkout feat/lunar_pole_exploration_rover
cd ~spaceros_ws
git clone git@github.com:RBinsonB/simulation.git
git checkout feat/lunar_pole_exploration_rover
```

Build the image:
```bash
cd ~spaceros_ws
docker build -f demos/lunar_pole_exploration_rover/docker/Dockerfile -t lunar_rover_image .
```

### Running the docker
Run the following command before running the container:
```bash
xhost +local:docker
```

Run the container by typing:
```bash
docker run --rm -it --name lunar_pole_exploration_rover \
--network host \
-e DISPLAY \
-e TERM \
-e QT_X11_NO_MITSHM=1 \
lunar_rover_image
```

If you have gpu
```bash
docker run --rm -it --name lunar_pole_exploration_rover \
--network host \
--privileged \
--gpus all \
-e NVIDIA_VISIBLE_DEVICES=all \
-e NVIDIA_DRIVER_CAPABILITIES=graphics \
-e DISPLAY=$DISPLAY \
-e TERM \
-e QT_X11_NO_MITSHM=1 \
-e XAUTHORITY=$XAUTHORITY \
--mount type=bind,source=/tmp/.X11-unix,target=/tmp/.X11-unix \
lunar_rover_image
```

Once the container is running, launch the demo by typing the following command:
```bash
ros2 launch lunar_pole_exploration_rover lunar_pole_exploration_rover.launch.py
```

### Controlling the rover

#### Setup

Open a ne terminal and attach the current running container:
```bash
docker exec -it lunar_pole_exploration_rover bash
```

Source the necessary packaegs. 
```bash
source ~/spaceros/install/setup.bash
```

```bash
source ~/demos_ws/install/setup.bash
```

#### Available commands

```bash
ros2 service call /move_forward std_srvs/srv/Empty
```

Stop the rover

```bash
ros2 service call /move_stop std_srvs/srv/Empty
```

Turn left

```bash
ros2 service call /turn_left std_srvs/srv/Empty
```

Turn right

```bash
ros2 service call /turn_right std_srvs/srv/Empty
```

Rotate in place

```bash
ros2 service call /turn_right std_srvs/srv/Empty
```

Rotate the camera 

```bash
ros2 service call /camera_rotate std_srvs/srv/Empty
```

Center the camera

```bash
ros2 service call /camera_center std_srvs/srv/Empty
```

### Solar panel and power
TODO


## Contribution details
Our contribution is as follows:
* A lunar south pole exploration Gazebo simulation modelled on NASA VIPER mission including:
  * A fully simulated rover model
  * A world and ground model to simulate the Mons Mouton (also called Liebnitz Beta) area.
* A set of Gazebo plugins to simulate power generation and consumption in space Robotics, used in the lunar pole exploration Gazebo simulation. These plugins are as follows:
  * A solar panel plugin to simulate power generation from the sun according to occlusion and angle between the sun and the panel
  * A radioisotope thermal generator plugin to simulate the constant power from a radioisotope thermal generator
  * A modified version of the linear battery plugin that is able to take as charge input the power outputs of the two previous plugins.
  * A sensor power load system plugin to simulate the power drawn by sensors

### Why It Matters
Our motivation and rational for this contribution is firstly based on the renewed global interest for the moon. The polar regions in particular are of high interest due to the high chance of water ice being present in permanently shaded craters. Said water could be used to establish a long lasting human presence on the moon (https://www.weforum.org/agenda/2023/08/space-water-ice-moon-south-pole/).

Unique challenges have to be addressed in this environment and would be interesting to simulate in Space-ROS. In particular the power generation and management:
* Solar power being constantly available outside of shaded area
* Solar power being unavailable while testing the presence of water ice in a shaded area
* Solar panel placement to adapt to the sun low position on the horizon

In addition, power generation and management strategies are a cornerstone of any space mission, and in particular space robotic missons. Many space missions have been saved or got extended beyond their initially planned lifetimes through the careful management of power loads. Deactivating science equipment, sensors and even actuators to reduce consumption. For example, the Opportunity and Spirit rovers extended their mission life well beyond the planned 90 days by deactivating non-essential equipment and reducing communication during dust storms (https://web.archive.org/web/20140902071407/http://www.nasa.gov/mission_pages/mer/mer-20070824.html). Being able to simulate those behaviors and strategies seems of great added value to Space-ROS.


### Lunar Pole Exploration Rover
The rover gazebo model is designed to be as close as posible to the real NASA VIPER rover.

It has four steerable wheels that allow a wide range of motion, including going sideways (it would even be holonomic if it wasn't for the limits on the wheel steer angles). TODO INSERT REF (@RBinsonB)

The rover is equipped with a main navigation camera (actually a pair) mounted on a mast. The camera is able to pan and tilt.

TODO RBinson insert image

The rover is powered by a battery. The battery is charged by three solar panels: one on the left, one on the right and on at the back of the rover. The panels are tilted sideways to face the sun which is low on the horizon when close to the moon south pole. Total power of the solar panel is 450W (TODO RBinsonB insert ref) and therefor each panel was estimated to produce 150W at full capacity (sun hitting horizontally). The solar panels are simulated using a specifically developed plugin, detailed later in the document.

#### Sensor suit
TODO @RBinsonb add more details and ref
It features a similar sensor suit of the real VIPER rover:
- A pair of monochrome cameras for navigation, NavCam, mounted on the rover mast
- A pair of monochrome cameras for the aft blind spots, AftCam, facing back
- An IMU
- Odometry plugin

#### Power system
#### Control node
##### Motion types

#### Rover model API
##### Subscribed Topics
@RBinsonB TODO
##### Published Topics
* **/model/lunar_pole_exploration_rover/left_solar_panel/solar_panel_output** (`ignition_msgs::msg::Float32`) -- Publishes the current output of the left solar panel in watt
* **/model/lunar_pole_exploration_rover/right_solar_panel/solar_panel_output** (`ignition_msgs::msg::Float32`) -- Publishes the current output of the right solar panel in watt
* **/model/lunar_pole_exploration_rover/rear_solar_panel/solar_panel_output** (`ignition_msgs::msg::Float32`) -- Publishes the current output of the rear solar panel in watt
* **/model/lunar_pole_exploration_rover/odometry** (`ignition_msgs::msg::Odometry`) -- Robot odometry
* **/model/lunar_pole_exploration_rover/odometry_with_covariance**(`ignition_msgs::msg::OdometryWithCovariance`) -- Robot odometry
* **/model/lunar_pole_exploration_rover/battery/rechargeable_battery/state**(`ìgnition_msgs::msg::BaterrySate`) -- Publishes the current state of the battery `rechargeable_battery´
* **/model/lunar_pole_exploration_rover/pose**(`ignition_msgs::msg::Pose`) -- Robot estimated pose from odometry
* **aft_cam_left/camera_info** (`ignition_msgs::msg::CameraInfo`) -- AftCam left camera info
* **aft_cam_right/camera_info** (`ignition_msgs::msg::CameraInfo`) -- AftCam right camera info
* **nav_cam_left/camera_info** (`ignition_msgs::msg::CameraInfo`) -- NavCam left camera info
* **nav_cam_right/camera_info** (`ignition_msgs::msg::CameraInfo`) -- NavCam right camera info
* **aft_cam_left/image_raw** (`ignition_msgs::msg::Image`) -- AftCam left camera image
* **aft_cam_right/image_raw** (`ignition_msgs::msg::Image`) -- AftCam right camera image
* **nav_cam_left/image_raw** (`ignition_msgs::msg::Image`) -- NavCam left camera image
* **nav_cam_right/image_raw** (`ignition_msgs::msg::Image`) -- NavCam right camera image

##### Services
@RBinsonB TODO

##### Controllable Joint Interfaces
* mast_head_pivot_joint (`revolute`) -- NavCam pan joint
* mast_camera_joint (`revolute`) -- NavCam tilt joint
* front_left_wheel_joint (`revolute`) -- front left wheel rotation joint
* rear_left_wheel_joint (`revolute`) -- rear left wheel rotation joint
* front_right_wheel_joint (`revolute`) -- front right wheel rotation joint
* rear_right_wheel_joint (`revolute`) -- rear right wheel rotation joint
* front_left_wheel_axle_joint (`revolute`) -- front left wheel steer joint
* rear_left_wheel_axle_joint (`revolute`) -- rear left wheel steer joint
* front_right_wheel_axle_joint (`revolute`) -- front right wheel steer joint
* rear_right_wheel_axle_joint (`revolute`) -- rear right wheel steer joint

### Mons Mouton World
A world and associated Gazebo terrain model for the Mons Mouton mountain located near the lunar south pole (also formaly refered to as Liebnitz Beta).

TODO RBinsonb add image of world

The differentiating characteristics of the Gazebo world are as follows:
* Gravity being set to lunar gravity  (TODO RBinsonb add world tag)
* Atmosphere being neglected (TODO RBinsonb add world tag)
* Sun light entity is placed low on the horizon and direction is set to mimic real sunlight conditions at Mons Mouton (TODO RBinsonb add world tag)

#### Mons Mouton Terrain Model
This model represents the 1sqkm area centered at (TODO RBinsonb add SCS coordinate) and was generated by using DEM data (site20) from the NASA LRO mission: https://pgda.gsfc.nasa.gov/products/78 (Barker, M.K., et al. (2021), Improved LOLA Elevation Maps for South Pole Landing Sites: Error Estimates and Their Impact on Illumination Conditions, Planetary & Space Science, Volume 203, 1 September 2021, 105119,  doi:10.1016/j.pss.2020.105119.).

TODO RBinsonb add picture of map

The model was textured and additional detailed elevation noise at a higher definition than the DEM data was added.

TODO RBinsonb add image of model

### Power System Plugins
#### SolarPanelPlugin
The solar panel plugin allows to simulate solar panel power output depending on the panel orientation relative to the sun and LOS.

* Required elements
  * **link_name** (`str`) -- The solar panel link in the model
  * **nominal_power** (`float`) -- The maximum power supplied by the solar panel when the sun is hitting perdendicular

* Publications
  * **/model/<model_name>/<link_name>/solar_panel_output** (`std_msgs::msg::Float`) -- Publishes the current solar panel output in watt.
 
##### How to setup the plugin
The plugin needs to be attached to a model. The link specified by **<link_name>** is the solar panel.


```XML
<plugin filename="libSolarPanelPlugin.so" name="simulation::SolarPanelPlugin">
    <link_name>rear_solar_panel</link_name>
    <nominal_power>150.0</nominal_power>
 </plugin>
```
        
- The solar panel link needs to have a visual element. The power output is calculated by using the Z-axis of the solar panel link (Z-axis is considered the normal axis to the solar panel).
- **Important**: Some URDF parsers lump together links with fixed joints. To ensure the plugin is working correctly, it might be necessary for the solar panel link to be joined with a dynamic type of joint (and by setting the limits to zero if no motion is required). Example below:

  ```XML
  <joint name="solar_panel_joint" type="revolute">
      <parent link="body"/>
      <child link="solar_panel"/>
      <origin xyz="0.0252 0.7253 0.3829" rpy="1.4673 0 ${-PI}"/>
      <axis xyz="0 0 1"/>
      <limit lower="0.0" upper="0.0" effort="1e10" velocity="0.0"/>
  </joint>
  ```
- The plugin need a sun entity in the world. The sun needs to be a `light` entity of type `directional` with the name `sun`. The element `<pose>` and `<direction>` of the sun are used by the plugin and need to be set properly. Example below:
  ```XML
  <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>-2000 2000 100 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <intensity>1.5</intensity>
      <attenuation>
          <range>100000000</range>
          <constant>1.0</constant>
          <linear>0.0</linear>
          <quadratic>0.001</quadratic>
      </attenuation>
      <direction>0.9 -0.9 -0.2</direction>
  </light>
  ```
- **LOS (Line-Of-Sight)**: If the sun is occulted, no power is supplied by the solar panel plugin. To know if the panel is occulted, the plugin checks the line of sight between the `sun` (by using the `<pose>` element) and the link visual children. The panel solar link needs to have at least one visual element for the plugin to work.
- **Power computation**: The plugin computes the power generated by the panel by checking the angle between the Z-axis of the solar panel link and the `direction` vector of the sun. TODO @RBinsonB add image and ref

#### RadioisotopeThermalGeneratorPlugin
The radioisotope thermal generator plugin allows to simulate an RTG power output. It provides a constant power supply.

* Required elements
  * **link_name** (`str`) -- The RTG link in the model
  * **nominal_power** (`float`) -- The constant power in watt generated by the RTG.

* Publications
  * **/model/<model_name>/<link_name>/radioisotope_thermal_generator_output** (`std_msgs::msg::Float`) -- Publishes the current solar panel output in watt.

##### How to setup the plugin
The plugin needs to be attached to a model. Example below:

```XML
<plugin filename="libRadioisotopeThermalGeneratorPlugin.so" name="simulation::RadioisotopeThermalGeneratorPlugin">
    <link_name>chassis</link_name>
    <nominal_power>100.0</nominal_power>
</plugin>
```

#### RechargeableBatteryPlugin
TODO @stevedanolodomor
#### SensorPowerSystemPlugin
TODO @stevedanolodomor



