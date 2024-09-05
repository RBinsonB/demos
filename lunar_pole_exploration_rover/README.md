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

Center the camera (TODO) camera center does not work

```bash
ros2 service call /camera_center std_srvs/srv/Empty
```

### Solar panel and power
TODO


## Contribution details
TODO

### Lunar Pole Exploration Rover
The rover gazebo model is designed to be as close as posible to the real NASA VIPER rover.

It has four steerable wheels that allow a wide range of motion, including going sideways (it would even be holonomic if it wasn't for the limits on the wheel steer angles). TODO INSERT REF (@RBinsonB)

The rover is equipped with a main navigation camera (actually a pair) mounted on a mast. The camera is able to pan and tilt.

TODO RBinson insert image

The rover is powered by a battery. The battery is charged by three solar panels: one on the left, one on the right and on at the back of the rover. The panels are tilted sideways to face the sun which is low on the horizon when close to the moon south pole. Total power of the solar panel is 450W (TODO RBinsonB insert ref) and therefor each panel was estimated to produce 150W at full capacity (sun hitting horizontally). The solar panels are simulated using a specifically developed plugin, detailed later in the document.

#### Sensor suit
#### Power system
#### Control node
##### Motion types

### Mons Mouton World

### Power System Plugins
#### SolarPanelPlugin
#### RtgPlugin
#### RechargeableBatteryPlugin
TODO @stevedanolodomor
#### SensorPowerSystemPlugin
TODO @stevedanolodomor



