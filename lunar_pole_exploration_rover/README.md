# lunar_pole_exploration_rover

This package (and associated assets in the simulation repository) provides a Gazebo simulation of a lunar south pole exploration mission inspired by the real NASA VIPER mission.

It includes a realistic environment, the Mons Mouton, where the VIPER rover was planned to land, and a fully controllable rover model. 

The key elements of the simulation is the integration of specifically developped Gazebo plugins. A solar panel plugin and associated battery and power consumption plugins aim to simulate the power generation challenge of the lunar south pole.

This package and associated plugins and models were developped by Robin Baran and Stevedan Ogochukwu Omodolor for the NASA Space ROS Sim Summer Sprint Challenge.

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
docker build -f demos/lunar_pole_exploration_rover/docker/Dockerfile -t lunar_rover_image
```

### Running the docker
Run the following command before running the container:
```bash
xhost +local:docker
```

Run the container by typing:
```bash
docker run --rm -it --name lunar_pole_exploration_rover --network host -e DISPLAY -e TERM -e QT_X11_NO_MITSHM=1 lunar_rover_image
```

Once the container is running, launch the demo by typing the following command:
```bash
ros2 launch lunar_pole_exploration_rover lunar_pole_exploration_rover.launch.py
```

### Controlling the rover
TODO


### Solar panel and power
TODO


## Contribution details
TODO

### Lunar Pole Exploration Rover

#### Sensor suit
#### Power system
#### Control node
##### Motion types

### Mons Mouton World

### Power System Plugins
#### SolarPanelPlugin
#### RtgPlugin
#### RechargeableBatteryPlugin
#### SensorPowerSystemPlugin



