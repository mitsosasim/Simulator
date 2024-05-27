Code made available by Team Gradient, in the BFMC2024... Was not tested by the organizing team!

<img src="https://github.com/ECC-BFMC/Simulator/blob/main/Picture1.png" width=30% height=30%>

# BFMC Simulator Project

The project contains the entire Gazebo simulator. 
- It can also be used to get a better overview of how the competition is environment looks like
- It can be used in to develop the vehicle state machine
- It can be used to simulate the path planning
- It can be used to set-up a configuration procedure for the real track
- Not suggested for image processing
- Try not to fall in the "continuous simulator developing" trap

Tips on how to install and work on it, can be found in the 


## The documentation is available in details here:
[Documentation](https://bosch-future-mobility-challenge-documentation.readthedocs-hosted.com/data/simulator.html)

## Running

Dependencies:
- `gazebo` 11 (classic)
- `ros2-humble`
- `ros2-humble-gazebo-{ros,msgs,dev,plugins}`
- `poetry`
- `direnv`
- `just`

`direnv` is useful for automating the environment setup with `.envrc` instead of sourcing manually.
The poetry venv is also set up in `.envrc`. The first time `.envrc` is sourced, the file
`./install/local_setup.bash` doesn't exist so it will throw a warning. After running
`colcon build`, this should be fine. The `.envrc` has only been tested on arch linux.
Modify it according to your system, if necessary.
```
$ poetry install
$ colcon build
```

Then exit and enter the directory to source the new environment.
