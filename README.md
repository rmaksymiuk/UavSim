# CSC480_Shark_Spotting_Project

### Creating Conda Environment
You will need anaconda or miniconda to create the environment needed to run the simulator. See how to install them [here](https://docs.conda.io/en/latest/miniconda.html)
To create the environment you need to run the software, run the following line.
`conda env create -f uavsim.yml`
To activate the envirement use: `conda activate uavsim`

## Running the Simulation
To run a simulation you will need to create a driver for the simulator. There is an example driver in "src/ExSim/BaseDrone_simulate.py"
- In the driver you need to
  - Create UAV objects to go in the environment
  - Create EnvObjects to go in the environment. A useful method for this is "spawn_objects" in the util.py file
  - Create a configuration for the environment. 
  - Call `environment.simulate(plotting=True)`. If plotting is true, UavSim will compile a video and save it to vids/sim.mp4. This video will be overwritten whenever a simulation is run.

For defining the entities in the environment, it is helpful to look at the constructors for the UAV, Shark, and Environment Classes. Each of these classes takes a configuration dictionary. It is easy to get a sense of which behaviors are configurable through the configuration dictionary in the constructors of UAV.py, Shark.py, Environment.py, Cell_Path.py respectively. There are a lot of defaults used in simulate.py that could be changed by enumeration in the configuration dictionaries.


## Definining new Plans
To define a new plan for the UAV, you need to implement an instance of "Plan.py." There are three example implementations the the "src/ExPlan" directory.

## Defining new Speed Cost
To define a new Speed Cost for the UAV, use the default_speed_cost in the UAV.py file as an example. 
speed_cost is an attribute configurable through the UAV constructor.

## Defining new Default Shark Detected
To define a new shark_detected behavior for the UAV, use the default_shark_detected method in UAV.py as an example.
shark_detected is an attribute configurable through the UAV constructor.

