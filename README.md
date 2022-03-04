# CSC480_Shark_Spotting_Project

UML Diagram Website Used:
  https://app.diagrams.net/
 
### Creating Conda Environment
You will need anaconda or miniconda to create the environment needed to run the simulator. See how to install them [here](https://docs.conda.io/en/latest/miniconda.html)
Run the following line to create the environment you need to run the following line
`conda env create -f sharkspotting.yml`
To activate the envirement use: `conda activate sharkspotting`

## Running the Simulation
To run a simulation you will need to create a driver for the simulator. There is an example driver in "src/simulate.py"
- In the driver you need to
  - Create UAV objects to go in the environment
  - Create Shark objects to go in the environment. A useful method for this is "spawn_sharks" in the util.py file. This is used in "simulate.py"
  - Create a configuration for the environment. 
  - Call `environment.simulate()`. The simulation will then print out many coordinates that show the current postion and current goal for each UAV, and the number of points left in their current path. The simulation ends if a given time limit is exceeded, or if there are no points left in the path of any UAV.
  - The results of `environment.simulate()` will be outputted into vids/sim.mp4. If "vids/sim.mp4" already exists, it will be overridden.
For defining the entities in the environment, it is helpful to look at the constructors for the UAV, Shark, and Environment Classes. Each of these classes takes a configuration dictionary. It is easy to get a sense of which behaviors are configurable through the configuration dictionary in the constructors of UAV.py, Shark.py, Environment.py, Cell_Path.py respectively. There are a lot of defaults used in simulate.py that could be changed by enumeration in the configuration dictionaries.


## Definining new Plans
To define a new plan for the UAV, you need to implement an instance of "Plan.py." There is an example implementation at "Cell_Plan.py." The example implementation has a trivial method for updating paths if a UAV thinks it spots a shark. 

## Defining new Speed Cost
To define a new Speed Cost for the UAV, use the default_speed_cost in the UAV.py file as an example. 

## Defining new Default Shark Detected
To define a new shark_detected behavior for the UAV, use the default_shark_detected method in UAV.py as an example.

## Current Deficiencies
- There is currently no measure of energy usage
- There are too many False Positives reported by the drones. This could lead to unexpected results in the plan class. The frequency of False Positives could be augmented by adjusting the object_detection_performance method in the UAV class.
