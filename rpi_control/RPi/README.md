The control folder contains the trajectory generation code, as well as the main.py file that assembles all the parts of the project into the runnable Robot class that handles everything the robot does with the help of a Behavior class.

The core folder contains the parts of the robot.
Some files are used to interface with the hardware, some other parts represent higher level component, some of them handle sensor fusion or the model of the motors.

The experiments folder contains smaller files that gathers data for one specific experiment.
The experiments/importer.py sets up a few useful functions to make the experiments shorter to write.
The data is then moved the rpi_control/data folder for making graphs.
