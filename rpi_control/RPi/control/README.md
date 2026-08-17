As explained in the thesis draft, trajectories are a path and a speed profile.

I implemented a straight line path along with a jerk limited speed profile.

I also implemented paths using parametric curves like splines or circles.

The next step was to use the algorithm described in the paper 
"Jerk-limited time-optimal speed planning for arbitrary paths" 
(https://autopia.car.upm-csic.es/wp-content/papercite-data/pdf/artunedo2022_jerklimitedtime.pdf) 
to allow the robot to move smoothly along those curves by limiting the lateral acceleration.
