# sailboat_auto_navigation
Autopilot for a remote-controlled Sailboat

This work consists in the development of an autonomous navigation
system for a radio-controlled sailboat. The objective is to be able to
navigate following a set of GPS coordinates, planned from predetermined
geographic points by a base station. All work was done using Robot Opera
ting System.

To get a better understanding of this work, please read the PDF report.

My work is based on work in the following repository
https://github.com/Maritime-Robotics-Student-Society/sailing-robot/tree/master/src/sailing_robot/scripts

My modifications & contributions:

The model used to describe how the boat's orientation varies over time, assumes that the boat navigates in a circle due to the interaction between keel and submerged water and the rudder action, that is, it is not based on physics equations , but in a geometric construction. In my project, the variation in orientation is simply the angular speed of the boat around the "Yaw" axis, which is based on physical models involving aerodynamic and hydrodynamic forces.

The equations (based on the laws of kinematics) are exactly the same as those described in my report and in my code. To that code I add Gaussian noise and instead of using the position as a vector, I use the topic Odometry to better reflect the use of GPS.

Southampton College, describes the linear speed based on a polar diagram, it shows the theoretical speed attainable for a specific sailboat at various wind speeds [https://www.nauticed.org/sailing-blog/how-to-read- a-polar-plot-for-sailboats /], that is, it is not based on the laws of physics and to notice that the speed in an instant of time is independent of the speed in the previous instant of time! there is no type of dynamics in the developed simulator.

Simulator described by equations involving, once again, aerodynamic and hydrodynamic forces taking into account how the rudder and sail affect the condition of the boat. 
Implementation of two state machines (a high level and a low level) that switches via dynamic "switches" based on the boat's position relative to the wind (points of sailing).

The simulator of the English institution uses adaptive algorithms that so much mimic the behavior of a sailor, that is, in real time, the algorithm chooses the best maneuvers based on the adjustment of gains (dynamically) (read the PDF article available in the project dossier, for better understanding).

In conclusion:

The simulator developed by the English faculty, described a very crude mathematical model, based on only laws of kinematics, geometric constructions and even concepts specific to the world of sailing navigation. This model of the boat does not take into account any aerodynamic and hydrodynamic forces, therefore unreliable, for example to make identification of parameters (mass, moment of inertia, viscosity, etc.), however, it is a sufficient model to implement and study algorithms of high level and adaptive control that does not depend on a reliable model. In short, the simulator was developed to implement high-level algorithms and an adaptive control architecture and not to describe a complex mathematical model.

The development of my simulator, went the opposite way. I developed a complex simulator, involving laws of physics, however, my control architecture is not adaptive, having my navigation system implemented by only 2 state machines that switch according to the "points of sail".
The objective of the 2 simulators is the same (autonomous navigation between several points of coordinates) but the implementation, the solution and even the work philosophy were different.
