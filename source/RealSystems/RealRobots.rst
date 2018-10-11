===========
Real Robots
===========
Up to this point we have focused exclusively on robotics as either an abstract mathematical concept or within computer simulation. This section hopes to home in on the more concrete side of robotics: physical implementation. The act of transitioning from the notepad or simulation to a moving, physically-present system is not to be glossed over as a footnote. In the vast majority of cases a control scheme that works on a computer screen falls apart when moved into the real world.

Limitations of simulation:
--------------------------
It is impossible to overstate the importance of simulation to the field of Robotics. Simulation allows us to test new path-planning algorithms from our living rooms and run stochastic experiments hundreds of times within minutes. However, for simulation to be useful as a tool a few concessions must be granted.

Physics models are often reduced and linearized projections of the incredibly-complex real system. A classic example of this is the inverted pendulum, where the non-linear system model is routinely linearized certain set point and very real parameters like link inertia and joint friction are omitted.

Similarly sensors in simulation land are often idealized: your LiDAR can measure from 0.1cm to 100m, or the output of the infrared distance sensor on your simulation bot is not affected by the reflectance of the sensed object. In reality these sensors are not deterministic and will often exhibit strange behavior even in optimal conditions.

Simulations also ignore communications challenges. Communication between a central controller and a motor controller, for example, can take a non-zero amount of time and introduce a delay from issued command to system response. Networking tirelessly often presents throughput and latency challenges, something never seen in simulation. By and large simulation is an incredibly powerful tool that allows us to prove a concept, but it's important to understand that simulation is not a perfect representation of a physical system.

Moving to the real world:
-------------------------
Like many things in life, the first step in taking a project from the drawing board to reality is defining the purpose of the project. For what task is your robot being built? How is your robot to complete this task? What constraints are placed on the physical design of the system? Once a purpose has been decided on and constraints identified discussions of system complexity can take place. How complex must the robot be to complete its task?

A Roomba, for example, is designed exclusively to sweep-vacuum your floor. Such a system does not need a GPU-accelerated stereo-vision SLAM localization system to adequately traverse a room. Early Roombas simply used bump sensors on the exterior perimeter and random motion to produce satisfactory results. However, the aforementioned localization system might be necessary in an eight-legged robot developed for navigating the interior of a destroyed nuclear reactor.

Several practical considerations must be accounted for as well. In simulation we are given infinite control authority over our robotic system; in reality, motors have a hard limit to the amount of torque they can apply to a shaft. Batteries can only supply so much current at once. Drivetrain slop could effect wheel telemetry if calculated from the input motor side. Designing a robot for the challenge at hand is often more art than science, but always starts with a discussion of required capability.

Structure of a 'real' robot:
----------------------------
Discussion of what a robot is or is not can be found elsewhere in this text. For this context we'll look at what constitutes the physical embodiment of our existentially indeterminate robots. Surveying a broad sample of robots, from Roombas to Willow Garage's PR2, it is easy to see that there are many shared architectural elements through the spectrum. The vast majority of robots will have

Locomotion
~~~~~~~~~~
	One of the defining features of mobile robotics is the robot's ability to traverse its environment. This environment might be the floor of your bedroom, the airspace of your gymnasium, or even the walls of your office. Wheels are the most common form of locomotion due to their ease of implementation and extremely low complexity. Systems with articulated legs are very complex and extraordinarily difficult to control intelligently. Such systems have only recently become reasonable for robot locomotion due to increases in low-power, high-output computing.

Actuation
~~~~~~~~~
	In most cases a robot can affect its environment or internal state in some way. This is often accomplished by using actuators of some kind to turn, slide, or extend. For the vast majority of low-cost robotics these actuators are electrically powered, manifesting in the form of hobby servos, DC motors, steppers, or solenoids. As the budget increases high-power electric systems like three-phase AC servomotors or pneumatic pistons are common.

Power system
~~~~~~~~~~~~
	Motors need power to turn. All robots have some form of power distribution system to deliver correct voltage levels and supply current to every piece of hardware on the robot. Most commonly the power source is a battery bank (usually LiPo) that is then regulated down and distributed through a wiring harness. Some bots, like Boston Dynamics' Big Dog contain a small gasoline-powered generator to produce power remotely.

Sensors
~~~~~~~
	In order to make decisions robots must have some way of taking input from their environment. At a base level a sensor does one thing: transform environmental stimulus into a form the robot's computational hardware can understand. A sensor could be as complicated as an RGBD camera returning a point cloud or as simple as a bump switch. In the past decade LiDARs have become the go-to sensor for most terrestrial robotics applications.

Compute hardware
~~~~~~~~~~~~~~~~
	Critical to a robot's functionality is the ability to process incoming information and decide on a course of action. For complex functionality this usually necessitates serious computing horsepower; for such problems low-power, compact versions of desktop computing hardware are available. For most hobby, competition, and research work a dedicated single-board computer running a utilitarian Linux distribution provides enough number-crunching and interconnect capability. For simple robots a microcontroller may be all the processing power necessary. Very complex systems (i.e. DARPA challenge bots) almost exclusively offload the processing of sensor information and decision-making to off-site computing hardware.

Real-time controllers
~~~~~~~~~~~~~~~~~~~~~
	Even though a dedicated onboard computer might provide direction and high-level control of a robot, many tasks associated with robot operation require the singular attention of a sub-system controller. An SBC running a Linux OS does not have the real-time capability necessary to generate the waveforms driving hobby servos, or to keep track of every single tick of a wheel encoder. To handle these tasks dedicated controllers are often used. One example of this is a motor controller that takes in a target rotational speed via UART and monitors a shaft encoder, increasing or decreasing motor current as necessary to maintain a constant shaft speed.
