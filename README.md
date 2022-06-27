# haptic_waypoint_replanner (HWR)
Haptic-based high-level control strategy for aerial physical interaction with flexible structures.


The node needs to be included in a control loop. It has to subscribe to a state estimator (for the drone's full state) and to a F/T sensor (in our case Medusa from Bota System AG https://www.botasys.com/force-torque-sensors/medusa). It filters the F/T sensor continuously and publishes position waypoint to a position controller (in our case https://github.com/uzh-rpg/rpg_quadrotor_control). it has to be started by sending a boolean msg (equal to true) to the topic 'haptic_waypoint_replanner/start'.

Parameters can be passed when the node is called in a launch file.
