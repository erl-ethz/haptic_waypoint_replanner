# haptic_waypoint_replanner (HWR)
Haptic-based high-level control strategy for aerial physical interaction with flexible structures.


The node needs to be included in a control loop. It has to subscribe to a state estimator (drone's full state can be obtained with a motion capture system or, like in our case, from a tracking camera) and to a force sensor (in our case Medusa F/T sensor from Bota System AG https://www.botasys.com/force-torque-sensors/medusa). It filters the force sensor continuously and publishes position waypoints to a position controller (in our case https://github.com/uzh-rpg/rpg_quadrotor_control). The node can be started by sending a boolean msg (equal to true) to the topic 'haptic_waypoint_replanner/start'.

Parameters can be passed when the node is called in a launch file.
