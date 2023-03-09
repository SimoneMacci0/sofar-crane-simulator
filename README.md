# sofar-crane-simulator
First practical assignment for the SOFAR course. The project consists in a 2D simulation of a crane, developed with ROS2/Python, performing pick and place operations.

## Dependencies

The project targets ROS2 distributions. It has been successfully tested with Galactic and Humble distros (desktop installation).

The only external depencency needed is Arcade library (see [Instructions for Linux install](https://api.arcade.academy/en/latest/install/linux.html))

## Execution

Clone the repository in your workspace and compile as usual.

Run the simulation node with the command:

```ros2 run sofar_crane_simulator crane_sim_node```

## Assignment

You need to implement the following architecture, made up of 4 nodes:
1) The **simulation node** is already provided with this repository. It publishes a single topic (**/controller_setpoint**), which holds the (x,y) goal coordinates for the crane's end-effector. The controller setpoint depends on the current stage of the pick-and-place and a new setpoint is published whenever the simulation node receives messages on the **/next_stage** topic. You can easily inspect the various stages of the pick-and-place inside the simulation node. Finally, the node subscribes to **/motor_x** and **/motor_y** topics, which respectively hold the updated (x,y) end-effector coordinates returned by the control loop.
2) The **controller nodes**, which implement a simple PID controller to control, respectively, the horizontal and vertical motor of the crane. Feel free to implement your own controller or use available ones (e.g., [simple-pid-python](https://pypi.org/project/simple-pid/)).
The controllers receive the target position on the **/cotroller_setpoint** topic and activate the control loop to drive the crane's end-effector, publishing the corresponding updated position on their respective topic. Whenever the target position is reached, the control loop stops and the controller publishes an acknowledgment message on the corresponding topic.
3) The **robot logic node**, which acts has *high-level controller*, guiding the crane through the stages of the pick-and-place. The node waits for both controllers to be idle, then publishes the next stage of the pick-and-place on the given topic. 
Each pick-and-place action begins with the PICK stage (thus you will need to publish a **std_msgs/Int64** message with the data field set to 1) and concludes with the DROP stage, where the current container is delivered and a new one will spawn inside the simulation, increasing your overall score.

![Architecture](sofar_crane_simulator/resource/SOFAR-assign1-architecture.png)

Good luck ;)

## Troubleshooting

As of ROS2 Humble, there is a weird bug which prevents the application from being launched correctly unless the ```import arcade``` statement is placed as first. Whatever changes you do in the code, make sure to always keep that import as first line of code.
