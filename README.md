# `nov_fij_draw_logo` package
ROS 2 C++ package.  [![Static Badge](https://img.shields.io/badge/ROS_2-Humble-34aec5)](https://docs.ros.org/en/humble/)

This ROS2 package consists one of node, called `/draw_logo`. This node controls a turtle in turtlesim to draw a specific pattern (in this case, the logo of Széchenyi István University) based on predefined vectors and coordinates. It moves the turtle using linear and angular velocity commands while also teleporting it to certain locations - using the `/turtlesim/srv/TeleportAbsolute` service - to create a drawing. To control the pen's color, width, and visibility, it uses the `/turtlesim/srv/SetPen` service. It publishes movement commands using the `geometry_msgs/Twist` message type. After the drawing is completed, the node hides the turtle using the `/turtlesim/srv/Kill` service.

## Package, build and run

It is assumed that the workspace is `~/ros2_ws/`.

### Clone the package
``` r
cd ~/ros2_ws/src
```
``` r
git clone https://github.com/gera0719/nov_fij_draw_logo
```

### Build the package
``` r
cd ~/ros2_ws
```
``` r
colcon build --packages-select nov_fij_draw_logo --symlink-install
```
### Run the package
<details>
<summary> Don't forget to source before ROS commands.</summary>

``` bash
source ~/ros2_ws/install/setup.bash
```
</details>

#### Either

``` r
ros2 launch nov_fij_draw_logo logo_launch.py
```

#### or

``` r
ros2 run turtlesim turtlesim_node
```
``` r
ros2 run nov_fij_draw_logo draw_logo
```

## Sketch of the logo on a grid

<div align="center">
  <img src="img/draw_logo_sketch.jpg" alt="Sketch of the logo on a grid" width="401" height="400"/>
</div>

## Expected behaviour

<div align="center" >
  <img src="img/draw_logo_expected_behaviour.gif" alt="palya rajzolas" width="400" height="400"/>
</div>



## Mermaid diagram
``` mermaid
graph TD;

draw([ /draw_node]):::red --> cmd_vel[ /cmd_vel<br/>geometry_msgs/Twist]:::light --> turtle([ /turtlesim]):::red
draw --> teleport[/ /teleport_absolute<br/>turtlesim/srv/TeleportAbsolute /]:::dark --> turtle
draw --> set_pen[/ /set_pen<br/>turtlesim/srv/SetPen /]:::dark --> turtle
draw --> kill[/ /kill<br/>turtlesim/srv/Kill /]:::dark --> turtle


classDef light fill:#34aec5,stroke:#152742,stroke-width:2px,color:#152742  
classDef dark fill:#152742,stroke:#34aec5,stroke-width:2px,color:#34aec5
classDef white fill:#ffffff,stroke:#152742,stroke-width:2px,color:#152742
classDef red fill:#ef4638,stroke:#152742,stroke-width:2px,color:#fff
```
