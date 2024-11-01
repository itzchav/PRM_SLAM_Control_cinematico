# PRM_SLAM_Control_cinemático
## Offline
Gmapping:

```shell
cd ~/catkin_ws
source ./devel/setup.bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo plano_completo_gmapping_noetic.launch 
```
Teleoperación del robot:
```shell
cd ~/catkin_ws
source ./devel/setup.bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

Comando para la obtención del mapa como imagen:
```shell
cd ~/catkin_ws
source ./devel/setup.bash
rosrun map_server map_saver -f map
```

Código para obtener del mapa como imagen binarizada:
```shell
cd ~/catkin_ws
source ./devel/setup.bash
python3 rviz_binarizado.py
```

