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
cd prm_slam_ws/codigos_offline 
python3 rviz_binarizado.py
```

Código para obtener la trayectoria con PRM:
-Sin obtaculos
```shell
cd prm_slam_ws/codigos_offline 
python3 Prm_code.py
```

-Con obstaculos
```shell
cd prm_slam_ws/codigos_offline 
python3 Prm_code_obstaculos.py
```

Código para recorrer la trayectoria:
```shell
cd ~/prm_slam_ws/
source devel/setup.bash 
rosrun prm_slam prm_slam_offline.py 
```

## Online


Código para ejecutar la navegación online:
```shell
cd ~/prm_slam_ws/
source devel/setup.bash 
rosrun prm_slam prm_slam_online.py
```
