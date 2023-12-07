# ros_alvw_navigation

[![ROS2](https://img.shields.io/badge/ROS2-Humble-green)](https://docs.ros.org/en/humble/index.html)
[![Python](https://img.shields.io/badge/Python-v3.7-blue)](https://www.python.org/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-v22.04.4-red)](https://ubuntu.com/download)
[![Gazebo](https://img.shields.io/badge/Gazebo-v11.10-orange)](https://gazebosim.org/docs)

## Sobre

Este pacote ROS é referente as aulas de laboratório do curso de Robótica da Universidade Federal do Espirito Santo.


## Procedimentos para a instalação do pacote

* No terminal, abra a area de trabalho ROS (ros2_ws), e em seguida o diretório src:
```
cd ~/ros2_ws/src
```
* Faça o clone do pacote do GitHub:
```
git clone git@github.com:viniciusbaltoe/turtle_control_VBA.git
```
* Retorne ao diretório do WorkSpace (ros2_ws) e faça a atualização do colcon:
```
cd ~/ros2_ws && colcon build && source install/setup.bash
```

## Tutoriais de utilização do pacote

### Experimento do Laboratório 5
Neste experimento, uma tartaruga do Turtlesim é enviada para uma determinada coordenada toda as vezes que se atualiza o tópico /goal.

* Launch do Experimento 5:
```
ros2 launch turtle_control_VBA lab5_launch.py 
```
* Exemplo para envio de coordenada:
```
ros2 topic pub /goal geometry_msgs/msg/Pose2D "{
  x: 3.0,
  y: 4.0,
  theta: 1.57
}"
```

### Experimento do Laboratório 6
Neste experimento, uma tartaruga do Turtlesim é enviada para uma determinada coordenada toda as vezes que se atualiza o tópico /goal.

* Configuração inicial necessária para rodar o experimento:
```
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
```
* Launch do Experimento 6 - Turtlesim:
```
ros2 launch turtle_control_VBA lab6_turtlesim_launch.py 
```
* Launch do Experimento 6 - TurtleBot/Gazebo:
```

```
