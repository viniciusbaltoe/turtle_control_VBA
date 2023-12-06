from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # Nó para iniciar o nó de navegação
    turtlesim = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim_node',
        output='screen',
        emulate_tty=False,  # Não permite a exibição de mensagens de log no terminal
    )

    control = Node(
        package='turtle_control_VBA',  # Substitua pelo nome do seu pacote
        executable='turtle_control',
        name='turtle_controller',
        output='screen',
        emulate_tty=True,  # Permite a exibição de mensagens de log no terminal
    )

    manager = Node(
        package='turtle_control_VBA',  # Substitua pelo nome do seu pacote
        executable='goal_manager',
        name='goal_manager',
        output='screen',
        emulate_tty=True,  # Permite a exibição de mensagens de log no terminal
    )

    
    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(turtlesim)
    ld.add_action(control)
    ld.add_action(manager)

    return ld