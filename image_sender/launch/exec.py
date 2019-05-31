import launch
import launch_ros.actions

def generate_launch_description() :
    sender = launch_ros.actions.Node(
        package = 'image_sender', node_executable = 'image_sender', output = 'screen')
    receiver = launch_ros.actions.Node(
        package = 'qt_image_receiver', node_executable = 'qt_image_receiver', output = 'screen')

    return launch.LaunchDescription([
        sender, receiver,
        launch.actions.RegisterEventHandler(
            event_handler = launch.event_handlers.OnProcessExit(
                target_action = receiver,
                on_exit = [launch.actions.EmitEvent(event = launch.events.Shutdown())],
            )
        ),
    ])

