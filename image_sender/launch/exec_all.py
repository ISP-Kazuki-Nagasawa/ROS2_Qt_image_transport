import launch
import launch_ros.actions

def generate_launch_description() :
    sender = launch_ros.actions.Node(
        package = 'image_sender', node_executable = 'image_sender', output = 'screen')
    receiver_ros = launch_ros.actions.Node(
        package = 'image_sender', node_executable = 'image_receiver', output = 'screen')
    receiver_qt = launch_ros.actions.Node(
        package = 'qt_image_receiver', node_executable = 'qt_image_receiver', output = 'screen')

    return launch.LaunchDescription([
        sender, receiver_ros, receiver_qt,
        launch.actions.RegisterEventHandler(
            event_handler = launch.event_handlers.OnProcessExit(
                target_action = receiver_qt,
                on_exit = [launch.actions.EmitEvent(event = launch.events.Shutdown())],
            )
        ),
    ])

