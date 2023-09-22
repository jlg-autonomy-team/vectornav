# Copyright (c) 2022, Robotnik Automation
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# 
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""Launch vectornav node and configure parameters"""

from launch import LaunchDescription
from launch.actions import EmitEvent
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from robotnik_common.launch import AddArgumentParser, ExtendedArgument

import lifecycle_msgs.msg


def generate_launch_description():
    """Returns the launch description"""
    ret_ld = LaunchDescription()
    add_to_launcher = AddArgumentParser(ret_ld)

    add_to_launcher.add_arg(
        ExtendedArgument(
            name='robot_id',
            description='Robot ID',
            default_value='robot',
            use_env=True,
            environment='ROBOT_ID'
        )
    )

    add_to_launcher.add_arg(
        ExtendedArgument(
            name='namespace',
            description='Namespace for the node',
            default_value=LaunchConfiguration('robot_id'),
            use_env=True,
            environment='NAMESPACE'
        )
    )

    add_to_launcher.add_arg(
        ExtendedArgument(
            name='port',
            description='Port to connect to the IMU',
            default_value='/dev/ttyUSB0',
            use_env=True,
            environment='IMU_PORT'
        )
    )

    add_to_launcher.add_arg(
        ExtendedArgument(
            name='baudrate',
            description='Baudrate to connect to the IMU',
            default_value='921600',
            use_env=True,
            environment='IMU_BAUDRATE'
        )
    )

    add_to_launcher.add_arg(
        ExtendedArgument(
            name='imu_frame_id',
            description='IMU frame ID',
            default_value=[LaunchConfiguration('robot_id'), '/imu_link'],
            use_env=True,
            environment='IMU_FRAME_ID'
        )
    )

    add_to_launcher.add_arg(
        ExtendedArgument(
            name='async_output_rate',
            description='Async output rate',
            default_value='200',
            use_env=True,
            environment='ASYNC_OUTPUT_RATE'
        )
    )

    add_to_launcher.add_arg(
        ExtendedArgument(
            name='imu_output_rate',
            description='Fixed IMU rate',
            default_value='200',
            use_env=True,
            environment='IMU_OUTPUT_RATE'
        )
    )

    add_to_launcher.add_arg(
        ExtendedArgument(
            name='acc_bias_enable',
            description='Enable accelerometer bias calibration service',
            default_value='false',
            use_env=True,
            environment='ACC_BIAS_ENABLE'
        )
    )

    params = add_to_launcher.process_arg()

    main_node = LifecycleNode(
        package='vectornav',
        executable='vectornav_lifecycle_node',
        namespace=params['namespace'],
        name='imu',
        parameters=[
            {
                'serial_port': params['port'],
                'serial_baud': params['baudrate'],
                'frame_id': params['imu_frame_id'],
                'async_output_rate': params['async_output_rate'],
                'imu_output_rate': params['imu_output_rate'],
                'acc_bias_enable': params['acc_bias_enable'],
                'has_rotation_reference_frame': True,
                'set_acc_bias_seconds': 5.0,
                'rotation_reference_frame': [
                    1.0,  0.0,  0.0,
                    0.0,  1.0,  0.0,
                    0.0,  0.0,  1.0,
                ],
            },
        ],
        output='screen'
    )
    ret_ld.add_action(main_node)
    ret_ld.add_action(
        EmitEvent(
            event=ChangeState(
                lifecycle_node_matcher=matches_action(main_node),
                transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE
            )
        )
    )
    ret_ld.add_action(
        EmitEvent(
            event=ChangeState(
                lifecycle_node_matcher=matches_action(main_node),
                transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE
            )
        )
    )

    return ret_ld
