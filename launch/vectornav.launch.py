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

import launch
import launch_ros
import os

def read_params(ld : launch.LaunchDescription):
    environment = launch.substitutions.LaunchConfiguration('environment')
    namespace = launch.substitutions.LaunchConfiguration('namespace')
    robot_id = launch.substitutions.LaunchConfiguration('robot_id')
    port = launch.substitutions.LaunchConfiguration('port')
    baudrate = launch.substitutions.LaunchConfiguration('baudrate')
    imu_frame_id = launch.substitutions.LaunchConfiguration('imu_frame_id')
    async_output_rate = launch.substitutions.LaunchConfiguration('async_output_rate')
    imu_output_rate = launch.substitutions.LaunchConfiguration('imu_output_rate')

    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='environment',
        description='Read parameters from environment variables',
        choices=['true', 'false'],
        default_value='true')
    )

    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='namespace',
        description='Namespace for the node',
        default_value='robot')
    )

    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='robot_id',
        description='Robot ID',
        default_value=['robot'])
    )

    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='port',
        description='Port to connect to the IMU',
        default_value='/dev/ttyUSB_IMU')
    )

    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='baudrate',
        description='Baudrate to connect to the IMU',
        default_value='921600')
    )

    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='imu_frame_id',
        description='IMU frame ID',
        default_value=[robot_id, '_imu_link'])
    )

    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='async_output_rate',
        description='Async output rate',
        default_value='200')
    )

    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='imu_output_rate',
        description='Fixed IMU rate',
        default_value='200')
    )

    ret = {}

    if environment == 'false':
        ret = {
            'namespace': namespace,
            'port': port,
            'baudrate': baudrate,
            'imu_frame_id': imu_frame_id,
            'async_output_rate': async_output_rate,
            'imu_output_rate': imu_output_rate
        }
    else:
        if 'NAMESPACE' in os.environ:
            ret['namespace'] = os.environ['NAMESPACE']
        else: ret['namespace'] = namespace

        if 'PORT' in os.environ:
            ret['port'] = os.environ['PORT']
        else: ret['port'] = port

        if 'BAUDRATE' in os.environ:
            ret['baudrate'] = int(os.environ['BAUDRATE'])
        else: ret['baudrate'] = baudrate

        if 'IMU_FRAME_ID' in os.environ:
            ret['imu_frame_id'] = os.environ['IMU_FRAME_ID']
        elif 'ROBOT_ID' in os.environ:
            ret['imu_frame_id'] = os.environ['ROBOT_ID'] + '_imu_link'
        else:
            ret['imu_frame_id'] = imu_frame_id
        
        if 'ASYNC_OUTPUT_RATE' in os.environ:
            ret['async_output_rate'] = int(os.environ['ASYNC_OUTPUT_RATE'])
        else: ret['async_output_rate'] = async_output_rate

        if 'IMU_OUTPUT_RATE' in os.environ:
            ret['imu_output_rate'] = int(os.environ['IMU_OUTPUT_RATE'])
        else: ret['imu_output_rate'] = imu_output_rate

    return ret


def generate_launch_description():
    ld = launch.LaunchDescription()

    params = read_params(ld)

    ld.add_action(launch_ros.actions.Node(
        package='vectornav',
        executable='vnpub',
        name='imu',
        namespace=params['namespace'],
        parameters=[{
            'serial_port': params['port'],
            'serial_baud': params['baudrate'],
            'frame_id': params['imu_frame_id'],
            'async_output_rate': params['async_output_rate'],
            'imu_output_rate': params['imu_output_rate'],
        }]
    ))

    return ld
