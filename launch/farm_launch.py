from simple_launch import SimpleLauncher, GazeboBridge


sl = SimpleLauncher(use_sim_time = True)

sl.declare_arg('nx', default_value = 1, description = 'how many turbines in X-axis')
sl.declare_arg('ny', default_value = 1, description = 'how many turbines in Y-axis')

sl.declare_arg('x', default_value = 0., description = 'X-position of first turbine')
sl.declare_arg('y', default_value = 0., description = 'Y-position of first turbine')
sl.declare_arg('yaw', default_value = 0., description = 'Yaw of turbines')
sl.declare_arg('scale', default_value = 200., description = 'Distances of turbines')
sl.declare_arg('velocity', default_value = -2., description = 'Velocity of turbines')
sl.declare_arg('nacelle_yaw', default_value = 0., description = 'Preset yaw angle of the nacelle (radians)')
sl.declare_arg('blade_pitch', default_value = 0., description = 'Preset pitch angle of the blades (radians)')

sl.declare_arg('gz_gui', True)

sl.declare_arg('gz', True)
sl.declare_arg('spawn', True)


def launch_setup():

    if sl.arg('gz'):
        gz_args = '-r'
        if not sl.arg('gz_gui'):
            gz_args += ' -s'
        sl.gz_launch(sl.find('floatgen', 'floatgen_world.sdf'), gz_args)

    if sl.arg('spawn'):
        ns = 'farm'
        with sl.group(ns=ns):
            # Generate URDF from xacro (shared between RSP and Gazebo spawn)
            urdf = sl.robot_description('floatgen', 'farm.xacro',
                                        xacro_args=sl.arg_map('x', 'y', 'yaw', 'nx', 'ny', 'scale', 'velocity', 'nacelle_yaw', 'blade_pitch'))

            # run RSP with given parameters
            sl.robot_state_publisher('floatgen', 'farm.xacro',
                                            xacro_args=sl.arg_map('x', 'y', 'yaw', 'nx', 'ny', 'scale', 'velocity', 'nacelle_yaw', 'blade_pitch'))

            # spawn in Gazebo via -param (robot_state_publisher does NOT publish /robot_description topic in Humble)
            sl.node('ros_gz_sim', 'create',
                    arguments=['-param', 'robot_description', '-name', ns],
                    parameters=[{'robot_description': urdf}])

            # joint_state bridge
            gz_js_topic = GazeboBridge.model_prefix(ns)+'/joint_state'
            js_bridge = GazeboBridge(gz_js_topic, 'joint_states', 'sensor_msgs/JointState', GazeboBridge.gz2ros)
            sl.create_gz_bridge([GazeboBridge.clock(), js_bridge], 'turbine_bridge')

    return sl.launch_description()


generate_launch_description = sl.launch_description(launch_setup)
