
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # QoS args
    reliability = LaunchConfiguration('reliability', default='reliable')
    history = LaunchConfiguration('history', default='keep_last')
    depth = LaunchConfiguration('depth', default='10')
    deadline_ms = LaunchConfiguration('deadline_ms', default='0')
    latency_budget_ms = LaunchConfiguration('latency_budget_ms', default='0')
    rate_hz = LaunchConfiguration('rate_hz', default='1000')

    # thresholds
    lat_p95_warn_ms = LaunchConfiguration('lat_p95_warn_ms', default='3.0')
    lat_p95_error_ms = LaunchConfiguration('lat_p95_error_ms', default='6.0')
    jitter_warn_us = LaunchConfiguration('jitter_warn_us', default='150.0')
    jitter_error_us = LaunchConfiguration('jitter_error_us', default='400.0')
    loss_warn = LaunchConfiguration('loss_warn', default='0.001')
    loss_error = LaunchConfiguration('loss_error', default='0.01')

    # fault knobs
    fault_drop_n = LaunchConfiguration('fault_drop_n', default='0')
    fault_delay_us = LaunchConfiguration('fault_delay_us', default='0')

    # run info
    run_id = LaunchConfiguration('run_id', default='local')
    out_dir = LaunchConfiguration('out_dir', default='results')

    degrade_after_warn_count = LaunchConfiguration('degrade_after_warn_count', default='3')
    restore_after_ok_count = LaunchConfiguration('restore_after_ok_count', default='5')

    return LaunchDescription([
        # QoS & thresholds args
        DeclareLaunchArgument('reliability'),
        DeclareLaunchArgument('history'),
        DeclareLaunchArgument('depth'),
        DeclareLaunchArgument('deadline_ms'),
        DeclareLaunchArgument('latency_budget_ms'),
        DeclareLaunchArgument('rate_hz'),
        DeclareLaunchArgument('lat_p95_warn_ms'),
        DeclareLaunchArgument('lat_p95_error_ms'),
        DeclareLaunchArgument('jitter_warn_us'),
        DeclareLaunchArgument('jitter_error_us'),
        DeclareLaunchArgument('loss_warn'),
        DeclareLaunchArgument('loss_error'),
        DeclareLaunchArgument('fault_drop_n'),
        DeclareLaunchArgument('fault_delay_us'),
        DeclareLaunchArgument('run_id'),
        DeclareLaunchArgument('out_dir'),
        DeclareLaunchArgument('degrade_after_warn_count'),
        DeclareLaunchArgument('restore_after_ok_count'),

        Node(
            package='qos_rt_lab',
            executable='qos_rt_pub',
            name='qos_rt_pub',
            parameters=[{
                'reliability': reliability,
                'history': history,
                'depth': depth,
                'deadline_ms': deadline_ms,
                'latency_budget_ms': latency_budget_ms,
                'rate_hz': rate_hz,
                'fault_drop_n': fault_drop_n,
                'fault_delay_us': fault_delay_us
            }]
        ),
        Node(
            package='qos_rt_lab',
            executable='qos_rt_sub',
            name='qos_rt_sub',
            parameters=[{
                'reliability': reliability,
                'history': history,
                'depth': depth,
                'deadline_ms': deadline_ms,
                'latency_budget_ms': latency_budget_ms,
                'run_id': run_id,
                'out_dir': out_dir,
                'lat_p95_warn_ms': lat_p95_warn_ms,
                'lat_p95_error_ms': lat_p95_error_ms,
                'jitter_warn_us': jitter_warn_us,
                'jitter_error_us': jitter_error_us,
                'loss_warn': loss_warn,
                'loss_error': loss_error
            }]
        ),
        Node(
            package='qos_rt_lab',
            executable='qos_supervisor',
            name='qos_supervisor',
            parameters=[{
                'degrade_after_warn_count': degrade_after_warn_count,
                'restore_after_ok_count': restore_after_ok_count
            }]
        ),
    ])
