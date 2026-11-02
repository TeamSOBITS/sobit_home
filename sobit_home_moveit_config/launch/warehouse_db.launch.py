import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Backend registry. SQLite needs no server: the database is a plain file that
# move_group opens directly. MongoDB stores its data in a directory served by a
# mongod wrapper node, so that node has to be running before move_group starts.
WAREHOUSE_BACKENDS = {
    'sqlite': {
        'plugin': 'warehouse_ros_sqlite::DatabaseConnection',
        'needs_server': False,
    },
    'mongo': {
        'plugin': 'warehouse_ros_mongo::MongoDatabaseConnection',
        'needs_server': True,
    },
}


def warehouse_parameters(backend, database_path, host, port):
    """Parameters move_group needs to talk to the warehouse.

    SQLite ignores host/port and takes the database file path in warehouse_host,
    which is why the two backends do not share one parameter layout.
    """
    plugin = WAREHOUSE_BACKENDS[backend]['plugin']
    if backend == 'sqlite':
        return {
            'warehouse_plugin': plugin,
            'warehouse_host': database_path,
            'warehouse_port': 0,
        }
    return {
        'warehouse_plugin': plugin,
        'warehouse_host': host,
        'warehouse_port': int(port),
    }


def _launch_setup(context, *args, **kwargs):
    backend = LaunchConfiguration('warehouse_backend').perform(context).lower()
    if backend not in WAREHOUSE_BACKENDS:
        raise RuntimeError(
            f"unknown warehouse_backend '{backend}', expected one of "
            f"{sorted(WAREHOUSE_BACKENDS)}")

    database_path = LaunchConfiguration('warehouse_database_path').perform(context)
    host = LaunchConfiguration('warehouse_host').perform(context)
    port = LaunchConfiguration('warehouse_port').perform(context)
    robot_name = LaunchConfiguration('robot_name')

    nodes = []
    if WAREHOUSE_BACKENDS[backend]['needs_server']:
        # mongod stores a directory, not a file, so database_path is used as-is.
        os.makedirs(database_path, exist_ok=True)
        nodes.append(
            Node(
                package='warehouse_ros_mongo',
                executable='mongo_wrapper_ros.py',
                name='mongo_wrapper_ros',
                namespace=robot_name,
                parameters=[{
                    'overwrite': False,
                    'database_path': database_path,
                    'warehouse_port': int(port),
                    'warehouse_host': host,
                    'warehouse_exec': 'mongod',
                }],
                output='screen',
            )
        )
    else:
        os.makedirs(os.path.dirname(database_path), exist_ok=True)

    # Populates an empty database with the default demo contents.
    nodes.append(
        Node(
            package='moveit_ros_warehouse',
            executable='moveit_init_demo_warehouse',
            name='moveit_init_demo_warehouse',
            namespace=robot_name,
            parameters=[warehouse_parameters(backend, database_path, host, port)],
            output='screen',
        )
    )
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='robot_name',
            default_value='sobit_home',
            description='Robot name used as namespace'),
        DeclareLaunchArgument(
            name='warehouse_backend',
            default_value='sqlite',
            choices=sorted(WAREHOUSE_BACKENDS),
            description='Warehouse storage backend'),
        DeclareLaunchArgument(
            name='warehouse_database_path',
            default_value=os.path.join(
                os.path.expanduser('~'), '.ros', 'sobit_home_warehouse.sqlite'),
            description='SQLite: database file. MongoDB: data directory'),
        DeclareLaunchArgument(
            name='warehouse_host',
            default_value='localhost',
            description='MongoDB host (ignored by the sqlite backend)'),
        DeclareLaunchArgument(
            name='warehouse_port',
            default_value='33829',
            description='MongoDB port (ignored by the sqlite backend)'),
        OpaqueFunction(function=_launch_setup),
    ])
