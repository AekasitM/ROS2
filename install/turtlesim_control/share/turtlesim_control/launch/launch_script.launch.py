from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import ExecuteProcess,DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import yaml, argparse


#spawn_turtle = []
turtle_names = ['Foxy','Noetic','Humble','Iron'] #For Node spawn

def modify_config_namespace(path:str,new_path:str,namespace:str): #For load .yaml file
    # deserializing a YAML file
    with open(path,'r') as file:
        data = yaml.load(file,Loader=yaml.SafeLoader)

    #creating new content
    new_data = {namespace: data}

    # serializing to a new YAML file
    with open(new_path,'w') as file:
        yaml.dump(new_data,file)

def Create_Node(turtle_name,path):    #Function create Node                                 
    
    controller = Node(          #Node controller
        package='turtlesim_control',
        executable='controller.py',
        parameters=[{'linear_gain': 3.0,
                     'angular_gain': 6.0,
                     'tolerance': 0.1}],  #new_config_path
        namespace=turtle_name
    )

    scheduler = Node(           #Node scheduler
        package='turtlesim_control',
        executable='scheduler.py',
        namespace=turtle_name,
        arguments= ['-f', path] #['-f', full_path] 
        #arguments=['-f', '/home/tuchapong1234/fra501_ws/src/turtlesim_control/via_point/via_point_01.yaml']
    )

    return [controller,scheduler]

def create_pizza_Node():            #create Node for spawn pizza
    pizza_spawner_Foxy = Node(
        package='turtlesim_control',
        executable='pizza_spawner_Foxy.py',
        namespace='Foxy'
    )

    pizza_spawner_Humble = Node(
        package='turtlesim_control',
        executable='pizza_spawner_Humble.py',
        namespace='Humble'
    )

    pizza_spawner_Iron = Node(
        package='turtlesim_control',
        executable='pizza_spawner_Iron.py',
        namespace='Iron'
    )

    pizza_spawner_Noetic = Node(
        package='turtlesim_control',
        executable='pizza_spawner_Noetic.py',
        namespace='Noetic'
    )
    return [pizza_spawner_Foxy,pizza_spawner_Humble,pizza_spawner_Iron,pizza_spawner_Noetic]

    

def Spawn_turtles(turtle_name):     #choose position tospawn turtle
    cmd_spawn = LaunchConfiguration('cmd',default=[
        'ros2 service call /spawn_turtle turtlesim/srv/Spawn "{x: 0.1, y: 0.1, theta: 0.0, name: \'',turtle_name,'\'}"'
    ])

    spawn_turtle =ExecuteProcess(cmd= [[cmd_spawn]], shell=True)
    return spawn_turtle

def Kill_turtles(turtle_name):      #For kill tutle1
    cmd_kill = LaunchConfiguration('cmd',default=[
        'ros2 service call /remove_turtle turtlesim/srv/Kill "{name: \'',turtle_name,'\'}"'
    ])

    kill_turtle =ExecuteProcess(cmd= [[cmd_kill]], shell=True)
    return kill_turtle
    


def render_file(context:LaunchContext,launch_description:LaunchConfiguration,turtle_name:str): #For make Node File by passing turtlename
    
    control_pkg = get_package_share_directory('turtlesim_control')
    full_path = os.path.join(control_pkg,'via_point', 'via_point_'+turtle_name+'.yaml') #'via_point_01.yaml'
    
    config_path = os.path.join(control_pkg,'config', 'controller_config.yaml')

    nodes = Create_Node(turtle_name,full_path)
    launch_description.add_action(nodes[1])

    launch_description.add_action(nodes[0])

    
def render_file_pizza(context:LaunchContext,launch_description:LaunchConfiguration):    #for create pizza node
    node_pizza = create_pizza_Node()
    launch_description.add_action(node_pizza[0])
    launch_description.add_action(node_pizza[1])
    launch_description.add_action(node_pizza[2])
    launch_description.add_action(node_pizza[3])
    

    
def generate_launch_description(): #For call opaqueFunction to launch Node
    launch_description = LaunchDescription()


    turtlesim_plus = Node(
        package='turtlesim_plus',
        executable='turtlesim_plus_node.py',
    )
    
    Opaque_function_Foxy = OpaqueFunction(function=render_file,args=[launch_description,turtle_names[0]])
    Opaque_function_Noetic = OpaqueFunction(function=render_file,args=[launch_description,turtle_names[1]])
    Opaque_function_Humble = OpaqueFunction(function=render_file,args=[launch_description,turtle_names[2]])
    Opaque_function_Iron = OpaqueFunction(function=render_file,args=[launch_description,turtle_names[3]])
    Opaque_function_pizza = OpaqueFunction(function=render_file_pizza,args=[launch_description])

    launch_description.add_action(Opaque_function_pizza)

    launch_description.add_action(Opaque_function_Foxy)
    launch_description.add_action(Opaque_function_Noetic)
    launch_description.add_action(Opaque_function_Humble)
    launch_description.add_action(Opaque_function_Iron)

    launch_description.add_action(Kill_turtles('turtle1'))

    launch_description.add_action(Spawn_turtles(turtle_names[0]))
    launch_description.add_action(Spawn_turtles(turtle_names[1]))
    launch_description.add_action(Spawn_turtles(turtle_names[2]))
    launch_description.add_action(Spawn_turtles(turtle_names[3]))

    launch_description.add_action(turtlesim_plus)

    return launch_description