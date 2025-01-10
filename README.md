This is a particle filter to implement robot localization and solve kidnapped robot problem.

### Building Package:

* Move package to your colcon workspace (`src` directory)
* Rebuild colcon workspace 
        
        colcon build    # ----- run from root directory of your colcon workspace

### Running the node:

The localisation node can be tested in the Stage simulator.

        ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -p node_names:=[map_server] -p autostart:=true
        ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=<path_to_map_file>
        ros2 run stage_ros2 stage_ros2 --ros-args -p world_file:=./src/socspioneer/data/meeting.world
        ros2 launch socspioneer keyboard_teleop.launch.py  # ---- run only if you want to move robot using keyboard 
        ros2 run pf_localisation node.py    

For the node to start publishing you must set an initial pose estimate, for example, through RViz2.

### Published Topics:

Running the node successfully will publish the following topics:

* `/amcl_pose` 
* `/estimatedpose`
* `/particlecloud`

All of these can be visualised in RViz by adding the appropriate Views.
