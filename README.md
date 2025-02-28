# Floodfill_cpp
Floodfill algorithm implemented in ROS environment.

## How to Install

```bash
git clone https://github.com/gelardrc/floodfill_cpp.git
```

## Parameters

- **`BUFFER_RADIUS`** (default = `1`):
  Inflates map objects by this size.
- **`start`** (default = `[34, 25]`):
  Starting position on the map.
- **`goal`** (default = `[29, 44]`):
  Goal position on the map.
- **`animated`** (default = `False`):
  Enables path visualization in RViz while wavefront is being calculated.
- **`map`** (default = `True`):
  If `True`, uses the package's default map.
  To use a custom map, set this to `False` and run the `map_server` package in another terminal.
- ** map_name**
  Set map_name inside map directory - (options : map_name:= "map_test" "exmaple" )

## How to Run

```bash
rosrun floodfill_cpp planner.py 
```

## Examples

```bash
roslaunch wavefront_ros example.launch
```

## To-Do List

- Replace global parameters with local ones to facilitate migration to ROS 2.
- Add `.yaml` configuration files for quicker set.
- Add A* and BA* as backtracking methos.
