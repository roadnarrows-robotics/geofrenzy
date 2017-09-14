# File Structure Layout (Tenative)

## Directory Structure
```
collision-avoidance-library/
launch/
gazebo/
msg/
samples/
  fences/
  images/
  shapes/
scripts/
src/
  common/
  sentinels/
  avoidance/
  detection/
  sensors/
  <Node-Src>
srv/
```
### collision-avoidance-library/
Linked git source from https://github.com/01org/collision-avoidance-library

### launch/
All Geofrenzy specific launch files.

### gazebo/
Gazebo simulations. Both ground and UAS simulations are supported.
URDFS, rviz, gazebo, etc. The Laelaps and quadcopter are excellent starting
points.

### msg/
ROS messages files `*.msg`.

### samples/
Directory holds examples and testbed data.

#### fences/
Geofeature feature collections compatible json files.
#### images/
Images directory. Includes censored images.
#### shapes/
Shapes directory. Files here can be converted to Feature Collection json files.

### scripts/
Directory holding script utilities.
The scripts are not necessary ROS.

### src/
Geofrenzy source files.

#### common/
Common shared source.
```
gf_math.[h,cpp]
gf_ros.[h,cpp]
gf_scene.[h,cpp] (new split from gf_math)
etc
```

#### sentinels/
Geofence entitlements sentinel plug-ins for `gf_sensor_relay`.
```
Sentinel.[h,cpp]
CamSentinel.[h,cpp]
MavSentinel.[h,cpp]
StopSentinel.[h,cpp]
etc
```
#### avoidance/
Collision avoidance strategy plug-ins.
Can be compatible with the coav library or Geofrenzy specific.

#### detection/
Collision avoidance detection plug-ins.
Can be compatible with the coav library or Geofrenzy specific.

#### sensors/
Collision avoidance sensor plug-ins. 
Can be compatible with the coav library or Geofrenzy specific.

### Node-Src
Geofrenzy ROS nodes source. These all reside directly under the src/ directory.
Although, we can create sub-dirs, if that is better.

All nodes have a interface .h file and an implemention .cpp file.
The main() is located in a separate .cpp file.

So, for each node, we have at a minimum:
```
gf_<node_class>.h
gf_<node_class>.cpp
gf_<node_class>_node.cpp
```
The `gf_<node_class>_node.cpp` contains the main() and includes the .h file.

Example:
```
gf_map_server.h
gf_map_server.cpp
gf_map_server_node.cpp
```

#### Breach Inhibitor Node
Inihibits crossing of a geofence.
I believe we can wrap this into the new coav node, where the
* fence equals detection
* distance equals the detector
* tiltmax equals the avoidance strategy

Files (if not incorporated into coav node):
```
gf_breach_inhibitor.h
gf_breach_inhibitor.cpp
gf_breach_inhibitor_node.cpp
```

#### Collision Avoidance Node
Real-time ROS Node to avoid collisions.
The main loop:
```
 avoidance->avoid(detector->detect(sensor->read()));
```
Files:
```
gf_coav.h
gf_coav.cpp
gf_coav_node.cpp
```

#### Occupancy Grid Map Node
```
gf_map_server.h
gf_map_server.cpp
gf_map_server_node.cpp
```

#### Geofrenzy Portal Server Node
```
gf_server.h
gf_server.cpp
gf_server_node.cpp
```

#### Geofence Virtual Cloud
```
gf_vcloud.h
gf_vcloud.cpp
gf_vcloud_node.cpp
```

#### Geofrenzy Sensor Relay Node
```
gf_sensor_relay.h
gf_sensor_relay.cpp
gf_sensor_relay_node.cpp
```

### srv/
ROS services files `*.srv`.
