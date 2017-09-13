#!/bin/bash

# boulder
boulder_lat=40.3809009222
boulder_lon=-105.099170504

# others lat/lon here

# hertz
hz=1.0

lat=${boulder_lat}
lon=${boulder_lon}
alt=0.0

echo "Publish /fix with lat=${lat}, lon=${lon} at ${hz} Hz"

rostopic pub -r ${hz} /fix sensor_msgs/NavSatFix "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
status: {status: 0, service: 0}
latitude: ${lat}
longitude: ${lon}
altitude: ${alt}
position_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
position_covariance_type: 0"
