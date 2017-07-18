# Sample Data

This directory holds sample data to:
 - unit test applications
 - provide user examples

## Subdirectories
### shapes/
The _shapes_ subdirectory holds Json shapes files that can be converted to 
Geofrenzy geographic feature collection Json files with the 
**_latlongmeters.py_** utility.

### fences/
The _fences_ subdirectory contains Geofrenzy compatible feature collection
Json file.
The Geofrenzy portal server **_gf_server_** ROS node can read from a feature
collection file rather than from the portal backend.
