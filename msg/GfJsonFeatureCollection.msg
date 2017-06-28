### Geofrenzy GfJsonFeatureCollection message
#
# The Json feature collection specifies as set of geographical features
# specified as a json string.
#
# The validity and timeliness of the data is specified by the access time.
# If zero (0), the data is not valid and the feature set is empty.
# If the data was read from the cache, then the access time is the time of the
# cache from the portal.
#
Header          header      # Standard ROS header
time            access_time # The last valid portal access time.
std_msgs/String features    # The collection of features with geographic
                            # centric fences formatted as a json string.
time            fix_time    # time when last NavFix was generated
