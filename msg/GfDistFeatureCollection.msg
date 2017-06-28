### Geofrenzy GfDistFeatureCollection message
#
# The distance feature collection specifies as set of geographical features.
# Each feature specifies as set of fences along with their associated 
# entitlement. The fences are distances from a reference point.
#
# The validity and timeliness of the data is specified by the access time.
# If zero (0), the data is not valid and the feature set is empty.
# If the data was read from the cache, then the access time is the time of the
# cache from the portal.
#
Header          header      # Standard ROS header
time            access_time # The last valid portal access time.
GfDistFeature[] features    # The collection of features with distance
                            # centric fences.
time            fix_time    # time when last NavFix was generated
