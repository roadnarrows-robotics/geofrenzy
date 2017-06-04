### Geofrenzy GfGeoFeatureCollection message
#
# The geographical feature collection specifies as set of geographical features.
# Each feature specifies as set of fences along with their associated 
# entitlement.
#
# The validity and timeliness of the data is specified by the access time.
# If zero (0), the data is not valid and the feature set is empty.
# If the data was read from the cache, then the access time is the time of the
# cache from the portal.
#
Header          header      # Standard ROS header
time            access_time # last valid portal access time
GfGeoFeature[]  features    # The collection of features with geographic
                            # centric fences.
