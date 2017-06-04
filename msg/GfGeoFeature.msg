### Geofrenzy GfGeoFeature message
#
# The geographical feature specifies as set of fences along with their
# associated entitlements.
#
int64         gf_class_idx  # Geofrenzy feature class index.
string[]      entitlements  # List of entitlements. Each string is associated
                            # with a dwell message:
                            #   gf_node/class_idx/<str>/dwell
GeoPolygon[]  geometry      # Fence polygon with points
                            # (latitude, longitude, altitude) [deg, deg, meters]
