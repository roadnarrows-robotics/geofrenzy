### Geofrenzy GfGeoFeature message
#
# The geographical feature specifies as set of fences along with their
# associated entitlements.
#
int64         gf_class_idx  # Geofrenzy feature class index.
int64[]       gf_ent_idx    # List of entitlements. Each index is associated
                            # with a dwell message:
                            #   geofrenzy/<gf_class_idx>/<gf_ent_idx>/dwell
GeoPolygon[]  geometry      # Fence polygon with points
                            # (latitude, longitude, altitude) [deg, deg, meters]
