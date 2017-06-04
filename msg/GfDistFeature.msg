### Geofrenzy GfDistFeature message
#
# The distance feature specifies as set of fences along with their
# associated entitlements in meters centered on the location (and possibly 
# orientation) of the requestor.
#
int64         gf_class_idx  # Geofrenzy class index.
string[]      entitlements  # List of entitlements. Each string is associated
                            # with a dwell message:
                            #   gf_node/class_idx/<str>/dwell
Polygon64[]   geometry      # Fence polygon with points (x, y, z) [meters]
