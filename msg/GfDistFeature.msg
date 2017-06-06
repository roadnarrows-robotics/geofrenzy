### Geofrenzy GfDistFeature message
#
# The distance feature specifies as set of fences along with their
# associated entitlements in meters centered on the location (and possibly 
# orientation) of the requestor.
#
int64         gf_class_idx  # Geofrenzy class index.
int64[]       gf_ent_idx    # List of entitlements. Each index is associated
                            # with a dwell message:
                            #   geofrenzy/<gf_class_idx>/<gf_ent_idx>/dwell
Polygon64[]   geometry      # Fence polygon with points (x, y, z) [meters]
