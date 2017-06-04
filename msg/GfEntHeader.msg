### Geofrenzy GfEntHeader message
#
# The geofrenzy entitlement header contains common identifying properties for
# an entitlement.
#
# The validity and timeliness of the data is specified by the access time.
# If zero (0), the data is not valid and the feature set is empty.
# If the data was read from the cache, then the access time is the time of the
# cache from the portal.
#

# Supported entitlements. Update as needed.
string GfEntBaseThreshold = "threshold"
string GfEntBaseProfile   = "profile"
string GfEntBaseColor     = "color"
string GfEntBaseProfile   = "boolset"

# Message fields
time    access_time   # The last valid portal access time.
int64   gf_class_idx  # Geofrenzy class index
int64   gf_ent_idx    # Geofrenzy unique entitlement index
string  gf_ent_base   # Entitlement base name
bool    dwell         # [Not] within fence
