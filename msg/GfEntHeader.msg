### Geofrenzy GfEntHeader message
#
# The Geofrenzy entitlement header contains common identifying properties for
# an entitlement.
#
# The validity and timeliness of the data is specified by the access time.
# If zero (0), the data is not valid and the dwell fields are unreliable.
# If the data was read from the cache, then the access time is the time of the
# cache from the portal.
#

# Message fields
time    access_time   # The last valid portal access time.
int64   gf_class_idx  # Geofrenzy class index.
int64   gf_ent_idx    # Geofrenzy unique entitlement index.
string  gf_ent_base   # Entitlement base name (and data type).
bool    dwell         # [Not] within the fence
