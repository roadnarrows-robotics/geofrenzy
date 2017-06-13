### Geofrenzy GfDwellJson message
#
# A Json dwell message specifies a current entitlement in relation to the
# tracked location as an encoded Json string. Although this message is the
# most extensible for future dwell types, it does require Json processing
# on the receiver side.
#
Header      header      # Standard ROS header
GfEntJson   entitlement # Dwell entitlement
