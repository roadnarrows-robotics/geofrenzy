### Geofrenzy GfEntTVP message
#
# The Geofrenzy entitlement type-value pair holds a generatlized entitlement
# data attribute.
#
# ROS messges do not support conditional values (and their types), so it is not
# possible to define a generalize entitlement that _only_ contains data for that
# entitlement type (ent_idx and/or ent_base). So the approach here is define a
# type-value message. The disadvantage is that all data is tranmitted but only
# one value is relevant.
#

# Attribute types
uint8 GfEntAttrTypeUndef  = 0   # undefined

uint8 GfEntAttrTypeBool   = 1   # boolean; (bool)val_int

uint8 GfEntAttrTypeU8     = 2   #  8-bit unsigned int; (uint8_t)val_int
uint8 GfEntAttrTypeU16    = 3   # 16-bit unsigned int; (uint16_t)val_int
uint8 GfEntAttrTypeU32    = 4   # 32-bit unsigned int; (uint32_t)val_int
uint8 GfEntAttrTypeU64    = 5   # 64-bit unsigned int; (uint64_t)val_int

uint8 GfEntAttrTypeI8     = 6   #  8-bit signed int; (int8_t)val_int
uint8 GfEntAttrTypeI16    = 7   # 16-bit signed int; (int16_t)val_int
uint8 GfEntAttrTypeI32    = 8   # 32-bit signed int; (int32_t)val_int
uint8 GfEntAttrTypeI64    = 9   # 64-bit signed int; (int64_t)val_int

uint8 GfEntAttrTypeFpn32  = 10  # 32-bit floating-point number; (float)val_fpn
uint8 GfEntAttrTypeFpn64  = 11  # 64-bit floating-point number; (double)val_fpn

uint8 GfEntAttrTypeString = 12  # string; val_str

# Message fields
uint8   attr_type     # Attribute type. One of the above constants.
int64   val_int       # A bool, unsigned integer, or signed integer value
float64 val_fpn       # A float or double value
string  val_str       # A string value
