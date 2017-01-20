class geojson_point
{
  public:
    double longitude; ///<longitude in goejson feature
    double latitude;  ///<latitude in geojson feature
};

class geojson_points
{
  public:
    std::vector<geojson_point> point; ///<point list in geosjson feature
};

class geojson_linear_rings
{
  public:
    std::vector<geojson_points> ring; ///<rings in geojson polygon
};

class geojson_geometry
{
  public:
    std::string type;                        ///< geojson geometry type
    std::vector<geojson_linear_rings> rings; ///<geojson array of rings comprising geojson polygon
};

class geojson_gfproperties
{
  public:
    std::string entitlement; ///<GeoFrenzy entitlement string for fence
    std::string inout;       ///<is the GPS locations submitted in the GoeFrenzy Fence request inside or outside this fence
                             ///< 'i' = within the fence; 'o' = outside the fence;
};

class geojson_feature
{
  public:
    std::string type;
    geojson_gfproperties properties; ///<geojson feature properties - should hold the GeoFrenzy metadata about the fence
    geojson_geometry geometry;       ///<geojson feature geometry
};

class geojson_feature_collection
{
  public:
    std::vector<geojson_feature> feature; ///<geojson feature in geojson feature collection
};

class geojson_root_fc
{
  public:
    std::string type;                    ///<geojson root type
    geojson_feature_collection features; ///<features list in geojson feature collection
};
