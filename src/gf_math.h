// GEOFRENZY FILE HEADER HERE

#ifndef _GF_MATH_H
#define _GF_MATH_H

#include <math.h>

#include <string>
#include <limits>
#include <ostream>

#include <Eigen/Eigen>
#include <Eigen/Geometry>

#include "geofrenzy/Polygon64.h"

namespace geofrenzy
{
  namespace gf_math
  {
    /*!
     * \defgroup gfmath Geofrenzy Math
     * 
     * The Eigen3 library data types and algorithms are used by this Geofrenzy
     * source. With ROS, Eigen3 is required, so no additional third-party
     * packages are required for the math.
     * \{
     */

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Eigen geometric constructs
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \defgroup gfmath_types_geo Geometric Data Types
     * \brief Geometric constructs in 3 dimensional ambient space.
     *
     * Eigen data types versions of geometric constructs such as point, line,
     * color, and plane.
     */

    /*!
     * \ingroup gfmath_types_geo
     * \brief Point in 3D space: 3x1 vector of doubles.
     */
    typedef Eigen::Vector3d EigenPoint3;

    /*!
     * \ingroup gfmath_types_geo
     * \brief Parameterized 1D line in 3d space: l(t) = o + t * d.
     */
    typedef Eigen::ParametrizedLine<double, 3> EigenLine3;

    /*!
     * \ingroup gfmath_types_geo
     * \brief 2D plane in 3D space.
     */
    typedef Eigen::Hyperplane<double, 3> EigenPlane3;

    /*!
     * \ingroup gfmath_types_geo
     * \brief Bounding box cuboid.
     * */
    struct EigenBBox3
    {
      EigenPoint3 m_min;  ///< minimum x,y,z
      EigenPoint3 m_max;  ///< maximum x,y,z
    };


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Eigen color spaces
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \defgroup gfmath_types_color Color Data Types
     * \brief Color data types.
     */

    /*!
     * \ingroup gfmath_types_color
     * \brief Color specified as red-green-blue: 3x1 vector of doubles.
     *
     * Each color component is a color intensity in the range [0.0, 1.0]
     * where 0.0 is no color and 1.0 is full intensity.
     */
    typedef EigenPoint3 EigenRGB;

    /*!
     * \ingroup gfmath_types_color
     * \brief Color specified as red-green-blue-alpha: 4x1 vector of doubles.
     *
     * Each color component is a color intensity in the range [0.0, 1.0]
     * where 0.0 is no color and 1.0 is full intensity.
     *
     * The alpha channel is in the range [0.0, 1.0] where 0.0 completely
     * transparent and 1.0 is completely opaque.
     */
    typedef Eigen::Vector4d EigenRGBA;

    /*!
     * \ingroup gfmath_types_color
     * \brief Depth plus color: 6x1 vector of doubles.
     */
    typedef Eigen::Matrix<double, 6, 1> EigenXYZRGB;

    /*!
     * \ingroup gfmath_types_color
     * \brief Depth plus color plus alpha: 7x1 vector of doubles.
     */
    typedef Eigen::Matrix<double, 7, 1> EigenXYZRGBA;


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Eigen scene
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \defgroup gfmath_types_scene Scene Data Types
     * \brief An Eigen scene is built up of objects, with each object
     * corresponding to a Geofrenzy fence.
     */

    /*! 
     * \ingroup gfmath_types_scene
     * \brief List vector of planes container type.
     */
    typedef std::vector<EigenPlane3> EigenPlane3List;

    /*! 
     * \ingroup gfmath_types_scene
     * \brief List vector of bounding boxes container type.
     */
    typedef std::vector<EigenBBox3> EigenBBox3List;

    /*! 
     * \ingroup gfmath_types_scene
     * \brief Eigen scene object data type.
     *
     * Each object has an associated RGBA color, and two synchronized lists of
     * 2D planes and their clipping regions. Each plane is the infinite
     * extension of a Geofrenzy fence polygon segment. 
     */
    struct EigenSceneObj
    {
      EigenRGBA         m_color;    ///< RGBA color of fence
      EigenPlane3List   m_planes;   ///< list of fence planes
      EigenBBox3List    m_bboxes;   ///< list of fence clipping bounding boxes
    };

    /*! 
     * \ingroup gfmath_types_scene
     * \brief Eigen scene data type is a vector of scene objects.
     */
    typedef std::vector<EigenSceneObj> EigenScene;


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Ouput types  
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \defgroup gfmath_types_out Output Data Types
     * \brief Geofrenzy math calculated ouput data types.
     *
     * From Geofrenzy features, a synthetic set of depth plus color points 
     * are generated. These data points can be converted to standard set of
     * ROS messages to create virtual sensors such as point cloud structured
     * light sensors and laser scanners. 
     */

    /*! 
     * \ingroup gfmath_types_out
     * \brief List of depth plus RGB insensity points.
     *
     * The list may or may not be an 2D ordered list and may contain multiple
     * collinear points for the same (x,y).
     */
    typedef std::vector<EigenXYZRGB> EigenXYZRGBList;

    /*! 
     * \ingroup gfmath_types_out
     * \brief List of depth plus RGBA insensity points.
     *
     * The list may or may not be an 2D ordered list and may contain multiple
     * collinear points for the same (x,y).
     */
    typedef std::vector<EigenXYZRGBA> EigenXYZRGBAList;


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Constants
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \defgroup gfmath_const Constants
     * \brief Data constants.
     * \{
     */

    // EigenPoint3
    const size_t  _X = 0; ///< x coordinate (pt[_X] == pt.x())
    const size_t  _Y = 1; ///< y coordinate (pt[_Y] == pt.y())
    const size_t  _Z = 2; ///< z coordinate (pt[_Z] == pt.z())

    // EigenRBG, EigenRGBA
    const size_t  _R = 0; ///< red   (pt[_R] == pt.x())
    const size_t  _G = 1; ///< green (pt[_G] == pt.y())
    const size_t  _B = 2; ///< blue  (pt[_B] == pt.z())
    const size_t  _A = 3; ///< alpha (pt[_A] == pt.w()?)

    // EigenXYZRBG, EigenXYZRGBA
    const size_t  _RED    = 3; ///< xyzred   (pt[_RED])
    const size_t  _GREEN  = 4; ///< xyzgreen (pt[_GREEN])
    const size_t  _BLUE   = 5; ///< xyzblue  (pt[_BLUE])
    const size_t  _ALPHA  = 6; ///< xyzalpha (pt[_ALPHA])
 
    const double Inf = std::numeric_limits<double>::infinity(); ///< infinity

    /*!
     * \brief Default fence color. 50% transparent blueish gray.
     */
    const EigenRGBA FenceColorDft(0.25, 0.25, 0.35, 0.50);

    /*!
     * \brief Minimum fence height (meters).
     */
    const double    FenceMinHeight = 0.10;

    /*!
     * \brief Scanning bit-or'ed options.
     */
    enum ScanOptions
    {
      /*!
       * The default scanning option.
       *  - Include all instersecting points along any traced ray. This option
       *    excludes use of the 2D option.
       *  - Do not produce any 2D structure. That is, if no intersection along
       *    a traced ray is detected, no point is added.
       *  - Do not alpha blend colors along ray.
       */
      ScanOptionDft = 0x00,

      /*!
       * Generate a full height x width 2D structure of points. Any "no object"
       * point has a value of inf.
       */
      ScanOption2D  = 0x01,

      /*!
       *  - Include only the nearest point of a set of intersections along a
       *    traced ray.
       */
      ScanOptionNearest = 0x02,

      /*!
       * Alpha blend colors.
       */
      ScanOptionAlphaBlend = 0x04
    };

    /*! \} */ // gfmath_const


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Functions
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
 
    /*!
     * \defgroup gfmath_func  Functions
     * \{
     */

    /*!
     * \brief Convert degrees to radians.
     *
     * \param degress Degrees.
     *
     * \return Radians
     */
    inline double radians(const double degrees)
    {
      return degrees / 180.0 * M_PI;
    }

    /*!
     * \brief Convert radians to degress.
     *
     * \return radians  Radians
     *
     * \param Degrees
     */
    inline double degrees(const double radians)
    {
      return 180.0 * radians / M_PI;
    }

    /*!
     * \brief Convert spherical coordinate (r,theta,phi) to Cartesion (x,y,z).
     *
     * The Sperical coordinates are as used in mathematics.
     *
     * \param r     Radial distance [0, inf).
     * \param theta Azimuthal angle from x+ axis (-pi, pi].
     * \param phi   Polar angle from z+ [0, pi].
     *
     * \return Cartesion (x,y,z) point.
     */
    EigenPoint3 sphericalToCartesian(const double r,
                                     const double theta,
                                     const double phi);
    
    /*!
     * \brief Convert spherical coordinate (r,theta,phi) to Cartesion (x,y,z).
     *
     * The Sperical coordinates are as used in mathematics.
     *
     * \param       r       Radial distance [0, inf)
     * \param       theta   Azimuthal angle from x+ axis (-pi, pi].
     * \param       phi     Polar angle from z+ [0, pi].
     * \param[out]  pt      Cartesion (x,y,z) point.
     */
    void sphericalToCartesian(const double r,
                              const double theta,
                              const double phi,
                              EigenPoint3  &pt);

    /*!
     * \brief Test if point is within a cuboid bounding box.
     *
     * \param pt    Point to test location.
     * \param bbox  Bounding box.
     *
     * \return Returns true or false.
     */
    bool within(const EigenPoint3 &pt, const EigenBBox3 &bbox);
    
    /*!
     * \brief Convert 24-bit red-green-blue color space into color intensities.
     *
     * \param       red     Red   [0, 255].
     * \param       green   Green [0, 255].
     * \param       blue    Blue  [0, 255].
     * \param[out]  rgb     Color intensities [0.0, 1.0].
     */
    void rgb24ToIntensities(const unsigned int red,
                            const unsigned int green,
                            const unsigned int blue,
                            EigenRGB           &rgb);

    /*!
     * \brief Convert 24-bit red-green-blue-alpha color space into color
     * intensities plus alpha.
     *
     * \param       red     Red   [0, 255].
     * \param       green   Green [0, 255].
     * \param       blue    Blue  [0, 255].
     * \param       alpha   Alpha transparent to opaque [0.0, 1.0].
     * \param[out]  rgba    Color intensities plus alpha [0.0, 1.0].
     */
    void rgb24ToIntensities(const unsigned int red,
                            const unsigned int green,
                            const unsigned int blue,
                            const double       alpha,
                            EigenRGBA          &rgba);

    /*!
     * \brief Convert color intensities into 24-bit red-green-blue color space.
     *
     * \param[in]   rgb     Color intensities [0.0, 1.0].
     * \param[out]  red     Red   [0, 255].
     * \param[out]  green   Green [0, 255].
     * \param[out]  blue    Blue  [0, 255].
     */
    void intensitiesToRgb24(const EigenRGB &rgb,
                            unsigned int   &red,
                            unsigned int   &green,
                            unsigned int   &blue);

    /*!
     * \brief Convert color intensities plus alpha channel into 24-bit
     * red-green-blue-alpha color space.
     *
     * \param[in]   rgba    Color intensities plus alpha [0.0, 1.0] plus alpha.
     * \param[out]  red     Red   [0, 255].
     * \param[out]  green   Green [0, 255].
     * \param[out]  blue    Blue  [0, 255].
     * \param       alpha   Alpha transparent to opaque [0.0, 1.0].
     */
    void intensitiesToRgb24(const EigenRGBA &rgba,
                            unsigned int    &red,
                            unsigned int    &green,
                            unsigned int    &blue,
                            double          &alpha);

    /*!
     * \brief Blend two colors using the alpha channel.
     *
     * \sa https://en.wikipedia.org/wiki/Alpha_compositing
     *
     * \param[in]   colorFg   Foreground color intensities + alpha.
     * \param[in]   colorBg   Background color intensities + alpha.
     * \param[out]  colorOut  Output blended color intensities + alpha.
     */
    void alphaBlend(const EigenRGBA &colorFg,
                    const EigenRGBA &colorBg,
                    EigenRGBA       &colorOut);

    ///@{
    /*!
     * \brief Point stream insertion operator.
     *
     * \param os  Output stream.
     * \param pt  Object to insert.
     *
     * \return Reference to output stream.
     */
    std::ostream &operator<<(std::ostream &os, const EigenPoint3 &pt);

    std::ostream &operator<<(std::ostream &os, const EigenRGBA &pt);

    std::ostream &operator<<(std::ostream &os, const EigenXYZRGB &pt);

    std::ostream &operator<<(std::ostream &os, const EigenXYZRGBA &pt);
    ///@}

    /*!
     * \brief Bounding box stream insertion operator.
     *
     * \param os    Output stream.
     * \param bbox  Object to insert.
     *
     * \return Reference to output stream.
     */
    std::ostream &operator<<(std::ostream &os, const EigenBBox3 &bbox);

    /*!
     * \brief Create a scene object.
     *
     * \param polygon       ROS polygon defining a fence. Each (x,y,z) point
     *                      specifies a distance from a reference point or an
     *                      observer (meters).
     * \param color         Color attribute applied to the fence.
     *                      (red-green-blue intensities + alpha).
     * \param fenceHeight   Height of fence (meters).  
     * \param[out] sceneObj Created scene object.
     */
    void createSceneObj(const Polygon64 &polygon,
                        const EigenRGBA &color,
                        const double    &fenceHeight,
                        EigenSceneObj   &sceneObj);

    /*!
     * \brief Scan virtual scene to generate a list of intersecting depth +
     * color points.
     *
     * Scanning proceeds from the minimum to maximum phi in height steps, with
     * each step sweeping from the minimum to maximum theta in width steps.
     *
     * The spherical angles are as used in mathematics.
     *
     * \param       thetaMin    Azimuthal minimum angle from x+ axis (-pi, pi].
     * \param       thetaMax    Azimuthal max angle from x+ axis (-pi, pi].
     * \param       phiMin      Polar minimum angle from z+ [0, pi].
     * \param       phiMax      Polar maximum angle from z+ [0, pi].
     * \param       width       Width resolution. Number of horizontal points.
     * \param       height      Height resoluion. Number of vertical points.
     * \param       scene       The scene to scan.
     * \param[out]  intersects  List of intersecting points.
     * \param       options     Options to control the scan.
     */
    void scanScene(const double thetaMin, const double thetaMax,
                   const double phiMin,   const double phiMax,
                   const size_t width,    const size_t height,
                   const EigenScene       &scene,
                   EigenXYZRGBAList       &intersects,
                   uint32_t               options = ScanOptionDft);

    /*!
     * \brief Trace ray through virtual scene to generate a list of
     * intersecting depth + color points.
     *
     * \param       ray         The ray (parametrized line) to trace.
     * \param       scene       The scene.
     * \param[out]  intersects  List of intersecting points.
     * \param       options     Options to control the trace.
     *
     * \return Number of intersections added.
     */
    size_t traceRay(const EigenLine3 &ray,
                    const EigenScene &scene,
                    EigenXYZRGBAList &intersects,
                    uint32_t         options = ScanOptionDft);

    /*! \} */ // end of gfmath_func group


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Unit Tests
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

#define GF_MATH_UT  ///< define/undef to enable/disable unit test functions.

#ifdef GF_MATH_UT
    /*!
     * \defgroup gfmath_ut Unit Test
     * \{
     */

    const int UtPolynumTriangle   = 0;  ///< equalateral triangle with 50m sides
    const int UtPolynumRectangle  = 1;  ///< 20m x 30m rectangle
    const int UtPolynumHexagon    = 2;  ///< hexagon with 10m sides
    const int UtPolynumTee        = 3;  ///< 40m x 50m tee 

    /*!
     * \brief Make a ROS polygon message from a canned shape.
     *
     * \param polynum       Canned shape number. See above.
     * \param offset        Offset add to polygon position.
     * \param scale         Polygon size scale multiplier. 
     * \param [out] polygon Output polygon message.
     */
    void utMakeCannedPolygon(const int         polynum,
                             const EigenPoint3 &offset,
                             const double      scale,
                             Polygon64         &polygon);

    /*!
     * \brief Scan a single polygon.
     *
     * \param polygon Polygon to scan.
     */
    void utScanPolygon(const Polygon64 &polygon);
#endif // GF_MATH_UT

    /*! \} */ // end of gfmath_ut group
    
    /*! \} */ // end of gfmath group

  } // namespace gf_math
} // namespace geofrenzy

#endif // _GF_MATH_H
