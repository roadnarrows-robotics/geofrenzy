////////////////////////////////////////////////////////////////////////////////
//
// Package:   Geofrenzy Robot ROS Package
//
// Link:      https://github.com/roadnarrows-robotics/geofrenzy
//
// File:      gf_math.h
//
/*! \file
 *
 * \brief The Geofrenzy math declarations.
 *
 * \author Bill Coon (bill@roadnarrows.com)
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Maintainer:
 * Chris Stradtman (chris.stradtman@geo.network)
 *
 * \par Copyright:
 * (C) 2017  GeoNetwork
 * (http://www.geo.network)
 * \n All Rights Reserved
 *
 * \par License
 * Apache 2.0
 * 
 * EULA:
 * See EULA.md
 */
////////////////////////////////////////////////////////////////////////////////

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

    /*!
     * \defgroup gfmath_types Data Types
     * \brief Geofrenzy math data types.
     * \{
     * \}
     */

    /*!
     * \defgroup gfmath_const Constants
     * \brief Geofrenzy math data constants.
     * \{
     * \}
     */

    /*!
     * \defgroup gfmath_basic_ops Basic Operations 
     * \brief Geofrenzy math basic operations.
     * \{
     * \}
     */

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Eigen geometric constructs and simple operations.
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \ingroup gfmath_types
     * \brief Basic Geometric Data Types
     *
     * Eigen data types of geometric constructs in 2 and 3 dimensional
     * ambient space, such as point, line, and plane.
     * \{
     */

    /*!
     * \brief Point in 2D space: 2x1 vector of doubles.
     */
    typedef Eigen::Vector2d EigenPoint2;

    /*!
     * \brief Point in 3D space: 3x1 vector of doubles.
     */
    typedef Eigen::Vector3d EigenPoint3;

    /*!
     * \brief Parameterized 1D line in 2D space: l(t) = o + t * d.
     */
    typedef Eigen::ParametrizedLine<double, 2> EigenLine2;

    /*!
     * \brief Parameterized 1D line in 3D space: l(t) = o + t * d.
     */
    typedef Eigen::ParametrizedLine<double, 3> EigenLine3;

    /*!
     * \brief 2D plane in 3D space.
     */
    typedef Eigen::Hyperplane<double, 3> EigenPlane3;

    /*!
     * \brief 1 2-tuple minimum,maximum limit.
     *
     * Limits: [minmax[_MIN], minmax[_MAX]]
     */
    typedef EigenPoint2 EigenMinMax1; 

    /*!
     * \brief 2 2-tuple minimum,maximum limits.
     *
     * Limits: [m_min[k], m_max[k]], k=0,1
     */
    struct EigenMinMax2
    {
      EigenPoint2 m_min;  ///< minimums k=0,1 or x,y
      EigenPoint2 m_max;  ///< maximums k=0,1 or x,y
    };

    /*!
     * \brief 2 3-tuple minimum,maximum limits.
     *
     * Limits: [m_min[k], m_max[k]], k=0,2
     */
    struct EigenMinMax3
    {
      EigenPoint3 m_min;  ///< minimum k=0,2 or x,y,z
      EigenPoint3 m_max;  ///< maximum k=0,2 or x,y,z
    };

    /*!
     * \brief Bounding box rectangular cuboid.
     */
    typedef EigenMinMax3 EigenBBox3;

    /*! 
     * \brief Basic container types.
     */

    /*!
     * \brief List (vector) of 2D points.
     */
    typedef std::vector<EigenPoint2> EigenPoint2List;

    /*! 
     * \brief List (vector) of 3D points.
     */
    typedef std::vector<EigenPoint3> EigenPoint3List;

    /*! 
     * \brief List (vector) of limits container type.
     */
    typedef std::vector<EigenMinMax2> EigenMinMax2List;

    /*! 
     * \brief List (vector) of 3D bounding boxes container type.
     */
    typedef std::vector<EigenBBox3> EigenBBox3List;

    /*! \} */ // gfmath_types

    /*!
     * \ingroup gfmath_const
     * \brief Geometric constants.
     * \{
     */

    /*! tau is defined as 2 pi */
    #define M_TAU (2.0 * M_PI)

    /*!
     * \brief Minimum and maximum indices.
     *
     * \sa EigenPoint2
     */
    const size_t  _MIN = 0; ///< x coordinate (= pt.x())
    const size_t  _MAX = 1; ///< y coordinate (= pt.y())
  
    /*!
     * Cartesian x,y,z coordinates indices.
     *
     * \sa EigenPoint2(_X,_Y only), EigenPoint3, EigenXYZRGB, EigenXYZRGBA
     *
     */
    const size_t  _X = 0; ///< x coordinate (= pt.x())
    const size_t  _Y = 1; ///< y coordinate (= pt.y())
    const size_t  _Z = 2; ///< z coordinate (= pt.z())
  
    /*!
     * \brief Spherical coordinates indices r,theta,phi.
     *
     * \sa EigenPoint3
     */
    const size_t  _R      = 0;  ///< radial distance [0, inf).
    const size_t  _THETA  = 1;  ///< azimuthal angle from x+ axis (-pi, pi].
    const size_t  _PHI    = 2;  ///< polar angle from z+ [0, pi].

    /*!
     * \brief X-Y plane quadrants.
     */
    const int _QNone  = 0;    ///< no quadrant - on an axis line
    const int _QI     = 1;    ///< quadrant 1: x+, y+
    const int _QII    = 2;    ///< quadrant 2: x-, y+
    const int _QIII   = 3;    ///< quadrant 3: x-, y-
    const int _QIV    = 4;    ///< quadrant 4: x+, y-

    /*!
     * \brief X-Y-Z solid octants.
     */
    const int _ONone  = 0;    ///< no octant - on an axis plane
    const int _OI     = 1;    ///< octant 1: x+, y+, z+
    const int _OII    = 2;    ///< octant 2: x-, y+, z+
    const int _OIII   = 3;    ///< octant 3: x-, y-, z+
    const int _OIV    = 4;    ///< octant 4: x+, y-, z+
    const int _OV     = 5;    ///< octant 5: x+, y+, z-
    const int _OVI    = 6;    ///< octant 6: x-, y+, z-
    const int _OVII   = 7;    ///< octant 7: x-, y-, z-
    const int _OVIII  = 8;    ///< octant 8: x+, y-, z-

    /*!
     * \brief Infinity
     */
    const double Inf = std::numeric_limits<double>::infinity(); ///< infinity

    /*!
     * \brief Origin in 2D space.
     */
    const EigenPoint2 Origin2(0.0, 0.0);

    /*!
     * \brief Origin in 3D space.
     */
    const EigenPoint3 Origin3(0.0, 0.0, 0.0);

    /*!
     * \brief Minimum fence height (meters).
     */
    const double FenceMinHeight = 0.10;

    /*!
     * \brief Default precision.
     */
    const double PrecisionDft = 1.0e-10;

    /*! \} */ // gfmath_const
  
    /*!
     * \ingroup gfmath_basic_ops
     * \brief Geometric basic operations.
     * \{
     */

    /*!
     * \brief Convert degrees to radians.
     *
     * \param degrees Degrees.
     *
     * \return Radians
     */
    inline double radians(const double degrees)
    {
      return degrees / 180.0 * M_PI;
    }

    /*!
     * \brief Convert radians to degrees.
     *
     * \param radians  Radians
     *
     * \return Degrees
     */
    inline double degrees(const double radians)
    {
      return 180.0 * radians / M_PI;
    }

    /*!
     * \brief Cap value between [min, max].
     *
     * \param val Value to cap.
     * \param min Minimum value.
     * \param max Maximum value.
     *
     * \return Capped value [min, max].
     */
    inline double cap(const double val, const double min, const double max)
    {
      return val < min? min: val > max? max: val;
    }

    /*!
     * \brief Determine sign of value.
     *
     * Zero is considered position.
     *
     * \param val   Value to test.
     *
     * \return Returns 1 or -1 if value is non-negative or negative,
     * respectively.
     */
    inline int sign(const double val)
    {
      return val >= 0.0? 1: -1;
    }

    /*
     * \brief Convert 3D point to x-y plane projected 2D point.
     *
     * \return Returns 2D point on x-y plane.
     */
    inline EigenPoint2 xy(const EigenPoint3 &pt)
    {
      return EigenPoint2(pt.x(), pt.y());
    }

    /*
     * \brief Convert 3D point to x-z plane projected 2D point.
     *
     * \return Returns 2D point on x-z plane.
     */
    inline EigenPoint2 xz(const EigenPoint3 &pt)
    {
      return EigenPoint2(pt.x(), pt.z());
    }

    /*
     * \brief Convert 3D point to y-z plane projected 2D point.
     *
     * \return Returns 2D point on y-z plane.
     */
    inline EigenPoint2 yz(const EigenPoint3 &pt)
    {
      return EigenPoint2(pt.y(), pt.z());
    }

    /*!
     * \brief Test if values are approximately equal.
     *
     * \param val0      Value 0.
     * \param val1      Value 1.
     * \param precision Precision of equality.
     *
     * \return Returns true or false.
     */
    inline bool isApprox(const double val0,
                         const double val1,
                         const double precision = PrecisionDft)
    {
      return fabs(val1 - val0) <= precision;
    }

    /*!
     * \brief Test if value are approximately equal to 0 (zero).
     *
     * \param val       Value to test.
     * \param precision Precision of equality.
     *
     * \return Returns true or false.
     */
    inline bool isApproxZero(const double val,
                             const double precision = PrecisionDft)
    {
      return fabs(val) <= precision;
    }

    /*!
     * \brief Test if two points, within precision, are approximately equal.
     *
     * \param pt0         Point 0.
     * \param pt1         Point 1.
     * \param precesion   Precision of equality check.
     *
     * \return Returns true or false.
     */
    inline bool isApprox(const EigenPoint2 &pt0,
                         const EigenPoint2 &pt1,
                         const double      precision = PrecisionDft)
    {
      return  (fabs(pt1.x() - pt0.x()) <= precision) &&
              (fabs(pt1.y() - pt0.y()) <= precision);
    }

    /*!
     * \brief Test if two points, within precision, are approximately equal.
     *
     * \param pt0         Point 0.
     * \param pt1         Point 1.
     * \param precesion   Precision of equality check.
     *
     * \return Returns true or false.
     */
    inline bool isApprox(const EigenPoint3 &pt0,
                         const EigenPoint3 &pt1,
                         const double      precision = PrecisionDft)
    {
      return  (fabs(pt1.x() - pt0.x()) <= precision) &&
              (fabs(pt1.y() - pt0.y()) <= precision) &&
              (fabs(pt1.z() - pt0.z()) <= precision);
    }

    /*!
     * \brief Remap angle into equivalent (-pi, pi] range.
     *
     * \param a   Angle (radians).
     *
     * \return Equivalent angle in (-pi, pi].
     */
    inline double pi2pi(const double a)
    {
      if( a > M_PI )
      {
        return a - M_TAU;
      }
      else if( a <= -M_PI )
      {
        return a + M_TAU;
      }
      else
      {
        return a;
      }
    }

    /*!
     * \brief Conditionally rotate angle pi radians if negative.
     *
     * \param a   Angle (radians).
     *
     * \return New angle in [0.0, pi].
     */
    inline double rot180if(const double a)
    {
      double b = pi2pi(a);
      return b >= 0.0? b: b + M_PI;
    }

    /*!
     * \brief Rotate angle pi radians.
     *
     * \param a   Angle (radians).
     *
     * \return Rotated angle in (-pi, pi].
     */
    inline double rot180(const double a)
    {
      return pi2pi(a + M_PI);
    }

    /*!
     * \brief Convert polar coordinate (r,theta) to Cartesion (x,y).
     *
     * The polar coordinates are as used in mathematics.
     *
     * \param r     Radial distance [0, inf).
     * \param theta Azimuthal angle from x+ axis (-pi, pi].
     *
     * \return Cartesion (x,y) point.
     */
    inline EigenPoint2 polarToCartesian(const double r, const double theta)
    {
      return EigenPoint2(r * cos(theta), r * sin(theta));
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
    inline EigenPoint3 sphericalToCartesian(const double r,
                                            const double theta,
                                            const double phi)
    {
      return EigenPoint3(r * sin(phi) * cos(theta),
                         r * sin(phi) * sin(theta),
                         r * cos(phi));
    }
    
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
    inline void sphericalToCartesian(const double r,
                                     const double theta,
                                     const double phi,
                                     EigenPoint3  &pt)
    {
      pt << r * sin(phi) * cos(theta), r * sin(phi) * sin(theta), r * cos(phi);
    }

    /*!
     * \brief Calculate the Euclidean absolute distance between two points in
     * Re2.
     *
     * \param pt0   Point 0.
     * \param pt1   Point 1.
     *
     * \return Distance >= 0.0
     */
    inline double distance(const EigenPoint2 &pt0, const EigenPoint2 &pt1)
    {
      return sqrt(pow(pt1.x()-pt0.x(), 2.0) + pow(pt1.y()-pt0.y(), 2.0));
    }

    /*!
     * \brief Calculate the Euclidean absolute distance between two points in
     * Re3.
     *
     * \param pt0   Point 0.
     * \param pt1   Point 1.
     *
     * \return Distance >= 0.0
     */
    inline double distance(const EigenPoint3 &pt0, const EigenPoint3 &pt1)
    {
      return sqrt(pow(pt1.x()-pt0.x(), 2.0) +
                  pow(pt1.y()-pt0.y(), 2.0) +
                  pow(pt1.z()-pt0.z(), 2.0));
    }

    /*!
     * \brief Calculate the distance from the origin (0,0) to a line in 2D.
     *
     * The 2D line is defined by the two points pt1 and pt2.
     *
     * The projection point forms an orthogonal line between the origin and
     * that projection point to the 2D line.
     *
     * \param pt1   Line point 1.
     * \param pt2   Line point 2.
     *
     * \return Projection distance >= 0.0
     */
    inline double projection(const EigenPoint2 &pt1, const EigenPoint2 &pt2)
    {
      return fabs(pt2.x() * pt1.y() - pt2.y() * pt1.x()) /
              sqrt(pow(pt2.y() - pt1.y(), 2.0) + pow(pt2.x() - pt1.x(), 2.0));
    }

    /*!
     * \brief Calculate the distance from a point pt0 to a line in 2D
     *
     * The 2D line is defined by the two points pt1 and pt2.
     *
     * The projection on the line pt1,pt2 of the point pt0 forms a point on the
     * line that defines an orthogonal line between the projection and point
     * pt0
     *
     * \param pt1   Line point 1.
     * \param pt2   Line point 2.
     * \param pt0   Point to project.
     *
     * \return Projection distance >= 0.0
     */
    inline double projection(const EigenPoint2 &pt1,
                             const EigenPoint2 &pt2,
                             const EigenPoint2 &pt0)
    {
      double dx = pt2.x() - pt1.x();
      double dy = pt2.y() - pt1.y();

      return fabs(dy*pt0.x() - dx*pt0.y() + pt2.x()*pt1.y() - pt2.y()*pt1.x()) /
              sqrt(pow(dy, 2.0) + pow(dx, 2.0));
    }

    /*!
     * \brief Calculate the inclination angle of a line.
     *
     * The 2D line is defined by the two points pt0 and pt1.
     *
     * The inclination is the angle to the x-axis in [0.0, pi).
     *
     * \param pt0   Line point 1.
     * \param pt1   Line point 2.
     *
     * \return Inclination (radians).
     */
    inline double inclination(const EigenPoint2 &pt0, const EigenPoint2 &pt1)
    {
      double alpha = rot180if(atan2(pt1.y() - pt0.y(), pt1.x() - pt0.x()));
      return alpha < M_PI? alpha: 0.0;
    }

    /*!
     * \brief Find the intersection of two 2D parametric lines.
     *
     * \param line1   Parametric line 1 line1(t) = o1 + t * d1
     * \param line2   Parametric line 2 line2(u) = o1 + u * d1
     *
     * \return
     * If the lines intersect, then the intersection point is returned.
     * If the lines are parallel, the (Inf, Inf) point is returned.
     * If the lines are identical, any point on the line is returned.
     */
    EigenPoint2 intersection(const EigenLine2 &line1, const EigenLine2 &line2);

    /*!
     * \brief Find the t value for the parametric line at the given point.
     *
     * First x, then the y component is tried. No consistency check is made.
     *
     * \param line  Parametric line in 3D space.
     * \param pt    Point in the x-y plane.
     *
     * \return Returns t or Inf.
     */
    double t_param(const EigenLine3 &line, const EigenPoint2 &pt);

    /*!
     * \brief Check if the value is within the min,max limits.
     *
     * \param val   Value to check.
     * \param lim   Limits.
     *
     * \return Returns true or false if value in [min, max].
     */
    inline bool within(const double &val, const EigenMinMax1 &lim)
    {
      return  (val >= lim[_MIN]) && (val <= lim[_MAX]);
    }

    /*!
     * \brief Check if the value is within one of the set of min,max limits.
     *
     * \param val   Value to check.
     * \param lim   Limits.
     *
     * \return Returns true or false if value in [min[k], max[k]], k=0,1.
     */
    inline bool within(const double &val, const EigenMinMax2 &lim)
    {
      return  ((val >= lim.m_min[0]) && (val <= lim.m_max[0])) ||
              ((val >= lim.m_min[1]) && (val <= lim.m_max[1]));
    }

    /*!
     * \brief Check if the value is within one of the set of min,max limits.
     *
     * \param val   Value to check.
     * \param lim   Limits.
     *
     * \return Returns true or false if value in [min[k], max[k]], k=0,2.
     */
    inline bool within(const double &val, const EigenMinMax3 &lim)
    {
      return  ((val >= lim.m_min[0]) && (val <= lim.m_max[0])) ||
              ((val >= lim.m_min[1]) && (val <= lim.m_max[1])) ||
              ((val >= lim.m_min[2]) && (val <= lim.m_max[2]));
    }

    /*!
     * \brief Determine the x-y plane quadrant the point lies within.
     *
     * \param pt        2D point.
     * \param precision Precision of position.
     *
     * \return Returns 1-4 for quadrants QI - QIV, respectively. Returns 0
     * if point lies on or is sufficiently close to an axis.
     */
    int quadrant(const EigenPoint2 &pt, const double precision = PrecisionDft);

    /*!
     * \brief Determine the x-y plane quadrant the angle from origin projects
     * into.
     *
     * \param theta     Angle in the x-y plane (radians).
     * \param precision Precision of angle.
     *
     * \return Returns 1-4 for quadrants QI - QIV, respectively. Returns 0
     * if angle lies on or is sufficiently close to an axis.
     */
    int quadrant(const double theta, const double precision = PrecisionDft);

    /*!
     * \brief Determine the x-y-z actant the point lies within.
     *
     * \param pt        3D point.
     * \param precision Precision of position.
     *
     * \return Returns 1-8 for octants OI - OVII, respectively. Returns 0
     * if point lies on or is sufficiently close to an axis plane.
     */
    int octant(const EigenPoint3 &pt, const double precision = PrecisionDft);

    /*!
     * \brief Test if point is contained within a rectangular cuboid bounding
     * box.
     *
     * \param pt    Point to test.
     * \param bbox  Bounding box.
     *
     * \return Returns true or false.
     */
    inline bool contained(const EigenPoint3 &pt, const EigenBBox3 &bbox)
    {
      return  (pt.x() >= bbox.m_min.x()) && (pt.x() <= bbox.m_max.x()) &&
              (pt.y() >= bbox.m_min.y()) && (pt.y() <= bbox.m_max.y()) &&
              (pt.z() >= bbox.m_min.z()) && (pt.z() <= bbox.m_max.z());
    }

    /*!
     * \brief Point in Polygon test.
     *
     * This method uses the edge crossing Counting Number algorithm.
     *
     * \sa http://geomalgorithms.com/a03-_inclusion.html
     *
     * If the number of edge crossings is odd, the the point is inside the
     * polygon. Otherwise it lies outside. Any point on an edge or vertex
     * is considered inside.
     *
     * Note that this simple algortihm does not work well with complex,
     * crossing polygons.
     *
     * \param pt      Point to test.
     * \param polygon Closed polygon with polygon[n] == polygon[0].
     *
     * \return Returns true if inside, false if outside.
     */
    bool pipCn(const EigenPoint2 &pt, const EigenPoint2List &polygon);

    /*!
     * \brief Point in Polygon test.
     *
     * The 3D point is projected onto the x-y plane (i.e. z = 0)
     *
     * \param pt      Point to test.
     * \param polygon Closed polygon with polygon[n] == polygon[0].
     *
     * \return Returns true if inside, false if outside.
     */
    inline bool pipCn(const EigenPoint3 &pt, const EigenPoint2List &polygon)
    {
      EigenPoint2 pt2(pt.x(), pt.y());
      return pipCn(pt2, polygon);
    }

    /*! \} */ // gfmath_basic_ops


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Eigen color spaces and simple operations.
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \ingroup gfmath_types
     * \brief Basic Color Data Types
     * \{
     */

    /*!
     * \brief Color specified as red-green-blue: 3x1 vector of doubles.
     *
     * Each color component is a color intensity in the range [0.0, 1.0]
     * where 0.0 is no color and 1.0 is full intensity.
     */
    typedef EigenPoint3 EigenRGB;

    /*!
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
     * \brief Depth plus color: 6x1 vector of doubles.
     */
    typedef Eigen::Matrix<double, 6, 1> EigenXYZRGB;

    /*!
     * \brief Depth plus color plus alpha: 7x1 vector of doubles.
     */
    typedef Eigen::Matrix<double, 7, 1> EigenXYZRGBA;

    /*! \} */ // gfmath_types

    /*!
     * \ingroup gfmath_const
     * \brief Color constants.
     * \{
     */

    /*!
     * \brief Color red-green-blue-alpha indices.
     *
     * \sa EigenRBG, EigenRGBA
     */
    const size_t  _RED    = 0; ///< red   (= pt.x())
    const size_t  _GREEN  = 1; ///< green (= pt.y())
    const size_t  _BLUE   = 2; ///< blue  (= pt.z())
    const size_t  _ALPHA  = 3; ///< alpha (= pt.w()?)
  
    /*!
     * \brief Color red-green-blue-alpha indices for XYZ+ points.
     *
     * \sa EigenXYZRBG, EigenXYZRGBA
     */
    const size_t  _XYZRED   = 3; ///< xyzred
    const size_t  _XYZGREEN = 4; ///< xyzgreen
    const size_t  _XYZBLUE  = 5; ///< xyzblue
    const size_t  _XYZALPHA = 6; ///< xyzalpha

    /*!
     * \brief Maximum 24-bit color 8-bit channel (rgb) values and mask.
     */
    const double        Color24ChanMin  =   0.0;    ///< minimum value
    const double        Color24ChanMax  = 255.0;    ///< maximum value
    const unsigned int  Color24ChanMask = 0x00ff;   ///< mask

    /*!
     * \brief Default fence color. 50% transparent blueish gray.
     */
    const EigenRGBA FenceColorDft(0.30, 0.30, 0.45, 0.50);

    /*! \} */ // gfmath_const
 
    /*!
     * \ingroup gfmath_basic_ops
     * \brief Color basic operations.
     * \{
     */

    /*!
     * \brief Make black color.
     *
     * \param[out] color  Color to black.
     */
    inline void mkBlack(EigenRGB &color)
    {
      color << 0.0, 0.0, 0.0;
    }

    /*!
     * \brief Make transparent black color.
     *
     * \param[out] color  Color to black.
     */
    inline void mkBlack(EigenRGBA &color)
    {
      color << 0.0, 0.0, 0.0, 0.0;
    }

    /*!
     * \brief Convert 24-bit red-green-blue color space into color intensities.
     *
     * \param       red     Red   [0, 255].
     * \param       green   Green [0, 255].
     * \param       blue    Blue  [0, 255].
     * \param[out]  rgb     Color intensities [0.0, 1.0].
     */
    inline void rgb24ToIntensities(const unsigned int red,
                                   const unsigned int green,
                                   const unsigned int blue,
                                   EigenRGB           &rgb)
    {
      rgb <<  (double)(  red & Color24ChanMask)/Color24ChanMax,
              (double)(green & Color24ChanMask)/Color24ChanMax,
              (double)( blue & Color24ChanMask)/Color24ChanMax;
    }

    /*!
     * \brief Convert 24-bit red-green-blue plus alpha color space into color
     * intensities plus alpha.
     *
     * \param       red     Red   [0, 255].
     * \param       green   Green [0, 255].
     * \param       blue    Blue  [0, 255].
     * \param       alpha   Alpha transparent to opaque [0.0, 1.0].
     * \param[out]  rgba    Color intensities plus alpha [0.0, 1.0].
     */
    inline void rgb24ToIntensities(const unsigned int red,
                                   const unsigned int green,
                                   const unsigned int blue,
                                   const double       alpha,
                                   EigenRGBA          &rgba)
    {
      rgba << (double)(  red & Color24ChanMask)/Color24ChanMax,
              (double)(green & Color24ChanMask)/Color24ChanMax,
              (double)( blue & Color24ChanMask)/Color24ChanMax,
              cap(alpha, 0.0, 1.0);
    }

    /*!
     * \brief Convert color intensities into 24-bit red-green-blue color space.
     *
     * \param[in]   rgb     Color intensities [0.0, 1.0].
     * \param[out]  red     Red   [0, 255].
     * \param[out]  green   Green [0, 255].
     * \param[out]  blue    Blue  [0, 255].
     */
    inline void intensitiesToRgb24(const EigenRGB &rgb,
                                   unsigned int   &red,
                                   unsigned int   &green,
                                   unsigned int   &blue)
    {
      red   = (unsigned int)(rgb[_RED]   * Color24ChanMax) & Color24ChanMask;
      green = (unsigned int)(rgb[_GREEN] * Color24ChanMax) & Color24ChanMask;
      blue  = (unsigned int)(rgb[_BLUE]  * Color24ChanMax) & Color24ChanMask;
    }

    /*!
     * \brief Convert color intensities plus alpha channel into 24-bit
     * red-green-blue plus alpha color space.
     *
     * \param[in]   rgba    Color intensities plus alpha [0.0, 1.0] plus alpha.
     * \param[out]  red     Red   [0, 255].
     * \param[out]  green   Green [0, 255].
     * \param[out]  blue    Blue  [0, 255].
     * \param       alpha   Alpha transparent to opaque [0.0, 1.0].
     */
    inline void intensitiesToRgb24(const EigenRGBA &rgba,
                                   unsigned int    &red,
                                   unsigned int    &green,
                                   unsigned int    &blue,
                                   double          &alpha)
    {
      red   = (unsigned int)(rgba[_RED]   * Color24ChanMax) & Color24ChanMask;
      green = (unsigned int)(rgba[_GREEN] * Color24ChanMax) & Color24ChanMask;
      blue  = (unsigned int)(rgba[_BLUE]  * Color24ChanMax) & Color24ChanMask;
      alpha = rgba[_ALPHA];
    }

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

    /*! \} */ // gfmath_basic_ops


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Eigen scene
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \defgroup gfmath_scene Scene Data Types and Functions
     * \brief An Eigen scene is built up of objects, with each object
     * a set of attributes and surfaces.
     *
     * \{
     */

    /*!
     * \brief Eigen planar surface rendering properties.
     *
     * The properties are used to speed computer rendering of a surface.
     */
    struct EigenSurface
    {
      unsigned int    m_num;        ///< surface number in scene object
      EigenPoint3List m_vertices;   ///< surface vertices
      EigenPlane3     m_plane;      ///< infinite plane
      double          m_altitude;   ///< surface base altitude
      double          m_length;     ///< surface length
      double          m_height;     ///< surface height
      double          m_inclination;///< surface inclination angle from x-axis
      double          m_projection; ///< project origin on base 2D line
      EigenBBox3      m_bbox;       ///< clipping bounding box
      EigenMinMax2    m_thetas;     ///< horizontal apparent theta limits
      double          m_subtended;  ///< subtended angle of surface from viewer
    };

    /*!
     * \brief List (vector) of surfaces container type.
     */
    typedef std::vector<EigenSurface> EigenSurfaceList;

    /*! 
     * \brief Eigen scene object data type.
     *
     * Each object has a a set of rendering attributes (e.g. color, texture)
     * plus a set of surfaces. The set of surfaces typically specifiy a fully
     * connected, close shape (e.g. polyhedron), but this is not a requirement.
     *
     * A scene object must contain at least one surface.
     */
    struct EigenSceneObj
    {
      EigenRGBA         m_color;      ///< RGBA color attribute
      bool              m_hasCaps;    ///< object has top and bottom caps
      EigenBBox3        m_bbox;       ///< object bounding 3D box
      EigenPoint2List   m_footprint;  ///< object base footprint
      EigenSurfaceList  m_surfaces;   ///< the object surface properties
      /*!
       * \brief Clear scene object of surfaces and set attribute defaults.
       */
      void clear()
      {
        m_color   = FenceColorDft;
        m_hasCaps = false;
        m_footprint.clear();
        m_surfaces.clear();
      }
    };

    /*! 
     * \brief Eigen scene data type is a vector of scene objects.
     */
    typedef std::vector<EigenSceneObj> EigenScene;

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
       * Include only the nearest point of a set of intersections along a
       * traced ray.
       */
      ScanOptionNearest = 0x02,

      /*!
       * Alpha blend colors.
       */
      ScanOptionAlphaBlend = 0x04
    };

    ///@{
    /*!
     * \brief Useful scan option macros.
     *
     * \param _opt  Option bits
     *
     * \return Boolean
     */
    #define SCANOPT_2D(_opt)            ((_opt) & ScanOption2D)
    #define SCANOPT_ALPHA_BLEND(_opt)   ((_opt) & ScanOptionAlphaBlend)
    #define SCANOPT_NEAREST_ONLY(_opt)  ((_opt) & ScanOptionNearest)
    #define SCANOPT_XRAY_VIS(_opt)      !SCANOPT_NEAREST_ONLY(_opt)
    ///@}

    /*! \} */ // gfmath_scene


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Ouput types  
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \ingroup gfmath_scene
     * \defgroup gfmath_cloud Output Cloud Data Types
     * \brief Geofrenzy math calculated ouput data types.
     *
     * From Geofrenzy features, a synthetic set of depth plus color points 
     * are generated. These data points can be converted to standard set of
     * ROS messages to create virtual sensors such as point cloud structured
     * light sensors and laser scanners. 
     *
     * \{
     */

    /*! 
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

    /*! \} */ // gfmath_cloud

    /*!
     * \ingroup gfmath_scene
     * \{
     */

    /*!
     * \brief Create a scene object.
     *
     * \param polygon       ROS polygon defining a fence. Each (x,y,z) point
     *                      specifies a distance from a reference point or an
     *                      observer (meters).
     * \param color         Color attribute applied to the fence.
     *                      (red-green-blue intensities + alpha).
     * \param fenceAlt      Base altitude from ground (meters).
     * \param fenceHeight   Height of fence from base (meters).  
     * \param hasCaps       The object is a polyhedron. That is, it has both
     *                      top(ceiling) and bottom(floor) horizontal caps.
     * \param[out] sceneObj Created scene object.
     */
    void createSceneObj(const Polygon64 &polygon,
                        const EigenRGBA &color,
                        const double    &fenceAlt,
                        const double    &fenceHeight,
                        const bool      hasCaps,
                        EigenSceneObj   &sceneObj);

    /*!
     * \brief Scan virtual scene to generate a list of intersecting depth +
     * color points.
     *
     * In this operation mode, the scene is a posteriori detected by a virtual
     * scanning sensor.
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
     * \brief Grid virtual scene fences to generate a list of depth + color
     * points.
     *
     * In this operation mode, the scene is a priori known and a fast grid
     * of the fences is performed.
     *
     * \param       gridSize    Grid size.
     * \param       scene       The scene to scan.
     * \param[out]  intersects  List of intersecting points.
     * \param       options     Options to control the grid generations.
     */
    void gridScene(const double       gridSize,
                   const EigenScene   &scene,
                   EigenXYZRGBAList   &intersects,
                   uint32_t           options = ScanOptionDft);

    /*! \} */ // gfmath_scene

    ///@{
    /*!
     * \brief Stream insertion operators.
     *
     * \param os  Output stream.
     * \param arg Object to insert.
     *
     * \return Reference to output stream.
     */
    std::ostream &operator<<(std::ostream &os, const EigenPoint2 &pt);

    std::ostream &operator<<(std::ostream &os, const EigenPoint3 &pt);

    std::ostream &operator<<(std::ostream &os, const EigenRGBA &pt);

    std::ostream &operator<<(std::ostream &os, const EigenXYZRGB &pt);

    std::ostream &operator<<(std::ostream &os, const EigenXYZRGBA &pt);

    std::ostream &operator<<(std::ostream &os, const EigenMinMax2 &minmax);

    std::ostream &operator<<(std::ostream &os, const EigenMinMax3 &minmax);

    std::ostream &operator<<(std::ostream &os, const EigenSurface &surface);

    std::ostream &operator<<(std::ostream &os, const EigenSceneObj &sceneObj);
    ///@}

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Unit Tests
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

#undef GF_MATH_UT  ///< define/undef to enable/disable unit test functions.

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
