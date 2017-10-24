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
 * The geometric math and data structures supports basic geometry.
 *
 * For a more complete libray, see:\n
 *  Computational Geometry Algorithms Library (CGAL) https://www.cgal.org
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
#include <cassert>

#include <Eigen/Eigen>
#include <Eigen/Geometry>

namespace geofrenzy
{
  namespace gf_math
  {
    /*!
     * \defgroup gfmath Geofrenzy Math
     * 
     * The Eigen3 library data types and algorithms are used by the Geofrenzy
     * source. With ROS, Eigen3 is required, so no additional third-party
     * packages are required.
     * \{
     */

    //--------------------------------------------------------------------------
    // Math Data Types, Constants, and Defines
    //
    // Eigen geometric constructs in 2 and 3 dimensional ambient space, such as
    // point, line, and plane. Also color space
    //--------------------------------------------------------------------------

    #define M_TAU (2.0 * M_PI)  ///< tau is defined as 2 pi 

    /*!
     * \brief Units.
     */
    enum Units
    {
      UnitsUndef,     ///< undefined or "as is" units
      UnitsMeters,    ///< meters
      UnitsKm,        ///< kilometers
      UnitsRadians,   ///< radians
      UnitsDegrees    ///< degrees
    };

    /*!
     * \brief Coordinate systems.
     */
    enum Coordinates
    {
      Cartesian = 0,  ///< x,y or x,y,z coordinates
      Spherical,      ///< polar r,theta or spherical rho,theta,phi coordinates
      Cylindrical     ///< cylindrical r,theta,z coordinates
    };

    /*!
     * \brief Point in 2D space: 2x1 vector of doubles.
     */
    typedef Eigen::Vector2d EigenPoint2;

    /*!
     * \brief Point in 3D space: 3x1 vector of doubles.
     */
    typedef Eigen::Vector3d EigenPoint3;

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

    /*!
     * \brief 24-bit color, 8-bits per channel (rgb), values and mask.
     */
    const double        Color24ChanMin  =   0.0;    ///< minimum value
    const double        Color24ChanMax  = 255.0;    ///< maximum value
    const unsigned int  Color24ChanMask = 0x00ff;   ///< mask

    /*!
     * \brief Macro to convert color channel instensity to 8-bit RGB channel
     * value.
     *
     * \param _i  Channel intensity [0.0, 1.0].
     *
     * \return 8-bit color value [0, 255].
     */
    #define COLOR8(_i) \
      (((unsigned int)((_i) * Color24ChanMax)) & Color24ChanMask)

    /*!
     * \brief Macro to convert 8-bit RGB channel value to intensity.
     *
     * \param _c  8-bit color value [0, 255].
     *
     * \return Channel intensity [0.0, 1.0].
     */
    #define COLORI(_c) (((double)((_c) & Color24ChanMask)) / Color24ChanMax)

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
     * \brief One 2-tuple minimum,maximum limit.
     *
     * Limits: [m_min, m_max]
     */
    struct EigenMinMax1
    {
      double  m_min;  ///< minimum scalar value.
      double  m_max;  ///< maximum scalar value.
    };

    /*!
     * \brief Two 2-tuple minimum,maximum limits.
     *
     * Limits: [m_min[k], m_max[k]], k=0,1
     */
    struct EigenMinMax2
    {
      EigenPoint2 m_min;  ///< minimums k=0,1 or x,y
      EigenPoint2 m_max;  ///< maximums k=0,1 or x,y
    };

    /*!
     * \brief Rectangular boundary.
     */
    typedef EigenMinMax2 EigenBoundary2;

    /*!
     * \brief Two 3-tuple minimum,maximum limits.
     *
     * Limits: [m_min[k], m_max[k]], k=0,2
     */
    struct EigenMinMax3
    {
      EigenPoint3 m_min;  ///< minimum k=0,2 or x,y,z
      EigenPoint3 m_max;  ///< maximum k=0,2 or x,y,z
    };

    /*!
     * \brief Angle that an object subtends in ambient 2D space.
     */
    class EigenSubtend1
    {
    public:

      /*!
       * \brief Default constructor.
       */
      EigenSubtend1() : m_a0(0.0), m_omega(0.0) { }

      /*!
       * \brief Copy constructor.
       */
      EigenSubtend1(const EigenSubtend1 &src)
          : m_a0(src.m_a0), m_omega(src.m_omega) { }

      /*!
       * \brief Initialization constructor.
       */
      EigenSubtend1(const double a0, const double omega)
          : m_a0(a0), m_omega(omega) { }

      /*!
       * \brief Destructor.
       */
      ~EigenSubtend1() { }

      /*!
       * \brief Clear data.
       */
      void clear()
      {
        m_a0    = 0.0;
        m_omega = 0.0;
      }

      /*!
       * \brief Set subtended angle.
       * 
       * \param a0    Starting angle (radians).
       * \param omega Subtended angle ccw from a0 (radians).
       */
      void subtend(const double a0, const double omega)
      {
        m_a0    = a0;
        m_omega = omega;
      }

      /*!
       * \brief Set subtended angle.
       * 
       * \param subtend  Subtended angle.
       */
      void subtend(const EigenSubtend1 &subtend)
      {
        m_a0    = subtend.m_a0;
        m_omega = subtend.m_omega;
      }

      /*!
       * \brief Set starting angle.
       * 
       * \param a0  Starting angle (radians).
       */
      void a0(double a0)
      {
        m_a0 = a0;
      }

      /*!
       * \brief Set subtended angle.
       * 
       * \param omega Subtended angle ccw from a0 (radians).
       */
      void omega(double omega)
      {
        m_omega = omega;
      }

      /*!
       * Access functions.
       */
      double a0()                 { return m_a0; }
      const double a0() const     { return m_a0; }
      double omega()              { return m_omega; }
      const double omega() const  { return m_omega; }

      /*!
       * \brief Test if angle is within the subtended angle.
       *
       * Naive test that does not check rotation wrap.
       *
       * \param  a  Angle to check (radians).
       *
       * \return Returns true or false.
       */
      bool within(const double a) const
      {
        return (a >= m_a0) && (a <= m_a0 + m_omega);
      }

      /*!
       * \brief Stream insertion operators.
       *
       * \param os  Output stream.
       * \param obj Object to insert.
       *
       * \return Reference to output stream.
       */
      friend std::ostream &operator<<(std::ostream        &os,
                                      const EigenSubtend1 &obj);

      friend class EigenSubtend2;

    protected:
      double  m_a0;     ///< starting angle (radians)
      double  m_omega;  ///< subtended angle (radians)

    }; // class EigenSubtend1

    /*!
     * \brief Solid angle that an object subtends in ambient 3D space.
     *
     * Theta is the subtended angle from the +x axis specified as 
     * counter-clockwise range starting from a0.
     *
     * Phi is the subtended angle from the +z axis.
     */
    class EigenSubtend2
    {
    public:
      /*!
       * \brief Default constructor.
       */
      EigenSubtend2() { }

      /*!
       * \brief Copy constructor.
       */
      EigenSubtend2(const EigenSubtend2 &src)
          : m_theta(src.m_theta), m_phi(src.m_phi) { }

      /*!
       * \brief Destructor.
       */
      ~EigenSubtend2() { }

      /*!
       * \brief Clear data.
       */
      void clear()
      {
        m_theta.clear();
        m_phi.clear();
      }

      /*!
       * \brief Set theta subtended angle.
       * 
       * \param a0    Starting angle (radians).
       * \param omega Subtended angle ccw from a0 (radians).
       */
      void theta(const double a0, const double omega)
      {
        m_theta.m_a0    = a0;
        m_theta.m_omega = omega;
      }

      /*!
       * \brief Set theta subtended angle.
       * 
       * \param subtend  Subtended angle.
       */
      void theta(const EigenSubtend1 &subtend)
      {
        m_theta.m_a0    = subtend.m_a0;
        m_theta.m_omega = subtend.m_omega;
      }

      /*!
       * \brief Set phi subtended angle.
       * 
       * \param a0    Starting angle (radians).
       * \param omega Subtended angle downward from a0 (radians).
       */
      void phi(const double a0, const double omega)
      {
        m_phi.m_a0    = a0;
        m_phi.m_omega = omega;
      }

      /*!
       * \brief Set theta subtended angle.
       * 
       * \param subtend  Subtended angle.
       */
      void phi(const EigenSubtend1 &subtend)
      {
        m_phi.m_a0    = subtend.m_a0;
        m_phi.m_omega = subtend.m_omega;
      }

      /*!
       * Access functions.
       */
      EigenSubtend1 &theta()             { return m_theta; }
      const EigenSubtend1 &theta() const { return m_theta; }
      EigenSubtend1 &phi()               { return m_phi; }
      const EigenSubtend1 &phi() const   { return m_phi; }

      /*!
       * \brief Test if (theta,phi) are within the solid angle.
       *
       * \param  theta  Azimuthal angle from x+ axis (-pi, pi].
       * \param  phi    Polar angle from +z [0, pi].
       *
       * \return Returns true or false.
       */
      bool within(const double theta, const double phi) const;

      /*!
       * \brief Stream insertion operators.
       *
       * \param os  Output stream.
       * \param obj Object to insert.
       *
       * \return Reference to output stream.
       */
      friend std::ostream &operator<<(std::ostream        &os,
                                      const EigenSubtend2 &obj);

    protected:
      EigenSubtend1 m_theta;  ///< subtended azimuthal angle from +x axis.
      EigenSubtend1 m_phi;    ///< subtended polar angle from +z.

    }; // class EigenSubtend2

    /*!
     * \brief Rectangular cuboid boundary.
     */
    typedef EigenMinMax3 EigenBoundary3;

    /*!
     * \brief List (vector) of 2D points.
     */
    typedef std::vector<EigenPoint2> EigenPoint2List;

    /*! 
     * \brief List (vector) of 3D points.
     */
    typedef std::vector<EigenPoint3> EigenPoint3List;

    /*! 
     * \brief List of depth plus RGB insensity points.
     *
     * The list may or may not be an 2D ordered list and may contain multiple
     * collinear points for the same (x,y).
     */
    typedef std::vector<EigenXYZRGB> EigenXYZRGBList;

    /*! 
     * \brief List of depth plus RGBA insensity points.
     *
     * The list may or may not be an 2D ordered list and may contain multiple
     * collinear points for the same (x,y).
     */
    typedef std::vector<EigenXYZRGBA> EigenXYZRGBAList;

    /*!
     * \brief Infinity
     */
    const double Inf = std::numeric_limits<double>::infinity(); ///< infinity

    /*!
     * \brief Points at infinity.
     */
    const EigenPoint2 Inf2(Inf, Inf);           ///< ambient 2D space
    const EigenPoint3 Inf3(Inf, Inf, Inf);      ///< ambient 3D space

    /*!
     * \brief Origins
     */
    const EigenPoint2 Origin2(0.0, 0.0);        ///< ambient 2D space
    const EigenPoint3 Origin3(0.0, 0.0, 0.0);   ///< ambient 3D space

    /*!
     * \brief Unit vectors or versors (hat == ^).
     */
    const EigenPoint2 Ihat2(1.0, 0.0);          ///< x unit vector in 2D
    const EigenPoint2 Jhat2(0.0, 1.0);          ///< y unit vector in 2D
    const EigenPoint3 Ihat3(1.0, 0.0, 0.0);     ///< x unit vector in 3D
    const EigenPoint3 Jhat3(0.0, 1.0, 0.0);     ///< y unit vector in 3D
    const EigenPoint3 Khat3(0.0, 0.0, 1.0);     ///< z unit vector in 3D

    /*!
     * \brief Default precision.
     *
     * This default precision is somewhat optimized for location technologies,
     * which have at best, sub-millimeter linear resolutions.
     */
    const double PrecisionDft = 1.0e-5;

    //
    // Useful Indices
    //
    namespace gf_index
    {
      /*!
       * \brief Cartesian x,y,z coordinates indices.
       *
       * \sa EigenPoint2(_X, _Y only), EigenPoint3, EigenXYZRGB, EigenXYZRGBA
       */
      const size_t  _X = 0; ///< x coordinate pt[_X] or pt.x()
      const size_t  _Y = 1; ///< y coordinate pt[_Y] or pt.y()
      const size_t  _Z = 2; ///< z coordinate pt[_Z] or pt.z()
    
      /*!
       * \brief Polar and spherical coordinates indices.
       *
       * Polar:     pt[_R], pt[_THETA]
       * Spherical: pt[_RHO], pt[_THETA], pt[_PHI]
       *
       * \sa EigenPoint2(_R, _THETA only), EigenPoint3
       */
      const size_t  _R      = 0;  ///< radial distance [0, inf)
      const size_t  _RHO    = 0;  ///< radial distance [0, inf)
      const size_t  _THETA  = 1;  ///< azimuthal angle from +x axis (-pi, pi].
      const size_t  _PHI    = 2;  ///< polar angle from z+ [0, pi].
  
      /*!
       * \brief Size dimensions length x width x height.
       *
       * \sa EigenPoint2(_L, _W only), EigenPoint3
       */
      const size_t  _L = 0; ///< x coordinate pt[_L] or pt.x()
      const size_t  _W = 1; ///< y coordinate pt[_W] or pt.y()
      const size_t  _H = 2; ///< z coordinate pt[_H] or pt.z()
  
      /*!
       * \brief Color red-green-blue-alpha indices.
       *
       * \sa EigenRGB, EigenRGBA
       */
      const size_t  _RED    = 0; ///< red   pt[_RED]   or pt.x()
      const size_t  _GREEN  = 1; ///< green pt[_GREEN] or pt.y()
      const size_t  _BLUE   = 2; ///< blue  pt[_BLUE]  or pt.z()
      const size_t  _ALPHA  = 3; ///< alpha pt[_ALPHA] or pt.w()
    
      /*!
       * \brief Color red-green-blue-alpha indices for XYZ+ points.
       *
       * \sa EigenXYZRGB, EigenXYZRGBA
       */
      const size_t  _XYZRED   = 3; ///< xyzred
      const size_t  _XYZGREEN = 4; ///< xyzgreen
      const size_t  _XYZBLUE  = 5; ///< xyzblue
      const size_t  _XYZALPHA = 6; ///< xyzalpha

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
      const int _ONone  = 0;    ///< no octant - on an axes plane
      const int _OI     = 1;    ///< octant 1: x+, y+, z+
      const int _OII    = 2;    ///< octant 2: x-, y+, z+
      const int _OIII   = 3;    ///< octant 3: x-, y-, z+
      const int _OIV    = 4;    ///< octant 4: x+, y-, z+
      const int _OV     = 5;    ///< octant 5: x+, y+, z-
      const int _OVI    = 6;    ///< octant 6: x-, y+, z-
      const int _OVII   = 7;    ///< octant 7: x-, y-, z-
      const int _OVIII  = 8;    ///< octant 8: x+, y-, z-

    } // namespace gf_index


    //--------------------------------------------------------------------------
    // Print and Insertion Operators
    //--------------------------------------------------------------------------

    //
    // Print helpers.
    //
    extern int OIndent;    ///< stream output indentation

    /*!
     * \brief Set left indentation level.
     *
     * \param n Indentation level in number of spaces.
     */
    inline void setIndent(int n)
    {
      OIndent = n;
    }

    /*!
     * \brief Bump left indentation level.
     *
     * \param n +-Delta indentation level.
     *
     * \return New indentation level.
     */
    inline int bumpIndent(int n)
    {
      OIndent += n;
      return OIndent;
    }

    /*!
     * \brief Indent by current indentation level.
     *
     * \reutrn Space string.
     */
    std::string indent();

    ///@{
    /*!
     * \brief Print object.
     *
     * Note:  Since many of the math object types are simple typedefs of
     *        Eigen types, the Eigen insertion operators << can take precedence.
     *        Use the print versions, if needed, to force desired output.
     *
     * \param os  Output stream.
     * \param obj Object to print.
     */
    void print(std::ostream &os, const EigenPoint2 &obj);

    void print(std::ostream &os, const EigenPoint3 &obj);

    void print(std::ostream &os, const EigenLine2 &obj);

    void print(std::ostream &os, const EigenLine3 &obj);

    void print(std::ostream &os, const EigenPlane3 &obj);

    void print(std::ostream &os, const EigenRGBA &obj);

    void print(std::ostream &os, const EigenXYZRGB &obj);

    void print(std::ostream &os, const EigenXYZRGBA &obj);

    void print(std::ostream &os, const EigenMinMax2 &obj);

    void print(std::ostream &os, const EigenMinMax3 &obj);
    ///@}


    /*!
     * \brief Print point list.
     * 
     * If conversion units are specfied, then the points are converted to the
     * specified units prior to printing. Currently on radian-degree conversion
     * is supported.
     *
     * If indentation is true, then the current indentation level is used.
     *
     * \param os      Output stream.
     * \param points  Points to print.
     * \param mod     Points printed per line.
     * \param cvt     Conversion units.
     * \param bIndent Do [not] used current indentation level.
     */
    void print(std::ostream          &os,
               const EigenPoint3List &points,
               const size_t          mod, 
               const Units           cvt = UnitsUndef,
               const bool            bIndent = true);

    ///@{
    /*!
     * \brief Stream insertion operators.
     *
     * \param os  Output stream.
     * \param obj Object to insert.
     *
     * \return Reference to output stream.
     */
    inline std::ostream &operator<<(std::ostream &os, const EigenPoint2 &obj)
    {
      print(os, obj);
      return os;
    }

    inline std::ostream &operator<<(std::ostream &os, const EigenPoint3 &obj)
    {
      print(os, obj);
      return os;
    }

    inline std::ostream &operator<<(std::ostream &os, const EigenLine2 &obj)
    {
      print(os, obj);
      return os;
    }

    inline std::ostream &operator<<(std::ostream &os, const EigenLine3 &obj)
    {
      print(os, obj);
      return os;
    }

    inline std::ostream &operator<<(std::ostream &os, const EigenPlane3 &obj)
    {
      print(os, obj);
      return os;
    }

    inline std::ostream &operator<<(std::ostream &os, const EigenRGBA &obj)
    {
      print(os, obj);
      return os;
    }

    inline std::ostream &operator<<(std::ostream &os, const EigenXYZRGB &obj)
    {
      print(os, obj);
      return os;
    }

    inline std::ostream &operator<<(std::ostream &os, const EigenXYZRGBA &obj)
    {
      print(os, obj);
      return os;
    }

    inline std::ostream &operator<<(std::ostream &os, const EigenMinMax2 &obj)
    {
      print(os, obj);
      return os;
    }

    inline std::ostream &operator<<(std::ostream &os, const EigenMinMax3 &obj)
    {
      print(os, obj);
      return os;
    }
    ///@}


    //--------------------------------------------------------------------------
    // Basic Math and Geometric Operations
    //--------------------------------------------------------------------------
    
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Boot Camp
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

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
     * \brief Minimum of two values.
     *
     * \param val0  Value 0.
     * \param val1  Value 1.
     *
     * \return Minimum value.
     */
    inline double min(const double val0, const double val1)
    {
      return val0 <= val1? val0: val1;
    }

    /*!
     * \brief Maximum of two values.
     *
     * \param val0  Value 0.
     * \param val1  Value 1.
     *
     * \return Maximum value.
     */
    inline double max(const double val0, const double val1)
    {
      return val0 >= val1? val0: val1;
    }

    /*!
     * \brief Determine sign of value.
     *
     * Zero is considered positive.
     *
     * \param val   Value to test.
     *
     * \return Returns 1 or -1 depending if value is non-negative or negative,
     * respectively.
     */
    inline int sign(const double val)
    {
      return val >= 0.0? 1: -1;
    }

    /*!
     * \brief Calculate the hypotenuse c from right triangle with length of
     * sides a and b.
     *
     * \param a  Length of side a.
     * \param b  Length of side b.
     *
     * \return Length of c.
     */
    inline double pythagorean(const double a, const double b)
    {
      return sqrt(pow(fabs(a), 2.0) + pow(fabs(b), 2.0));
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
    
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Angles
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Convert degrees to radians.
     *
     * \param degrees Scalar in degrees.
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
     * \param radians  Scalar in radians.
     *
     * \return Degrees
     */
    inline double degrees(const double radians)
    {
      return 180.0 * radians / M_PI;
    }

    /*!
     * \brief Convert radian limits to degrees.
     *
     * \param lim   Minimum, maximum range limit in radians.
     *
     * \return Minimum,maximum in degrees.
     */
    inline EigenMinMax1 degrees(const EigenMinMax1 &lim)
    {
      EigenMinMax1  mM;

      mM.m_min = degrees(lim.m_min);
      mM.m_max = degrees(lim.m_max);

      return mM;
    }

    /*!
     * \brief Convert radian limits to degrees.
     *
     * \param lim   Two 2-tuple minimum,maximum range limits in radians.
     *
     * \param Equivalent ranges in degrees.
     */
    inline EigenMinMax2 degrees(const EigenMinMax2 &lim)
    {
      EigenMinMax2 mM;

      mM.m_min << degrees(lim.m_min.x()), degrees(lim.m_min.y());
      mM.m_max << degrees(lim.m_max.x()), degrees(lim.m_max.y());

      return mM;
    }

    /*!
     * \brief Convert radian subtended angle to degrees.
     *
     * \param subtend   Subtended angle (a0, omega).
     *
     * \param Equivalent subtended in degrees.
     */
    inline EigenSubtend1 degrees(const EigenSubtend1 &subtend)
    {
      EigenSubtend1 s;

      s.subtend(degrees(subtend.a0()), degrees(subtend.omega()));

      return s;
    }

    /*!
     * \brief Convert solid subtended angle in radians to degrees.
     *
     * \param subtend   Subtended angle [theta, phi].
     *
     * \param Equivalent subtended in degrees.
     */
    inline EigenSubtend2 degrees(const EigenSubtend2 &subtend)
    {
      EigenSubtend2 s;

      s.theta(degrees(subtend.theta()));
      s.phi(degrees(subtend.phi()));

      return s;
    }

    /*!
     * \brief Convert polar point specified in radians to degrees.
     *
     * \param rt  Polar point r,theta in radians.
     *
     * \param Equivalent polar point in degrees.
     */
    inline EigenPoint2 degrees(const EigenPoint2 &rt)
    {
      return EigenPoint2(rt[gf_index::_R], degrees(rt[gf_index::_THETA]));
    }

    /*!
     * \brief Convert spherical point specified in radians to degrees.
     *
     * \param rtp   Spherical point rho,theta,phi in radians.
     *
     * \param Equivalent spherical point in degrees.
     */
    inline EigenPoint3 degrees(const EigenPoint3 &rtp)
    {
      return EigenPoint3(rtp[gf_index::_RHO],
                        degrees(rtp[gf_index::_THETA]),
                        degrees(rtp[gf_index::_PHI]));
    }

    /*!
     * \brief Convert spherical min,max range specified in radians to degrees.
     *
     * \param rtpMinMax   Spherical range in radians.
     *
     * \param Equivalent spherical min,max range in degrees.
     */
    inline EigenMinMax3 degrees(const EigenMinMax3 &rtpMinMax)
    {
      EigenMinMax3  mM;

      mM.m_min = degrees(rtpMinMax.m_min);
      mM.m_max = degrees(rtpMinMax.m_max);

      return mM;
    }

    /*!
     * \brief Remap angle in (-pi, pi] to equivalent value in [0, tau) range.
     *
     * \param a   Angle (radians).
     *
     * \return Equivalent angle in [0, 2pi).
     */
    inline double zero2tau(const double a)
    {
      return fmod(a+M_TAU, M_TAU);
    }

    /*!
     * \brief Remap angle in (-2pi, 2pi] to equivalent value in (-pi, pi] range.
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
     * \brief Add two angles keeping result in (-pi, pi].
     *
     * \param a0  Angle 0 in radians.
     * \param a1  Angle 1 in radians.
     *
     * \return Added angles in radians (-pi, pi].
     */
    static inline double addAngles(const double a0, const double a1)
    {
      return pi2pi(a0 + a1);
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
     * \brief Take the supplementary angle of the given angle in radians.
     * 
     * \param a   Angle in radians (-pi, pi]
     * 
     * \return Supplimentary angle in radians (-pi, pi].
     */
    inline double supplementary(const double a)
    {
      return a >= 0.0? M_PI - a: -M_PI - a;
    }


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Norms and Metric Distances
    //
    // The Lp norms and distances supported are:
    //  * L1        - Sum of absolutes. A.K.A rectilinear or taxicab.norm.
    //  * squaredL2 - Sum of squares. L2 without the square root operation.
    //  * L2        - Square root of sum of squares. A.K.A Euclidean norm.
    //  * Linf      - Maximum absolute component. Infinity A.K.A maximum norm.
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Calculate the L1 absolute distance of scalars.
     *
     * For scalars, L1 == L2 == Linf
     *
     * \note Catastrophic cancellation may occur for nearly identical values.
     *
     * \param s0  Scalar value 0.
     * \param s1  Scalar value 1.
     *
     * \return Distance >= 0.
     */
    inline double L1Dist(const double s0, const double s1)
    {
      return fabs(s1 - s0);
    }

    /*!
     * \brief Calculate the L1 norm of a point in R2.
     *
     * \note Catastrophic cancellation may occur for nearly identical values.
     *
     * \param pt  Point.
     *
     * \return Norm.
     */
    inline double L1Norm(const EigenPoint2 &pt)
    {
      return fabs(pt.x()) + fabs(pt.y());
    }

    /*!
     * \brief Calculate the L1 absolute distance between two points in R2.
     *
     * \note Catastrophic cancellation may occur for nearly identical values.
     *
     * \param pt0 Point 0.
     * \param pt1 Point 1.
     *
     * \return Distance >= 0.
     */
    inline double L1Dist(const EigenPoint2 &pt0, const EigenPoint2 &pt1)
    {
      return fabs(pt1.x() - pt0.x()) + fabs(pt1.y() - pt0.y());
    }

    /*!
     * \brief Calculate the L1 norm of a point in R3.
     *
     * \param pt  Point.
     *
     * \return Norm.
     */
    inline double L1Norm(const EigenPoint3 &pt)
    {
      return fabs(pt.x()) + fabs(pt.y() + fabs(pt.z()));
    }

    /*!
     * \brief Calculate the L1 absolute distance between two points in R3.
     *
     * \note Catastrophic cancellation may occur for nearly identical values.
     *
     * \param pt0 Point 0.
     * \param pt1 Point 1.
     *
     * \return Distance >= 0.
     */
    inline double L1Dist(const EigenPoint3 &pt0, const EigenPoint3 &pt1)
    {
      return  fabs(pt1.x() - pt0.x()) +
              fabs(pt1.y() - pt0.y()) +
              fabs(pt1.z() - pt0.z());
    }

    /*!
     * \brief Calculate the squared L2 distance of scalars.
     *
     * \note Catastrophic cancellation may occur for nearly identical values.
     *
     * \param s0  Scalar value 0.
     * \param s1  Scalar value 1.
     *
     * \return Distance >= 0.
     */
    inline double squaredL2Dist(const double s0, const double s1)
    {
      return pow(fabs(s1-s0), 2.0);
    }

    /*!
     * \brief Calculate the squared L2 norm of a point in R2.
     *
     * \param pt  Point.
     *
     * \return Norm.
     */
    inline double squaredL2Norm(const EigenPoint2 &pt)
    {
      return pt.squaredNorm();
    }

    /*!
     * \brief Calculate the squared L2 distance between two points in R2.
     *
     * \note Catastrophic cancellation may occur for nearly identical values.
     *
     * \param pt0 Point 0.
     * \param pt1 Point 1.
     *
     * \return Distance >= 0.
     */
    inline double squaredL2Dist(const EigenPoint2 &pt0, const EigenPoint2 &pt1)
    {
      return squaredL2Norm(EigenPoint2(pt1 - pt0));
    }

    /*!
     * \brief Calculate the squared L2 norm of a point in R3.
     *
     * \param pt  Point.
     *
     * \return Norm.
     */
    inline double squaredL2Norm(const EigenPoint3 &pt)
    {
      return pt.squaredNorm();
    }

    /*!
     * \brief Calculate the squared L2 distance between two points in R3.
     *
     * \note Catastrophic cancellation may occur for nearly identical values.
     *
     * \param pt0 Point 0.
     * \param pt1 Point 1.
     *
     * \return Distance >= 0.
     */
    inline double squaredL2Dist(const EigenPoint3 &pt0, const EigenPoint3 &pt1)
    {
      return squaredL2Norm(EigenPoint3(pt1 - pt0));
    }

    /*!
     * \brief Calculate the L2 norm of a point in R2.
     *
     * \note The Blue's L2 norm algorithm avoids underflow and overflow.
     *
     * \param pt  Point.
     *
     * \return Norm.
     */
    inline double L2Norm(const EigenPoint2 &pt)
    {
      return pt.blueNorm();
    }

    /*!
     * \brief Calculate the L2 absolute distance between two points in R2.
     *
     * \note Catastrophic cancellation may occur for nearly identical values.
     *
     * \param pt0   Point 0.
     * \param pt1   Point 1.
     *
     * \return Distance >= 0.
     */
    inline double L2Dist(const EigenPoint2 &pt0, const EigenPoint2 &pt1)
    {
      return sqrt(squaredL2Dist(pt0, pt1));
    }

    /*!
     * \brief Calculate the L2 norm of a point in R3.
     *
     * \note The Blue's L2 norm algorithm avoids underflow and overflow.
     *
     * \param pt  Point.
     *
     * \return Norm.
     */
    inline double L2Norm(const EigenPoint3 &pt)
    {
      return pt.blueNorm();
    }

    /*!
     * \brief Calculate the L2 absolute distance between two points in R3.
     *
     * \note Catastrophic cancellation may occur for nearly identical values.
     *
     * \param pt0   Point 0.
     * \param pt1   Point 1.
     *
     * \return Distance >= 0.
     */
    inline double L2Dist(const EigenPoint3 &pt0, const EigenPoint3 &pt1)
    {
      return sqrt(squaredL2Dist(pt0, pt1));
    }

    /*!
     * \brief Calculate the Linf norm of a point in R2.
     *
     * \param pt  Point.
     *
     * \return Norm.
     */
    inline double LInfNorm(const EigenPoint2 &pt)
    {
      return fabs(pt.maxCoeff());
    }

    /*!
     * \brief Calculate the Linf absolute distance between two points in R2.
     *
     * \note Catastrophic cancellation may occur for nearly identical values.
     *
     * \param pt0   Point 0.
     * \param pt1   Point 1.
     *
     * \return Distance >= 0.
     */
    inline double LInfDist(const EigenPoint2 &pt0, const EigenPoint2 &pt1)
    {
      return LInfNorm(EigenPoint2(pt1 - pt0));
    }

    /*!
     * \brief Calculate the Linf norm of a point in R3.
     *
     * \param pt  Point.
     *
     * \return Norm.
     */
    inline double LInfNorm(const EigenPoint3 &pt)
    {
      return fabs(pt.maxCoeff());
    }

    /*!
     * \brief Calculate the Linf absolute distance between two points in R3.
     *
     * \note Catastrophic cancellation may occur for nearly identical values.
     *
     * \param pt0   Point 0.
     * \param pt1   Point 1.
     *
     * \return Distance >= 0.
     */
    inline double LInfDist(const EigenPoint3 &pt0, const EigenPoint3 &pt1)
    {
      return LInfNorm(EigenPoint3(pt1 - pt0));
    }


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Approximates
    //
    // Stand back! DC may have the Justice League and Marvel the Avengers,
    // but lo, here be the Approximators!
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Test if values are approximately equal.
     *
     * \param s0        Scalar value 0.
     * \param s1        Scalar value 1.
     * \param precision Precision of equality.
     *
     * \return Returns true or false.
     */
    inline bool isApprox(const double s0,
                         const double s1,
                         const double precision = PrecisionDft)
    {
      return L1Dist(s0, s1) <= precision;
    }

    /*!
     * \brief Test if value are approximately equal to 0 (zero).
     *
     * \param s         Scalar value to test.
     * \param precision Precision of equality.
     *
     * \return Returns true or false.
     */
    inline bool isApproxZero(const double s,
                             const double precision = PrecisionDft)
    {
      return fabs(s) <= precision;
    }

    /*!
     * \brief Test if two points, within precision, are approximately equal.
     *
     * The L1 distance is used.
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
      return L1Dist(pt0, pt1) <= precision;
    }

    /*!
     * \brief Test if two points, within precision, are approximately equal.
     *
     * The L1 distance is used.
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
      return L1Dist(pt0, pt1) <= precision;
    }


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Coordinate System Converters
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    ///@{
    /*!
     * \brief Convert polar coordinate (r,theta) to Cartesion (x,y).
     *
     * The polar coordinates are as used in mathematics.
     *
     * \param r     Radial distance [0, inf).
     * \param theta Azimuthal angle from x+ axis (-pi, pi].
     * \param pt    Point in polar coordinates.
     *
     * \return Cartesion (x,y) point.
     */
    inline EigenPoint2 polarToCartesian(const double r, const double theta)
    {
      return EigenPoint2(r * cos(theta), r * sin(theta));
    }

    inline EigenPoint2 polarToCartesian(const EigenPoint2 &pt)
    {
      return polarToCartesian(pt[gf_index::_R], pt[gf_index::_THETA]);
    }
    ///@}

    ///@{
    /*!
     * \brief Convert Cartesian coordinate (x,y) to polar (r,theta).
     *
     * The polar coordinates are as used in mathematics with:
     *  - r is the radial distance [0, inf) from the origin
     *  - theta is the azimuthal angle from the x+ axis (-pi, pi].
     *
     * \param pt    Cartesian point (x,y).
     * \param x     X coordinate.
     * \param y     Y coordinate.
     *
     * \return Polar point (r,theta).
     */
    inline EigenPoint2 cartesianToPolar(const EigenPoint2 &pt)
    {
      return EigenPoint2(L2Norm(pt), atan2(pt.y(), pt.x()));
    }

    inline EigenPoint2 cartesianToPolar(const double x, const double y)
    {
      return cartesianToPolar(EigenPoint2(x, y));
    }
    ///@}

    ///@{
    /*!
     * \brief Convert spherical coordinate (r,theta,phi) to Cartesion (x,y,z).
     *
     * The Sperical coordinates are as used in mathematics.
     *
     * \param r     Radial distance [0, inf).
     * \param theta Azimuthal angle from x+ axis (-pi, pi].
     * \param phi   Polar angle from z+ [0, pi].
     * \param pt    Point in spherical coordinates.
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

    inline EigenPoint3 sphericalToCartesian(const EigenPoint3 &pt)
    {
      return sphericalToCartesian(pt[gf_index::_RHO],
                                  pt[gf_index::_THETA],
                                  pt[gf_index::_PHI]);
    }
    ///@}
    
    ///@{
    /*!
     * \brief Convert Cartesian coordinate (x,y,z) to spherical (r,theta,phi).
     *
     * The Sperical coordinates are as used in mathematics.
     *
     * \param pt    Cartesion (x,y,z) point.
     * \param x     X coordinate.
     * \param y     Y coordinate.
     * \param z     Z coordinate.
     *
     * \return Spherical coordinate (r,theta,phi) point.
     */
    EigenPoint3 cartesianToSpherical(const EigenPoint3 &pt);

    inline EigenPoint3 cartesianToSpherical(const double x,
                                            const double y,
                                            const double z)
    {
      return cartesianToSpherical(EigenPoint3(x, y, z));
    }
    ///@}


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Point - Line Geometry
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

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
     * \return Projection L2 distance >= 0.0
     */
    inline double projectionDist(const EigenPoint2 &pt1, const EigenPoint2 &pt2)
    {
      return fabs(pt2.x() * pt1.y() - pt2.y() * pt1.x()) /
              sqrt(pow(pt2.y() - pt1.y(), 2.0) + pow(pt2.x() - pt1.x(), 2.0));
    }

    /*!
     * \brief Calculate the distance from a point pt0 to a line in ambient 2D
     * space.
     *
     * The 2D line is defined by the two points pt1 and pt2.
     *
     * The projection on the line pt1,pt2 of the point pt0 forms a point on the
     * line that defines an orthogonal line between the projection and point
     * pt0.
     *
     * \param pt0   Point to project.
     * \param pt1   Line point 1.
     * \param pt2   Line point 2.
     *
     * \return Projection L2 distance >= 0.
     */
    inline double projectionDist(const EigenPoint2 &pt0,
                                 const EigenPoint2 &pt1,
                                 const EigenPoint2 &pt2)
    {
      double dx = pt2.x() - pt1.x();
      double dy = pt2.y() - pt1.y();

      return fabs(dy*pt0.x() - dx*pt0.y() + pt2.x()*pt1.y() - pt2.y()*pt1.x()) /
              sqrt(pow(dy, 2.0) + pow(dx, 2.0));
    }

    /*!
     * \brief Calculate the the point pt0 onto a line in ambient 3D * space.
     *
     * The 3D line is defined by the two points pt1 and pt2.
     *
     * The projection on the line pt1,pt2 of the point pt0 forms a point on the
     * line that defines an orthogonal line between the projection and point
     * pt0.
     *
     * \param pt0   Point to project.
     * \param pt1   Line point 1.
     * \param pt2   Line point 2.
     *
     * \return Project point on the line.
     */
    inline EigenPoint3 projection(const EigenPoint3 &pt0,
                                  const EigenPoint3 &pt1,
                                  const EigenPoint3 &pt2)
    {
      EigenLine3 line = EigenLine3::Through(pt1, pt2);

      return line.projection(pt0);
    }

    inline EigenPoint3 projection(const EigenPoint3 &pt1,
                                  const EigenPoint3 &pt2)
    {
      return projection(Origin3, pt1, pt2);
    }

    /*!
     * \brief Calculate the inclination angle of a line in ambient 2D space.
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
     * \brief Find the intersection of two parametric lines in ambient 2D space.
     *
     * \param line1   Parametric line 1 line1(t) = o1 + t * d1
     * \param line2   Parametric line 2 line2(u) = o2 + u * d2
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
     * First x, then the y component is tried. No consistency checks are made.
     *
     * \param line  Parametric line in ambient 3D space.
     * \param pt    Point in the x-y plane.
     *
     * \return Returns t or Inf.
     */
    double t_param(const EigenLine3 &line, const EigenPoint2 &pt);

    /*!
     * \brief Determine the x-intercept of a line defined by two points in
     * ambient 2D space.
     *
     * \param pt0   Line point 0.
     * \param pt1   Line point 1.
     *
     * \returns Returns the x-intercept. If the line does not intersect the
     * x-axis, Inf is returned.
     */
    double x_intercept(const EigenPoint2 &pt0, const EigenPoint2 &pt1);

    /*!
     * \brief Determine the y-intercept of a line defined by two points in
     * ambient 2D space.
     *
     * \param pt0   Line point 0.
     * \param pt1   Line point 1.
     *
     * \returns Returns the y-intercept. If the line does not intersect the
     * y-axis, Inf is returned.
     */
    double y_intercept(const EigenPoint2 &pt0, const EigenPoint2 &pt1);


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Checks on Limits, Locations, and Containment
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Determine which x-y quadrant the point is located.
     *
     * \param pt        2D point.
     * \param precision Precision of position.
     *
     * \return  Returns 1-4 for quadrants QI - QIV, respectively.
     *          Returns 0 if the point lies sufficiently close to an axis.
     */
    int quadrant(const EigenPoint2 &pt, const double precision = PrecisionDft);

    /*!
     * \brief Determine which x-y quadrant the ray from the origin is located.
     *
     * \param theta     Azimuthal angle of ray from x+ axis (-pi, pi].
     * \param precision Precision of angle.
     *
     * \return  Returns 1-4 for quadrants QI - QIV, respectively.
     *          Returns 0 if angle lies sufficiently close to an axis.
     */
    int quadrant(const double theta, const double precision = PrecisionDft);

    /*!
     * \brief Determine which x-y-z octant the point is located.
     *
     * \param pt        3D point.
     * \param precision Precision of position.
     *
     * \return  Returns 1-8 for octants OI - OVII, respectively.
     *          Returns 0 if point lies sufficiently close to an axes plane.
     */
    int octant(const EigenPoint3 &pt, const double precision = PrecisionDft);

    /*!
     * \brief Determine which x-y-z octant the ray from the origin is located.
     *
     * \param pt        3D point.
     * \param theta     Azimuthal angle of ray from x+ axis (-pi, pi].
     * \param phi       Polar angle of ray from z+ [0, pi].
     * \param precision Precision of position.
     *
     * \return  Returns 1-8 for octants OI - OVII, respectively.
     *          Returns 0 if point lies sufficiently close to an axes plane.
     */
    int octant(const double theta,
               const double phi,
               const double precision = PrecisionDft);

    /*!
     * \brief Check if the value is within the min,max limits.
     *
     * \param val   Value to check.
     * \param min   Scalar minimum value.
     * \param max   Scalar maximum value.
     *
     * \return Returns true or false if value in [min, max].
     */
    inline bool within(const double &val, const double min, const double max)
    {
      return  (val >= min) && (val <= max);
    }

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
      return  (val >= lim.m_min) && (val <= lim.m_max);
    }

    /*!
     * \brief Check if the value is within one of the set of min,max limits.
     *
     * \param val   Value to check.
     * \param lim   Limits.
     *
     * \return Returns true or false if value in any [min[k], max[k]], k=0,1.
     */
    inline bool withinOneOf(const double &val, const EigenMinMax2 &lim)
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
     * \return Returns true or false if value in any [min[k], max[k]], k=0,2.
     */
    inline bool withinOneOf(const double &val, const EigenMinMax3 &lim)
    {
      return  ((val >= lim.m_min[0]) && (val <= lim.m_max[0])) ||
              ((val >= lim.m_min[1]) && (val <= lim.m_max[1])) ||
              ((val >= lim.m_min[2]) && (val <= lim.m_max[2]));
    }

    ///@{
    /*!
     * \brief Seed boundary with initial values.
     *
     * \param pt      Point value.
     * \param bounds  Bounding area.
     */
    inline void seedBounds(const EigenPoint2 &seed, EigenBoundary2 &bounds)
    {
      bounds.m_min[0] = bounds.m_max[0] = seed[0];
      bounds.m_min[1] = bounds.m_max[1] = seed[1];
    }

    inline void seedBounds(const EigenPoint3 &seed, EigenBoundary3 &bounds)
    {
      bounds.m_min[0] = bounds.m_max[0] = seed[0];
      bounds.m_min[1] = bounds.m_max[1] = seed[1];
      bounds.m_min[2] = bounds.m_max[2] = seed[2];
    }
    ///@}

    ///@{
    /*!
     * \brief Conditionally expand bounds to include the point.
     *
     * \param pt      Point to include.
     * \param bounds  Bounding area.
     */
    void growBounds(const EigenPoint2 &pt, EigenBoundary2 &bounds);

    // conditionally expand bounds to include point
    void growBounds(const EigenPoint3 &pt, EigenBoundary3 &bounds);
    ///@}

    ///@{
    /*!
     * \brief Uniformally expand the bounds by epsilong
     *
     * \param epsilon Epsilon expansion.
     * \param bounds  Bounding area.
     *
     * \return Returns true or false.
     */
    // expane bounds by epsilon
    void growBounds(const double epsilon, EigenBoundary2 &bounds);

    // expane bounds by epsilon
    void growBounds(const double epsilon, EigenBoundary3 &bounds);
    ///@}

    ///@{
    /*!
     * \brief Test if point is in bounds.
     *
     * \param pt      Point to test.
     * \param bounds  Bounding area.
     *
     * \return Returns true or false.
     */
    inline bool inbounds(const EigenPoint2 &pt, const EigenBoundary2 &bounds)
    {
      return  (pt.x() >= bounds.m_min.x()) && (pt.x() <= bounds.m_max.x()) &&
              (pt.y() >= bounds.m_min.y()) && (pt.y() <= bounds.m_max.y());
    }

    inline bool inbounds(const EigenPoint3 &pt, const EigenBoundary3 &bounds)
    {
      return  (pt.x() >= bounds.m_min.x()) && (pt.x() <= bounds.m_max.x()) &&
              (pt.y() >= bounds.m_min.y()) && (pt.y() <= bounds.m_max.y()) &&
              (pt.z() >= bounds.m_min.z()) && (pt.z() <= bounds.m_max.z());
    }
    ///@}

    /*!
     * \brief Test if the point is within the polygon in ambient 2D space.
     *
     * \anchor pipcnalg
     * Point In Polygon, Crossing Number
     *
     * This method uses the edge crossing Crossing Number algorithm.
     *
     * \sa http://geomalgorithms.com/a03-_inclusion.html
     *
     * If the number of edge crossings is odd, then the point is inside the
     * polygon. Otherwise it lies outside. Any point on an edge or vertex
     * is considered inside.
     *
     * The polygon can either be convex or concave. The polygons should be
     * simple (i.e. no crossings). This algortihm does not work well with
     * complex, crossing polygons.
     *
     * \param pt      Point to test.
     * \param polygon Closed polygon. The polygon vertices may or may not
     *                include vertex polygon[last] == polygon[first].
     *                Regardless, the polygon is treated as closed.
     *
     * \return Returns true if inside, false if outside.
     */
    bool pipCn(const EigenPoint2 &pt, const EigenPoint2List &polygon);

    /*!
     * \brief Test if the point is within the polygon in ambient 3D space,
     * where z is constant (i.e. the polygon is horizontal to the x-y plane).
     *
     * \ref pipcnalg "Crossing Number Algorithm"
     *
     * \param pt      Point to test.
     * \param polygon Closed polygon. The polygon vertices may or may not
     *                include vertex polygon[last] == polygon[first].
     *                Regardless, the polygon is treated as closed.
     *
     * \return Returns true if inside, false if outside.
     */
    bool pipCnZ(const EigenPoint3 &pt, const EigenPoint3List &polygon);

    /*!
     * \brief Test if the point is within the polygon in ambient 3D space,
     * where the polygon and point are projected onto the x-y plane.
     *
     * \ref pipcnalg "Crossing Number Algorithm"
     *
     * \param pt      Point to test.
     * \param polygon Closed polygon. The polygon vertices may or may not
     *                include vertex polygon[last] == polygon[first].
     *                Regardless, the polygon is treated as closed.
     *
     * \return Returns true if inside, false if outside.
     */
    bool pipCnXY(const EigenPoint3 &pt, const EigenPoint3List &polygon);

    /*!
     * \brief Test if the point is within the polygon in ambient 3D space.
     *
     * \ref pipcnalg "Crossing Number Algorithm"
     *
     * \param pt      Point to test.
     * \param polygon Closed polygon. The polygon vertices may or may not
     *                include vertex polygon[last] == polygon[first].
     *                Regardless, the polygon is treated as closed.
     *
     * \return Returns true if inside, false if outside.
     */
    bool pipCn(const EigenPoint3 &pt, const EigenPoint3List &polygon);


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Color
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

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
      rgb <<  COLORI(red),
              COLORI(green),
              COLORI(blue);
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
      rgba << COLORI(red),
              COLORI(green),
              COLORI(blue),
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
      red   = COLOR8(rgb[gf_index::_RED]);
      green = COLOR8(rgb[gf_index::_GREEN]);
      blue  = COLOR8(rgb[gf_index::_BLUE]);
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
      red   = COLOR8(rgba[gf_index::_RED]);
      green = COLOR8(rgba[gf_index::_GREEN]);
      blue  = COLOR8(rgba[gf_index::_BLUE]);
      alpha = cap(rgba[gf_index::_ALPHA], 0.0, 1.0);
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

    /*! \} */ // end of gfmath doxy group

  } // namespace gf_math

} // namespace geofrenzy


#endif // _GF_MATH_H
