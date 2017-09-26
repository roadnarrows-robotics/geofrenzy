////////////////////////////////////////////////////////////////////////////////
//
// Package:   Geofrenzy Robot ROS Package
//
// Link:      https://github.com/roadnarrows-robotics/geofrenzy
//
// File:      gf_math.cpp
//
/*! \file
 *
 * \brief The Geofrenzy math definitions.
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

#include <stdio.h>
#include <math.h>

#include <iostream>
#include <string>
#include <limits>
#include <cassert>

#include <Eigen/Eigen>
#include <Eigen/Geometry>

#include "geometry_msgs/Point.h"
#include "geofrenzy/Polygon64.h"

#include "gf_math.h"

using namespace std;
using namespace Eigen;




//------------------------------------------------------------------------------
// Basic Eigen Operators and Geometry Functions
//------------------------------------------------------------------------------
namespace geofrenzy
{
  namespace gf_math
  {
    void print(ostream &os, const EigenPoint2 &obj)
    {
      os << "(" << obj[_X] << ", " << obj[_Y] << ")";
    }

    void print(ostream &os, const EigenPoint3 &obj)
    {
      os << "(" << obj[_X] << ", " << obj[_Y] << ", " << obj[_Z] << ")";
    }

    void print(ostream &os, const EigenLine2 &obj)
    {
      const EigenPoint2 &o = obj.origin();
      const EigenPoint2 &d = obj.direction();

      os << "origin: " << o << ", " << "direction: " << d;
    }

    void print(ostream &os, const EigenLine3 &obj)
    {
      const EigenPoint3 &o = obj.origin();
      const EigenPoint3 &d = obj.direction();

      os << "origin: " << o << ", " << "direction: " << d;
    }

    void print(ostream &os, const EigenPlane3 &obj)
    {
      size_t  rows = obj.coeffs().rows();

      os << "coeffs: (";
      for(size_t i = 0; i < rows - 1; ++i)
      {
        os << obj.coeffs()[i] << ", ";
      }
      if( rows > 0 )
      {
        os << obj.coeffs()[rows-1];
      }
      os << ")";
    }

    void print(ostream &os, const EigenRGBA &obj)
    {
      os << "("
          << obj[_RED]   << ", "
          << obj[_GREEN] << ", "
          << obj[_BLUE]  << ", "
          << obj[_ALPHA]
        << ")";
    }

    void print(ostream &os, const EigenXYZRGB &obj)
    {
      os << "("
          << obj[_X]        << ", "
          << obj[_Y]        << ", "
          << obj[_Z]        << ", "
          << obj[_XYZRED]   << ", "
          << obj[_XYZGREEN] << ", "
          << obj[_XYZBLUE]
        << ")";
    }

    void print(ostream &os, const EigenXYZRGBA &obj)
    {
      os << "("
          << obj[_X]        << ", "
          << obj[_Y]        << ", "
          << obj[_Z]        << ", "
          << obj[_XYZRED]   << ", "
          << obj[_XYZGREEN] << ", "
          << obj[_XYZBLUE]  << ", "
          << obj[_XYZALPHA]
        << ")";
    }

    void print(ostream &os, const EigenMinMax2 &obj)
    {
      os << "[" << obj.m_min << ", " << obj.m_max << "]";
    }

    void print(ostream &os, const EigenMinMax3 &obj)
    {
      os << "[" << obj.m_min << ", " << obj.m_max << "]";
    }

    EigenPoint3 cartesianToSpherical(const EigenPoint3 &pt)
    {
      double rho = L2Norm(pt);

      if( isApproxZero(rho) )
      {
        return EigenPoint3(0.0, 0.0, 0.0);
      }

      double theta = atan2(pt.y(), pt.x());
      double phi   = acos(pt.z()/rho);

      return EigenPoint3(rho, theta, phi);
    }

    EigenPoint2 intersection(const EigenLine2 &line1, const EigenLine2 &line2)
    {
      const EigenPoint2 &p1 = line1.origin();
      const EigenPoint2  p2 = line1.pointAt(1.0);
      const EigenPoint2 &p3 = line2.origin();
      const EigenPoint2  p4 = line2.pointAt(1.0);

      double x21 = p2.x() - p1.x();
      double y21 = p2.y() - p1.y();
      double x43 = p4.x() - p3.x();
      double y43 = p4.y() - p3.y();

      double d = x43 * y21 - x21 * y43;

      if( d == 0.0 )
      {
        EigenPoint2(Inf, Inf);
      }

      double x31 = p3.x() - p1.x();
      double y31 = p3.y() - p1.y();

      double t = (x43 * y31 - x31 * y43) / d;
      double u = (x21 * y31 - x31 * y21) / d;
  
      // cerr << t << ", " << u << endl;

      // points of intersections
      EigenPoint2 q1 = line1.pointAt(t);
      EigenPoint2 q2 = line2.pointAt(u);

      // cerr << q1 << ", " <<  q2 << endl;

      // should equal within precision
      if( isApprox(q1, q2, 0.001) )
      {
        return q1;
      }
      else
      {
        EigenPoint2(Inf, Inf);
      }
    }
    
    double t_param(const EigenLine3 &line, const EigenPoint2 &pt)
    {
      const EigenPoint3 &o = line.origin();
      const EigenPoint3 &d = line.direction();

      if( d.x() != 0.0 )
      {
        return (pt.x() - o.x())/ d.x();
      }
      else if( d.y() != 0.0 )
      {
        return (pt.y() - o.y())/ d.y();
      }
      else
      {
        return Inf;
      }
    }

    int quadrant(const EigenPoint2 &pt, const double precision)
    {
      // on an axis - no quadrant
      if( (fabs(pt.x()) < precision) || (fabs(pt.y()) < precision) )
      {
        return _QNone;
      }
      // QI or QIV
      else if( pt.x() > 0.0 )
      {
        return pt.y() > 0.0? _QI: _QIV;
      }
      // QII or QIII
      else
      {
        return pt.y() > 0.0? _QII: _QIII;
      }
    }

    int quadrant(const double theta, const double precision)
    {
      // QI
      if( (theta > precision) && (theta < M_PI_2-precision) )
      {
        return _QI;
      }
      // QII
      else if( (theta > M_PI_2+precision) && (theta < M_PI-precision) )
      {
        return _QII;
      }
      // QIII
      else if( (theta > -M_PI+precision)  && (theta < -M_PI_2-precision) )
      {
        return _QIII;
      }
      // QIV
      else if( (theta > -M_PI_2+precision) && (theta < precision) )
      {
        return _QIV;
      }
      // no quadrant
      else
      {
        return _QNone;
      }
    }

    int octant(const EigenPoint3 &pt, const double precision)
    {
      int o = quadrant(xy(pt), precision);

      // on x-z or y-z plane
      if( o == _QNone )
      {
        o = _ONone;
      }

      // on x-y plane 
      else if( fabs(pt.z()) < precision )
      {
        o = _ONone;
      }

      // bottom set
      else if( pt.z() < 0.0 )
      {
        o += 4;
      }

      return o;
    }

    int octant(const double theta, const double phi, const double precision)
    {
      int o = quadrant(theta, precision);

      // on x-z or y-z plane
      if( o == _QNone )
      {
        o = _ONone;
      }

      // on the z-axis
      else if( (fabs(phi) < precision) || (fabs(M_PI-phi) < precision) )
      {
        o = _ONone;
      }

      // bottom set
      else if( phi < 0.0 )
      {
        o += 4;
      }

      return o;
    }

    void growBounds(const EigenPoint2 &pt, EigenBoundary2 &bounds)
    {
      for(size_t i = 0; i < 2; ++i)
      {
        if( pt[i] < bounds.m_min[i] )
        {
          bounds.m_min[i] = pt[i];
        }
        else if( pt[i] > bounds.m_max[i] )
        {
          bounds.m_max[i] = pt[i];
        }
      }
    }

    void growBounds(const double epsilon, EigenBoundary2 &bounds)
    {
      for(size_t i = 0; i < 2; ++i)
      {
        bounds.m_min[i] -= epsilon;
        bounds.m_max[i] += epsilon;
      }
    }

    void growBounds(const EigenPoint3 &pt, EigenBoundary3 &bounds)
    {
      for(size_t i = 0; i < 3; ++i)
      {
        if( pt[i] < bounds.m_min[i] )
        {
          bounds.m_min[i] = pt[i];
        }
        else if( pt[i] > bounds.m_max[i] )
        {
          bounds.m_max[i] = pt[i];
        }
      }
    }

    void growBounds(const double epsilon, EigenBoundary3 &bounds)
    {
      for(size_t i = 0; i < 3; ++i)
      {
        bounds.m_min[i] -= epsilon;
        bounds.m_max[i] += epsilon;
      }
    }

    bool pipCn(const EigenPoint2 &pt, const EigenPoint2List &polygon)
    {
      size_t  j;
      int     cn = 0;

      for(j = 1; j < polygon.size(); ++j)
      {
        const EigenPoint2 &pt0 = polygon[j-1];
        const EigenPoint2 &pt1 = polygon[j];

        //
        // Upward crossing excludes endpoint. Downward crossing excludes
        // startpoint.
        //
        if( (pt0.y() <= pt.y()) && (pt1.y() > pt.y()) ||  // upward crossing
            (pt0.y() > pt.y()) && (pt1.y() <= pt.y()) )   // downward crossing
        {
          // slope - no divide by zero event given the above test   
          double vt = (pt.y()  - pt0.y()) / (pt1.y() - pt0.y());

          // compute the actual edge-ray intersect x-coordinate
          if( pt.x() <  pt0.x() + vt * (pt1.x() - pt0.x()) )
          {
            ++cn;   // a valid crossing of y=pt.y() right of pt.x()
          }
        }
      }

      return cn & 0x01? true: false;
    }

    void alphaBlend(const EigenRGBA &colorFg,
                    const EigenRGBA &colorBg,
                    EigenRGBA       &colorOut)
    {
      double fgAlpha  = colorFg[_ALPHA];
      double bgAlpha  = colorBg[_ALPHA];
      double outAlpha = fgAlpha + bgAlpha * (1.0 - fgAlpha);

      if( outAlpha > 0.0 )
      {
        colorOut = (colorFg * fgAlpha +
                    colorBg * bgAlpha * (1.0 - fgAlpha)) / outAlpha; 
        colorOut[_ALPHA] = outAlpha;
      }
      else
      {
        mkBlack(colorOut);
      }
    }

  } // namespace gf_math

} // namespace geofrenzy
