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

#include <Eigen/Eigen>
#include <Eigen/Geometry>

#include "geometry_msgs/Point.h"
#include "geofrenzy/Polygon64.h"

#include "gf_math.h"

using namespace std;
using namespace Eigen;


// RDK Notes:
// 1. Use openscenegraph

namespace geofrenzy
{
  namespace gf_math
  {
    // -------------------------------------------------------------------------
    // Private
    // -------------------------------------------------------------------------

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Local Data and Utilities 
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*
     * Hard-Coded Tune Parameters.
     */

    /*! Surface linear precision (meters) */
    const double EpsilonLinear = 0.01; 

    /*! Surface angular precision (radians) */
    const double EpsilonAngular = radians(1.0);

    /*! Origin distance precision (meters) */
    const double EpsilonOrigin = 0.5;
 
    /*! Parallel surface scan initial step (meters)  */
    const double ParallelStepInit = 0.1;
 
    /*! Parallel surface geometric progression common ratio */
    const double ParallelStepRatio = 1.05;

    /*!
     * \brief Local structure to hold intermediary scene point of interest info.
     */
    struct ScenePoI
    {
      EigenPoint3 m_xyz;    ///< xyz point
      EigenRGBA   m_rgba;   ///< rgba color
    };

    /*!
     * \brief Ordered map of ray points of interest.
     *
     * - Key:   Parametric line t.
     * - Value: Ray point of interest information.
     *
     * \note  The std::map container container automatically keeps the map in
     *        a natural ascending order by key.
     */
    typedef map<double, ScenePoI> RayOrderedPoI;

    /*!
     * \brief Push xyz point with the associated rgba color onto intersect list.
     *
     * \param[out]  intersects  List of intersecting points.
     * \param xyz   Depth (x,y,z) point.
     * \param rgba  Color intensity red-green-blue-alpha point.
     */
    static inline void pushIntersect(EigenXYZRGBAList &intersects,
                                     EigenPoint3      &xyz,
                                     EigenRGBA        &rgba)
    {
      EigenXYZRGBA  pt;

      pt << xyz, rgba;

      intersects.push_back(pt);
    }

    /*!
     * \brief Add two angles keeping result in (-pi, pi].
     *
     * \param alpha0  Angle 0 in radians.
     * \param alpha1  Angle 1 in radians.
     *
     * \return Added angles in radians (-pi, pi].
     */
    static inline double addAngles(const double alpha0, const double alpha1)
    {
      return pi2pi(alpha0 + alpha1);
    }

    /*!
     * \brief Little work-around to initialize an infinity point in global
     * space.
     *
     * \return Returns infinity point.
     */
    static inline EigenXYZRGBA mkInf()
    {
      EigenXYZRGBA v;
      v << Inf, Inf, Inf, 0.0, 0.0, 0.0, 0.0; 
      return v;
    }

    /*!
     * \brief Infinity and beyond point - a cold, black nothing.
     */
    static const EigenXYZRGBA InfPt = mkInf();


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Make a Scene Functions
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Make a bounding box around sets of x,y,z ranges.
     *
     * \param x0,x1       X range.
     * \param y0,y1       Y range.
     * \param z0,z1       Z range.
     * \param epsilon     Bounding region precision.
     * \param[out] bbox   Output bounding box.
     */
    static void mkBBox(const double x0, const double x1,
                       const double y0, const double y1,
                       const double z0, const double z1,
                       const double epsilon,
                       EigenBBox3   &bbox)
    {
      // min,max x
      if( x0 <= x1 )
      {
        bbox.m_min.x() = x0 - epsilon;
        bbox.m_max.x() = x1 + epsilon;
      }
      else
      {
        bbox.m_min.x() = x1 - epsilon;
        bbox.m_max.x() = x0 + epsilon;
      }
      
      // min,max y
      if( y0 <= y1 )
      {
        bbox.m_min.y() = y0 - epsilon;
        bbox.m_max.y() = y1 + epsilon;
      }
      else
      {
        bbox.m_min.y() = y1 - epsilon;
        bbox.m_max.y() = y0 + epsilon;
      }
      
      // min,max z
      if( z0 <= z1 )
      {
        bbox.m_min.z() = z0 - epsilon;
        bbox.m_max.z() = z1 + epsilon;
      }
      else
      {
        bbox.m_min.z() = z1 - epsilon;
        bbox.m_max.z() = z0 + epsilon;
      }
    }

    /*!
     * \brief Calculate surface x-y plane theta properties.
     *
     * \param [in,out] surface  Scene object surface.
     * \param epsilon_angle     Surface theta limits precision.
     * \param epsilon_origin    Surface origin precision.
     */
    static void calcSurfaceThetas(EigenSurface &surface,
                                  const double epsilon_angle,
                                  const double epsilon_origin)
    {
      //
      // Surface nearly edge-on from origin's perspective.
      //
      if( surface.m_projection < epsilon_origin )
      {
        surface.m_thetas.m_min[0] = surface.m_inclination - epsilon_angle;
        surface.m_thetas.m_max[0] = surface.m_inclination + epsilon_angle;

        // reflect accross origin
        surface.m_thetas.m_min[1] = rot180(surface.m_thetas.m_min[0]);
        surface.m_thetas.m_max[1] = rot180(surface.m_thetas.m_max[0]);

        surface.m_subtended = 2 * epsilon_angle;

        return;
      }

      // points 0 and 1 angles from origin
      double theta0 = atan2(surface.m_vertices[0].y(),
                            surface.m_vertices[0].x());

      double theta1 = atan2(surface.m_vertices[1].y(),
                            surface.m_vertices[1].x());

      // swap
      if( theta0 > theta1 )
      {
        double tmp = theta0;
        theta0 = theta1; theta1 = tmp;
      }

      //
      // Surface is one range.
      //
      if( (theta1 - theta0) <= M_PI )
      {
        surface.m_thetas.m_min[0] = theta0 - epsilon_angle;
        surface.m_thetas.m_max[0] = theta1 + epsilon_angle;
        surface.m_thetas.m_min[1] = surface.m_thetas.m_min[0];
        surface.m_thetas.m_max[1] = surface.m_thetas.m_max[0];

        surface.m_subtended =
                surface.m_thetas.m_max[0] - surface.m_thetas.m_min[0];
      }

      //
      // Surface spans across quadrants II and III (-pi), so two ranges.
      //
      else
      {
        surface.m_thetas.m_min[0] = -M_PI;
        surface.m_thetas.m_max[0] = theta0 + epsilon_angle;
        surface.m_thetas.m_min[1] = theta1 - epsilon_angle;
        surface.m_thetas.m_max[1] = M_PI;

        surface.m_subtended =
                surface.m_thetas.m_max[0] - surface.m_thetas.m_min[0] +
                surface.m_thetas.m_max[1] - surface.m_thetas.m_min[1];
      }
    }

    /*!
     * \brief Calculate scene object surface properties.
     *
     * \param           x0,x1    X range.
     * \param           y0,y1    Y range.
     * \param           z0,z1    Z range.
     * \param [in,out]  surface  Scene object surface.
     */
    static void calcSurfaceProps(const double x0, const double x1,
                                 const double y0, const double y1,
                                 const double z0, const double z1,
                                 EigenSurface &surface)
    {
      // surface vertices
      // TODO build surfaces such that the normal points outside of fence
      surface.m_vertices.push_back(EigenPoint3(x0, y0, z0));
      surface.m_vertices.push_back(EigenPoint3(x1, y1, z0));
      surface.m_vertices.push_back(EigenPoint3(x1, y1, z1));
      surface.m_vertices.push_back(EigenPoint3(x0, y0, z1));

      // working 2D projected points
      EigenPoint2 pt0(x0, y0);
      EigenPoint2 pt1(x1, y1);

      // 3 points define a plane
      surface.m_plane = EigenPlane3::Through(surface.m_vertices[0],
                                             surface.m_vertices[1],
                                             surface.m_vertices[2]);

      // length of surface base line
      surface.m_length = distance(surface.m_vertices[0], surface.m_vertices[1]);

      // total height of the surface
      surface.m_height = fabs(z1 - z0);

      // base line inclination
      surface.m_inclination = inclination(pt0, pt1);

      // origin project (distance) from extended base line 
      surface.m_projection = projection(pt0, pt1);

      // surface bounding box
      mkBBox(x0, x1, y0, y1, z0, z1, EpsilonLinear, surface.m_bbox);

      // surface thetas
      calcSurfaceThetas(surface, EpsilonAngular, EpsilonOrigin);
    }


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Generate Scene Points of Interest Functions
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Project vsensor ray onto the surface.
     *
     * The ray emanates from the origin at the spherical direction theta,phi.
     *
     * \param       theta       Azimuthal angle from x+ axis (-pi, pi].
     * \param       phi         Polar angle from z+ [0, pi].
     * \param       stepInit    Projection initial step size.
     * \param       stepRation  Geometric common ratio.
     * \param       color       Color of this surface.
     * \param       surface     Scene object surface.
     * \param [out] rayPoI      Order map of ray points of interest.
     *
     * \return Number of points of interest added.
     */
    static size_t projectRay(const double         theta,
                             const double         phi,
                             const double         stepInit,
                             const double         stepRatio,
                             const EigenRGBA      &color,
                             const EigenSurface   &surface,
                             RayOrderedPoI        &rayPoI)
    {
      EigenPoint3   pt;   // working point
      EigenLine3    ray;  // vsensor ray

      pt  = sphericalToCartesian(1.0, theta, phi);
      ray = EigenLine3::Through(Origin3, pt);

      // project ray onto the x-y plane
      EigenLine2 ray_xy(Origin2, xy(ray.pointAt(1.0)));

      // project surface lower points onto the x-y plane
      EigenPoint2 pt0(xy(surface.m_vertices[0]));
      EigenPoint2 pt1(xy(surface.m_vertices[1]));

      // normal to the line through pt0 and pt1
      EigenPoint2 normal = polarToCartesian(1.0, surface.m_inclination+M_PI_2);

      // normal line through pt0
      EigenLine2 line0(pt0, pt0+normal);

      // find intersection
      EigenPoint2 qt0 = intersection(ray_xy, line0);

      // no intersection
      if( isinf(qt0.x()) || isinf(qt0.y()) )
      {
        return 0;
      }

      double t0 = t_param(ray, qt0);

      // something is wacked
      if( isinf(t0) )
      {
        return 0;
      }

      // normal line through pt1
      EigenLine2 line1(pt1, pt1+normal);

      // find intersection
      EigenPoint2 qt1 = intersection(ray_xy, line1);

      // no intersection
      if( isinf(qt1.x()) || isinf(qt1.y()) )
      {
        return 0;
      }

      double t1 = t_param(ray, qt1);

      // wacked a mole
      if( isinf(t1) )
      {
        return 0;
      }

      // surface not in direction of the ray
      if( (t0 < 0.0) && (t1 < 0.0) )
      {
        return 0;
      }
      // surface at point 0 is behind - start at origin
      else if( t0 < 0.9 )
      {
        t0 = 0.0;
      }
      // surface at point 1 is behind - start at origin
      else if( t1 < 0.9 )
      {
        t1 = 0.0;
      }

      // swap
      if( t1 < t0 )
      {
        double t = t0;
        t0 = t1; t1 = t;
      }

      ScenePoI  poi;                // ray point of interest info
      double    t       = t0;       // iterator
      double    step    = stepInit; // step size
      size_t    n       = 0;        // number of points added   
      bool      isValid = true;     // valid so far

      // surface has only one color attribute
      poi.m_rgba = color;

      //
      // Loop through vsensor ray and project onto surface.
      // 
      while( isValid )
      {
        pt = ray.pointAt(t);                  // point along ray
        pt = surface.m_plane.projection(pt);  // project point into plane

        // add point of interest if within surface
        if( (isValid = contained(pt, surface.m_bbox)) )
        {
          poi.m_xyz = pt;       // ray intersection
          rayPoI[t] = poi;      // add to ascending ordered list

          step *= stepRatio;    // geometrically increase step size
          t    += step;         // next parametrize line value

          ++n;                  // number of points added
        }
      }
      
      return n;
    }

    /*!
     * \brief Post-process traced points of interest.
     *
     * \param [in]  rayPoI      Order map of ray points of interest.
     * \param[out]  intersects  List of intersecting points.
     * \param       options     Options to control the trace.
     *
     * \return Number of intersections added.
     */
    static size_t postprocPoI(RayOrderedPoI    &rayPoI,
                              EigenXYZRGBAList &intersects,
                              uint32_t         options)
    {
      //
      // No intersections.
      //
      if( rayPoI.empty() )
      {
        if( SCANOPT_2D(options) )
        {
          intersects.push_back(InfPt);
          return 1;
        }
        else
        {
          return 0;
        }
      }

      EigenRGBA               colorBlend;   // alpha blended color
      RayOrderedPoI::iterator iter;         // iterator

      //
      // Alpha blend color(s).
      //
      if( SCANOPT_ALPHA_BLEND(options) )
      {
        mkBlack(colorBlend);

        for(iter = rayPoI.begin(); iter != rayPoI.end(); ++iter)
        {
          // blend foreground and background to get new blended color
          alphaBlend(colorBlend, iter->second.m_rgba, colorBlend);

          // full opacity - further blending is unnecessary
          if( colorBlend[_ALPHA] >= 1.0 )
          {
            break;
          }
        }
      }

      //
      // Loop through all detected ray intersections/prjections.
      //
      for(iter = rayPoI.begin(); iter != rayPoI.end(); ++iter)
      {
        if( SCANOPT_ALPHA_BLEND(options) )
        {
          pushIntersect(intersects, iter->second.m_xyz, colorBlend);
        }
        else
        {
          pushIntersect(intersects, iter->second.m_xyz, iter->second.m_rgba);
        }

        // add only the nearest intersecting point
        if( SCANOPT_NEAREST_ONLY(options) )
        {
          return 1;
        }
      }

      // all points of interest were added
      return rayPoI.size();
    }

    /*!
     * \brief Trace vsensor ray through virtual scene to generate a list of
     * intersecting depth + color points.
     *
     * The Sperical coordinates are as used in mathematics.
     *
     * \param       theta       Ray's azimuthal angle from x+ axis (-pi, pi].
     * \param       phi         Ray's polar angle from z+ [0, pi].
     * \param       thetaNext   Next theta in scan. If scan line is at the end,
     *                          set thetaNext to theta.
     * \param       scene       The scene.
     * \param[out]  intersects  List of intersecting points.
     * \param       options     Options to control the trace.
     *
     * \return Number of intersections added.
     */
    static size_t traceRay(const double     theta,
                           const double     phi,
                           const double     thetaNext,
                           const EigenScene &scene,
                           EigenXYZRGBAList &intersects,
                           uint32_t         options)
    {
      bool          hasRay = false;   // lazy init boolean for ray
      EigenPoint3   pt;               // working point
      EigenLine3    ray;              // vsensor ray
      double        t;                // intersection parameter value
      ScenePoI      poi;              // ray point of interest info
      size_t        i, j;             // working indices
      RayOrderedPoI rayPoI;           // ordered map of ray points of interest

      //
      // Loop through all the scene objects to find all ray intersections.
      //
      for(i = 0; i < scene.size(); ++i)
      {
        const EigenSceneObj &sceneObj = scene[i];
        const EigenRGBA     &color    = sceneObj.m_color; 

        //
        // Loop through all of a scene object's polygonal surfaces to find all
        // ray intersections with this object.
        //
        for(j = 0; j < sceneObj.m_surfaces.size(); ++j)
        {
          // scene object surface
          const EigenSurface &surface = sceneObj.m_surfaces[j];

          //
          // Pre-filter out ray processing if it is not pointing in the
          // direction of this surface's min,max subtended thetas.
          //
          if( !within(theta, surface.m_thetas) )
          {
            continue;
          }

          //
          // This surface is nearly parallel to the ray. To guarantee sufficient
          // point density of this surface, travel along the nearest ray and
          // project points onto the surface.
          //
          // From the ray direction, the surface can be behind, span across, or
          // be in front.
          //
          if( SCANOPT_XRAY_VIS(options) &&
              isApproxZero(surface.m_projection, EpsilonOrigin) )
          {
            double  inc           = surface.m_inclination;
            double  inc180        = rot180(inc);
            double  dAng0         = theta - inc;
            double  dAng0Next     = thetaNext - inc;
            double  dAng180       = theta - inc180;
            double  dAng180Next   = thetaNext - inc180;
            double  thetaNearest  = theta;
            bool    isNearest     = false; 

            // this ray is the last ray in the scan line
            if( theta == thetaNext )
            {
              isNearest = true;
            }

            // the next theta crosses over the surface
            else if( sign(dAng0) != sign(dAng0Next) )
            {
              if( fabs(dAng0Next) < fabs(dAng0) )
              {
                thetaNearest = thetaNext;
              }
              isNearest = true;
            }

            // the next theta crosses over the reflected surface
            else if( sign(dAng180) != sign(dAng180Next) )
            {
              if( fabs(dAng180Next) < fabs(dAng180) )
              {
                thetaNearest = thetaNext;
              }
              isNearest = true;
            }

            if( isNearest )
            {
              projectRay(theta, phi, ParallelStepInit, ParallelStepRatio,
                        color, surface, rayPoI);
            }
          }

          //
          // Find intersection with 2D plane in 3D ambient space.
          //
          // Note that t can take on both positive and negative values,
          // but given how the ray was contructed, t will always be
          // non-negative.
          //
          else
          {
            // lazy init of ray (performance tweak)
            if( !hasRay )
            {
              sphericalToCartesian(1.0, theta, phi, pt);
              ray = EigenLine3::Through(Origin3, pt);
              hasRay = true;
            }

            t = ray.intersectionParameter(surface.m_plane);

            // check if the ray does intersect the surface plane
            if( isnan(t) || isinf(t) )
            {
              continue;
            }

            // calculate the point along the ray at t (plane intersection)
            pt = ray.pointAt(t);

            // check if the intersecting point is within of the bounding box.
            if( !contained(pt, surface.m_bbox) )
            {
              continue;
            }

            // ray intersection info
            poi.m_rgba = color;
            poi.m_xyz  = pt;

            // add to ascending distance ordered list
            rayPoI[t] = poi;
          }
        }
      }

      return postprocPoI(rayPoI, intersects, options);
    }

    /*!
     * \brief Grid a fence surface vertical stripe to generate a list of
     * depth + color points.
     *
     * \param       gridSize    Grid size.
     * \param       color       Color of this surface.
     * \param       surface     Scene object surface.
     * \param       basept      Start surface base point.
     * \param[out]  intersects  List of intersecting points.
     * \param       options     Options to control the grid generations.
     *
     * \return Number of points generated.
     */
    static size_t gridSurfaceStripe(const double       gridSize,
                                    const EigenRGBA    &color,
                                    const EigenSurface &surface,
                                    const EigenPoint3  &basept,
                                    EigenXYZRGBAList   &intersects,
                                    uint32_t           options)
    {
      EigenPoint3   pt = basept;
      EigenXYZRGBA  xyzrgba;
      size_t        n = 0;
      double        h;

      //
      // Loop from starting base point to surface top at the fixed x-y position.
      //
      for(h = pt.z(); h < surface.m_height; h += gridSize)
      {
        pt.z() = h;

        xyzrgba << pt, color;

        intersects.push_back(xyzrgba);

        ++n;
      }

      //
      // If the last grid point is not sufficiently close to the surface top,
      // add the top point.
      //
      if( !isApprox(h-gridSize, surface.m_height, EpsilonLinear) )
      {
        pt.z() = surface.m_height;

        xyzrgba << pt, color;

        intersects.push_back(xyzrgba);

        ++n;
      }

      return n;
    }

    /*!
     * \brief Grid fence surface to generate a list of depth + color
     * points.
     *
     * \param       gridSize    Grid size.
     * \param       color       Color of this surface.
     * \param       surface     Scene object surface.
     * \param[out]  intersects  List of intersecting points.
     * \param       options     Options to control the grid generations.
     *
     * \return Number of points generated.
     */
    static size_t gridSurface(const double       gridSize,
                              const EigenRGBA    &color,
                              const EigenSurface &surface,
                              EigenXYZRGBAList   &intersects,
                              uint32_t           options)
    {
      double        alpha;
      EigenPoint2   pt_xy;
      EigenPoint3   pt;
      EigenLine3    baseline;
      size_t        n = 0;
      double        w;

      const EigenPoint3 &pt0 = surface.m_vertices[0];
      const EigenPoint3 &pt1 = surface.m_vertices[1];

      //
      // Determine the surface base line angle in the x-y plane (-pi, pi].
      //
      alpha = atan2(pt1.y() - pt0.y(), pt1.x() - pt0.x());

      //
      // Find the point gridSize units away from a line through the origin
      // and parallel to the baseline.
      //
      pt_xy = polarToCartesian(gridSize, alpha);

      //
      // Translate the point to the baseline.
      //
      pt << pt0.x() + pt_xy.x(), pt0.y() + pt_xy.y(), pt0.z();
        
      //
      // Parametric line of the base.
      //
      baseline = EigenLine3::Through(pt0, pt);

      //
      // Loop through the surface length-wise and grid.
      //
      for(w = 0.0; w < surface.m_length; w += gridSize)
      {
        // starting base point
        pt = baseline.pointAt(w);

        // grid surface vertical strip
        n += gridSurfaceStripe(gridSize, color, surface, pt,
                               intersects, options);
      }

      //
      // If the last stripe is not sufficiently close to the surface far end,
      // the add an end strip of grid points.
      //
      if( !isApprox(w-gridSize, surface.m_length, EpsilonLinear) )
      {
        n += gridSurfaceStripe(gridSize, color, surface, pt1,
                               intersects, options);
      }

      return n;
    }

    // -------------------------------------------------------------------------
    // Public
    // -------------------------------------------------------------------------
          
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Stream operators.
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
 
    ostream &operator<<(ostream &os, const EigenPoint2 &pt)
    {
      os << "(" << pt[0] << "," << pt[1] << ")";
      return os;
    }

    ostream &operator<<(ostream &os, const EigenPoint3 &pt)
    {
      os << "(" << pt[0] << "," << pt[1] << "," << pt[2] << ")";
      return os;
    }

    ostream &operator<<(ostream &os, const EigenRGBA &pt)
    {
      os << "(" << pt[0] << "," << pt[1] << "," << pt[2] << "," << pt[3] << ")";
      return os;
    }

    ostream &operator<<(ostream &os, const EigenXYZRGB &pt)
    {
      os << "(" << pt[0] << "," << pt[1] << "," << pt[2] << ","
        << pt[3] << "," << pt[4] << "," << pt[5] << ")";
      return os;
    }

    ostream &operator<<(ostream &os, const EigenXYZRGBA &pt)
    {
      os << "(" << pt[0] << "," << pt[1] << "," << pt[2] << ","
        << pt[3] << "," << pt[4] << "," << pt[5] << ","
        << pt[6] << ")";
      return os;
    }

    ostream &operator<<(ostream &os, const EigenMinMax2 &minmax)
    {
      os << "[" << minmax.m_min << ", " << minmax.m_max << "]";
      return os;
    }

    ostream &operator<<(ostream &os, const EigenMinMax3 &minmax)
    {
      os << "[" << minmax.m_min << ", " << minmax.m_max << "]";
      return os;
    }

    ostream &operator<<(ostream &os, const EigenSurface &surface)
    {
      os << "{" << endl
        << "  surface:      " << surface.m_num << endl
        << "  vertices:" << endl
        << "  [" << endl;
        for(size_t i = 0; i< surface.m_vertices.size(); ++i)
        {
          os << "    " << surface.m_vertices[i] << "," << endl;
        }
      os << "  ]" << endl
        << "  plane:        " << surface.m_plane.coeffs() << endl
        << "  length:       " << surface.m_length << endl
        << "  height:       " << surface.m_height << endl
        << "  inclination:  " << degrees(surface.m_inclination) << endl
        << "  projection:   " << surface.m_projection << endl
        << "  bbox:         " << surface.m_bbox << endl
        << "  theta limits: " 
          << "["
            << degrees(surface.m_thetas.m_min[0]) << ", "
            << degrees(surface.m_thetas.m_max[0])
          << "], "
          << "["
            << degrees(surface.m_thetas.m_min[1]) << ", "
            << degrees(surface.m_thetas.m_max[1])
          << "], "
          << endl
        << "  subtended:    " << degrees(surface.m_subtended) << endl
        << "}";
          
      return os;
    }


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Geometric and Color Functions.
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

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


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Scene Functions
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    void createSceneObj(const Polygon64 &polygon,
                        const EigenRGBA &color,
                        const double    &fenceHeight,
                        EigenSceneObj   &sceneObj)
    {
      size_t            numPoints = polygon.points.size();
      EigenSurface      surface;
      double            height;
      size_t            i, j;

      // clear
      sceneObj.clear();

      // insufficient points in polygon
      if( numPoints < 2 )
      {
        return;
      }

      // constrain height
      height = fenceHeight < FenceMinHeight? FenceMinHeight: fenceHeight;

      //
      // Attrubutes
      //
      sceneObj.m_color = color;

      //
      // Loop through polygon points to create scene object surfaces.
      //
      for(i = 0, j = 1; j < numPoints; ++i, ++j)
      {
        sceneObj.m_surfaces.push_back(surface);

        sceneObj.m_surfaces.back().m_num = i;

        calcSurfaceProps(polygon.points[i].x, polygon.points[j].x,
                         polygon.points[i].y, polygon.points[j].y,
                         0.0, height,
                         sceneObj.m_surfaces.back());

        // cerr << sceneObj.m_surfaces.back() << endl;
      }

      // cerr << "Created scene object: " << endl;
      // cerr << "  color:    " << sceneObj.m_color << endl;
      // cerr << "  surfaces: " << sceneObj.m_surfaces.size() << endl;
    }

    void scanScene(const double thetaMin, const double thetaMax,
                   const double phiMin,   const double phiMax,
                   const size_t width,    const size_t height,
                   const EigenScene       &scene,
                   EigenXYZRGBAList       &intersects,
                   uint32_t               options)
    {
      double thetaStep = (thetaMax - thetaMin) / (double)(width - 1);
      double phiStep   = (phiMax - phiMin) / (double)(height - 1);
      double theta, phi;
      double thetaNext;

      //
      // Loop through scene top to bottom.
      //
      for(phi = phiMin; phi <= phiMax; phi += phiStep)
      {
        //
        // Loop through scene counter-clockwise
        //
        for(theta = thetaMin; theta <= thetaMax; theta += thetaStep)
        {
          thetaNext = theta + thetaStep;

          // end of scan line
          if( thetaNext > thetaMax )
          {
            thetaNext = theta;
          }

          traceRay(theta, phi, thetaNext, scene, intersects, options);
        }
      }
    }

    void gridScene(const double     gridSize,
                   const EigenScene &scene,
                   EigenXYZRGBAList &intersects,
                   uint32_t         options)
    {
      size_t  i, j;

      //
      // Loop thruough scene objects.
      //
      for(i = 0; i < scene.size(); ++i)
      {
        const EigenSceneObj &sceneObj = scene[i];
        const EigenRGBA     &color    = sceneObj.m_color; 

        //
        // Loop through all of a scene object's polygonal surfaces to grid.
        //
        for(j = 0; j < sceneObj.m_surfaces.size(); ++j)
        {
          // scene object surface
          const EigenSurface &surface = sceneObj.m_surfaces[j];

          // grid surface
          gridSurface(gridSize, color, sceneObj.m_surfaces[j],
                      intersects, options);
        }
      }
    }


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Unit Tests
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

#ifdef GF_MATH_UT
    void utMakeCannedPolygon(const int         polynum,
                             const EigenPoint3 &offset,
                             const double      scale,
                             Polygon64         &polygon)
    {
      // equilateral triange with side 50m
      static const double triangle[][3] =
      {
        {0.0, 0.0, 0.0}, {43.3, -25.0, 0.0}, {43.3, 25.0, 0.0}, {0.0, 0.0, 0.0}
      };

      // 20m x 30m rectangle
      static const double rectangle[][3] =
      {
        { 0.0, -10.0, 0.0}, {30.0, -10.0, 0.0},
        {30.0,  10.0, 0.0}, { 0.0,  10.0, 0.0},
        { 0.0, -10.0, 0.0}
      };

      // hexagon with 10m sides
      static const double hexagon[][3] =
      {
        { 0.0,     -5.0, 0.0},
        { 8.6603, -10.0, 0.0},
        {17.3205,  -5.0, 0.0},
        {17.3205,   5.0, 0.0},
        { 8.6603,  10.0, 0.0},
        { 0.0,      5.0, 0.0},
        { 0.0,     -5.0, 0.0}
      };

      // 40m x 50m tee
      static const double tee[][3] =
      {
        { 0.0,  -5.0, 0.0},
        {40.0,  -5.0, 0.0},
        {40.0, -20.0, 0.0},
        {50.0, -20.0, 0.0},
        {50.0,  20.0, 0.0},
        {40.0,  20.0, 0.0},
        {40.0,   5.0, 0.0},
        { 0.0,   5.0, 0.0},
        { 0.0,  -5.0, 0.0}
      };

      const double  (*poly)[3];
      size_t        numPoints;

      geometry_msgs::Point  pt;

      switch(polynum)
      {
        case UtPolynumTriangle:
          poly      = triangle;
          numPoints = 4;
          break;
        case UtPolynumRectangle:
          poly      = rectangle;
          numPoints = 5;
          break;
        case UtPolynumHexagon:
          poly      = hexagon;
          numPoints = 7;
          break;
        case UtPolynumTee:
          poly      = tee;
          numPoints = 9;
          break;
        default:
          return;
      }

      for(size_t i = 0; i < numPoints; ++i)
      {
        pt.x = scale * poly[i][0] + offset.x();
        pt.y = scale * poly[i][1] + offset.y();
        pt.z = scale * poly[i][2] + offset.z();
        polygon.points.push_back(pt);
      }

      // cerr << polygon << endl;
    }

    void utScanPolygon(const Polygon64 &polygon)
    {
      EigenScene        scene;
      EigenSceneObj     sceneObj;
      EigenXYZRGBAList  intersects;

      scene.push_back(sceneObj);

      cout << "Create scene" << endl;

      createSceneObj(polygon, FenceColorDft, 100.0, scene.back());

      cout << "Scan scene" << endl;

      scanScene(radians(-90.0), radians(90.0),
                radians(45.0),  radians(90.0),
                160, 120,
                scene, intersects, 0);

      cout << "Out ";
      cout << intersects.size() << endl;
    }
#endif // GF_MATH_UT

  } // namespace gf_math
} // namespace geofrenzy
