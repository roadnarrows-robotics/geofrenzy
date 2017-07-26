// GEOFRENZY FILE HEADER HERE

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
    /*!
     * \brief Hard-coded Tune Parameters.
     */
    const double EpsilonLinear      = 0.001;  ///< surface linear precision
    const double EpsilonAngular     = radians(1.0);
                                              ///< surface angular precision
    const double EpsilonOrigin      = 0.5;    ///< origin distance precision
    const double DeltaParStepMin    = 0.1;    ///< parallel surface scan step
    const double DeltaParStepScale  = 1.05;   ///< parallel surface geometric

    /*!
     * \brief Local structure to hold intermediary intersection info.
     */
    struct RayIntersect
    {
      EigenPoint3 m_xyz;    ///< xyz point
      EigenRGBA   m_rgba;   ///< rgba color
    };

    /*!
     * \brief Ordered list of ray intersections.
     *
     * - Key:   distance
     * - Value: ray intersect info
     *
     * Note:  The std::map container container automatically keeps the map in
     *        a natural ascending order by key.
     */
    typedef map<double, RayIntersect>   RayOrderedIntersects;

    /*!
     * \brief Little work-around to initialize an infinity point is global
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
     * \brief Add two angles keeping result in [-pi, pi].
     *
     * \param theta0  Angle 0 in radians.
     * \param theta1  Angle 1 in radians.
     *
     * \return Added angles in radians.
     */
    static inline double addAngles(const double theta0, const double theta1)
    {
      return pi2pi(theta0 + theta1);
    }

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
     * \brief Make theta limits from 2 points.
     *
     * \param pt0         Point 0.
     * \param pt1         Point 1.
     * \param epsilon     Limits precision.
     * \param[out] ptLim  Output theta min,max limits.
     */
    static void calcSurfaceThetas(EigenSurface &surface,
                                  const double e_ang,
                                  const double e_origin)
    {
      double theta0 = atan2(surface.m_pt0.y(), surface.m_pt0.x());
      double theta1 = atan2(surface.m_pt1.y(), surface.m_pt1.x());

      // swap
      if( theta0 > theta1 )
      {
        double t = theta0;
        theta0 = theta1;
        theta1 = t;
      }

      // surface edge-on from origin
      if( surface.m_projection < e_origin )
      {
        surface.m_thetas.m_min[0] = surface.m_inclination - e_ang;
        surface.m_thetas.m_max[0] = surface.m_inclination + e_ang;

        // reflect accross origin
        surface.m_thetas.m_min[1] = rot180(surface.m_thetas.m_min[0]);
        surface.m_thetas.m_max[1] = rot180(surface.m_thetas.m_max[0]);

        surface.m_subtended = 2 * e_ang;
      }

      // one range
      else if( (theta1 - theta0) <= M_PI )
      {
        surface.m_thetas.m_min[0] = theta0 - e_ang;
        surface.m_thetas.m_max[0] = theta1 + e_ang;
        surface.m_thetas.m_min[1] = surface.m_thetas.m_min[0];
        surface.m_thetas.m_max[1] = surface.m_thetas.m_max[0];

        surface.m_subtended =
                surface.m_thetas.m_max[0] - surface.m_thetas.m_min[0];
      }

      // spans across quadrants II and III (-pi), so two ranges
      else
      {
        surface.m_thetas.m_min[0] = -M_PI;
        surface.m_thetas.m_max[0] = theta0 + e_ang;
        surface.m_thetas.m_min[1] = theta1 - e_ang;
        surface.m_thetas.m_max[1] = M_PI;

        surface.m_subtended =
                surface.m_thetas.m_max[0] - surface.m_thetas.m_min[0] +
                surface.m_thetas.m_max[1] - surface.m_thetas.m_min[1];
      }
    }

    static void calcSurfaceParams(const double x0, const double x1,
                                  const double y0, const double y1,
                                  const double z0, const double z1,
                                  EigenSurface &surface)
    {
      surface.m_pt0 << x0, y0, 0.0;
      surface.m_pt1 << x1, y1, 0.0;
      surface.m_pt2 << x0, y0, z1;

      // 3 points define a plane
      surface.m_plane   = EigenPlane3::Through(surface.m_pt0,
                                               surface.m_pt1,
                                               surface.m_pt2);

      surface.m_length      = distance(surface.m_pt0, surface.m_pt1);
      surface.m_height      = fabs(z1 - z0);
      surface.m_inclination = inclination2(surface.m_pt0, surface.m_pt1);
      surface.m_projection  = projection2(surface.m_pt0, surface.m_pt1);

      mkBBox(x0, x1, y0, y1, z0, z1, EpsilonLinear, surface.m_bbox);

      calcSurfaceThetas(surface, EpsilonAngular, EpsilonOrigin);
    }

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
     * \brief Infinity and beyond point - a cold, black nothing.
     */
    static const EigenXYZRGBA InfPt = mkInf();

    size_t projectRay(const EigenLine3 &ray,
                      double t,
                      const double stepSize,
                      const double stepScale,
                      const EigenRGBA &color,
                      const EigenSurface &surface,
                      RayOrderedIntersects  &orderedIntersects)
    {
      EigenPoint3   pt;                 // working point
      RayIntersect  ri;                 // ray intersect info
      double        step    = stepSize; // step size
      size_t        n       = 0;        // number of points added   
      bool          isvalid = true;     // valid so far

      ri.m_rgba = color;  // surface has one color

      while( isvalid )
      {
        pt = ray.pointAt(t);                  // point along ray
        pt = surface.m_plane.projection(pt);  // project point into plane

        // add point if within surface
        if( (isvalid = contained(pt, surface.m_bbox)) )
        {
          ri.m_xyz = pt;              // ray intersection

          orderedIntersects[t] = ri;  // add to ascending ordered list

          step *= stepScale;          // geometrically increase step size

          t += step;                  // next parametrize line value

          ++n;                        // number of points added
        }
      }
      
      return n;
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
        << "  points:       "
          << surface.m_pt0 << ", "
          << surface.m_pt1 << ", "
          << surface.m_pt2 << endl
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

        calcSurfaceParams(polygon.points[i].x, polygon.points[j].x,
                          polygon.points[i].y, polygon.points[j].y,
                          0.0, height,
                          sceneObj.m_surfaces.back());

        //cerr << sceneObj.m_surfaces.back() << endl;
      }

      //cerr << "Created scene object: " << endl;
      //cerr << "  color:    " << sceneObj.m_color << endl;
      //cerr << "  surfaces: " << sceneObj.m_surfaces.size() << endl;
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
        // Loop through scene right to left.
        //
        for(theta = thetaMin; theta <= thetaMax; theta += thetaStep)
        {
          thetaNext = theta >= thetaMax? thetaMax: thetaNext + thetaStep;
          traceRay(theta, phi, thetaNext, scene, intersects, options);
        }
      }
    }

    size_t traceRay(const double     theta,
                    const double     phi,
                    const double     thetaNext,
                    const EigenScene &scene,
                    EigenXYZRGBAList &intersects,
                    uint32_t         options)
    {
      EigenPoint3   pt;                         // working point
      bool          hasRay = false;             // lazy init boolean for ray
      EigenLine3    ray;                        // ray
      double        t;                          // intersection parameter value
      RayIntersect  ri;                         // ray intersect info
      EigenRGBA     colorBlend;                 // alpha blended color
      size_t        i, j;                       // working indices

      RayOrderedIntersects  orderedIntersects;  // ordered list of intersects

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
          const EigenSurface &surface = sceneObj.m_surfaces[j];

#if 0 // RDK TODO
if( surface.m_num == 2 )
{  
  if( theta >= radians(-100) && theta <= radians(-80) )
    cerr << "rdk: " << degrees(theta) << endl;
}
#endif // RDK TODO
          // pre-filter the ray on this surface's min,max apparent thetas
          if( !within(theta, surface.m_thetas) )
          {
            continue;
          }

          // lazy init of ray (performance tweak)
          if( !hasRay )
          {
            sphericalToCartesian(1.0, theta, phi, pt);
            ray = EigenLine3::Through(Origin3, pt);
            hasRay = true;
          }

          //
          // Find intersection with 2D plane in 3D ambient space.
          //
          // Note that t can take on both positive and negative values,
          // but given how the ray was contructed, t will always be
          // non-negative.
          //
          t = ray.intersectionParameter(surface.m_plane);

          // check if ray does not intersect surface
          if( isnan(t) || isinf(t) )
          {
            continue;
          }

          // calculate the point along the ray at t (plane intersection)
          pt = ray.pointAt(t);

          // check if the intersecting point is outside of the bounding box.
          if( !contained(pt, surface.m_bbox) )
          {
            continue;
          }

          // ray intersection
          ri.m_rgba = color;
          ri.m_xyz  = pt;

          // add to ascending distance ordered list
          orderedIntersects[t] = ri;

          //
          // This surface is nearly parallel to the ray. To guarantee sufficient
          // point density of this surface, travel along the ray and project
          // points onto the surface.
          //
#if 0 // RDK TODO
          if( !(options & ScanOptionNearest) &&
              (surface.m_projection < EpsilonOrigin) )
          {
            // test if this ray is the closest to parallel surface
            if( fabs((rot180if(theta)     - surface.m_inclination)) <=
                fabs((rot180if(thetaNext) - surface.m_inclination)) )
            {
if( surface.m_num == 2 )
{  
cerr << "rdk: "
  << " surface " << surface.m_num << ":"
  << " phi = " << degrees(phi)
  << " theta = " << degrees(theta)
  << endl;
}
              projectRay(ray, t+DeltaParStepMin,
                DeltaParStepMin, DeltaParStepScale,
                color, surface, orderedIntersects);
            }
          }
#endif // RDK TODO
        }
      }

      //
      // No intersections.
      //
      if( orderedIntersects.empty() )
      {
        if( options & ScanOption2D )
        {
          intersects.push_back(InfPt);
          return 1;
        }
        else
        {
          return 0;
        }
      }

      RayOrderedIntersects::iterator  iter;

      //
      // Alpha blend color(s).
      //
      if( options & ScanOptionAlphaBlend )
      {
        mkBlack(colorBlend);

        for(iter = orderedIntersects.begin();
            iter != orderedIntersects.end();
            ++iter)
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
      // Loop through all detected ray intersections.
      //
      for(iter = orderedIntersects.begin();
          iter != orderedIntersects.end();
          ++iter)
      {
        if( options & ScanOptionAlphaBlend )
        {
          pushIntersect(intersects, iter->second.m_xyz, colorBlend);
        }
        else
        {
          pushIntersect(intersects, iter->second.m_xyz, iter->second.m_rgba);
        }

        // Add only the nearest intersecting point.
        if( options & ScanOptionNearest )
        {
          return 1;
        }
      }

      return orderedIntersects.size();
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

      //cerr << polygon << endl;
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
