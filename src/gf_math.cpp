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
    using namespace idx;

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
     * \brief Make black color.
     *
     * \param[out] color  Color to black.
     */
    static inline void mkBlack(EigenRGB &color)
    {
      color << 0.0, 0.0, 0.0;
    }

    /*!
     * \brief Make transparent black color.
     *
     * \param[out] color  Color to black.
     */
    static inline void mkBlack(EigenRGBA &color)
    {
      color << 0.0, 0.0, 0.0, 0.0;
    }

    /*!
     * \brief Make theta limits from 2 points.
     *
     * \param pt0         Point 0.
     * \param pt1         Point 1.
     * \param epsilon     Limits accrucay.
     * \param[out] ptLim  Output thetat min,max limits.
     */
    static void mkThetaLimits(const EigenPoint3 &pt0,
                              const EigenPoint3 &pt1,
                              const double      epsilon,
                              EigenPoint2       &ptLim)
    {
      double theta0 = atan2(pt0.y(), pt0.x());
      double theta1 = atan2(pt1.y(), pt1.x());

      if( theta0 <= theta1 )
      {
        ptLim[_MIN] = theta0 - epsilon;
        ptLim[_MAX] = theta1 + epsilon;
      }
      else
      {
        ptLim[_MIN] = theta1 - epsilon;
        ptLim[_MAX] = theta0 + epsilon;
      }
    }

    /*!
     * \brief Make a bounding box between two points within the given height
     * range.
     *
     * \param pt0         Point 0.
     * \param pt1         Point 1.
     * \param heightMin   Minimum height.
     * \param heightMax   Maximum height.
     * \param epsilon     Bounding region accrucay.
     * \param[out] bbox   Output bounding box.
     */
    static void mkBBox(const EigenPoint3 &pt0,
                       const EigenPoint3 &pt1,
                       const double      heightMin,
                       const double      heightMax,
                       const double      epsilon,
                       EigenBBox3        &bbox)
    {
      // min,max x
      if( pt0.x() <= pt1.x() )
      {
        bbox.m_min.x() = pt0.x() - epsilon;
        bbox.m_max.x() = pt1.x() + epsilon;
      }
      else
      {
        bbox.m_min.x() = pt1.x() - epsilon;
        bbox.m_max.x() = pt0.x() + epsilon;
      }
      
      // min,max y
      if( pt0.y() <= pt1.y() )
      {
        bbox.m_min.y() = pt0.y() - epsilon;
        bbox.m_max.y() = pt1.y() + epsilon;
      }
      else
      {
        bbox.m_min.y() = pt1.y() - epsilon;
        bbox.m_max.y() = pt0.y() + epsilon;
      }

      // min,max z
      bbox.m_min.z() = heightMin - epsilon;
      bbox.m_max.z() = heightMax + epsilon;
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

    EigenPoint3 sphericalToCartesian(const double r,
                                     const double theta,
                                     const double phi)
    {
      return EigenPoint3(r * sin(phi) * cos(theta),
                         r * sin(phi) * sin(theta),
                         r * cos(phi));

    }

    void sphericalToCartesian(const double r,
                              const double theta,
                              const double phi,
                              EigenPoint3  &pt)
    {
      pt.x() = r * sin(phi) * cos(theta);
      pt.y() = r * sin(phi) * sin(theta);
      pt.z() = r * cos(phi);
    }

    bool within(const EigenPoint3 &pt, const EigenBBox3 &bbox)
    {
      if( (pt.x() < bbox.m_min.x()) || (pt.x() > bbox.m_max.x()) )
      {
        return false;
      }
      else if( (pt.y() < bbox.m_min.y()) || (pt.y() > bbox.m_max.y()) )
      {
        return false;
      }
      else if( (pt.z() < bbox.m_min.z()) || (pt.z() > bbox.m_max.z()) )
      {
        return false;
      }
      else
      {
        return true;
      }
    }

    void rgb24ToIntensities(const unsigned int red,
                            const unsigned int green,
                            const unsigned int blue,
                            EigenRGB           &rgb)
    {
      rgb[_RED]   = (double)(  red & 0x0ff)/Color24ChannelMax;
      rgb[_GREEN] = (double)(green & 0x0ff)/Color24ChannelMax;
      rgb[_BLUE]  = (double)( blue & 0x0ff)/Color24ChannelMax;
    }

    void rgba24ToIntensities(const unsigned int red,
                            const unsigned int green,
                            const unsigned int blue,
                            const double       alpha,
                            EigenRGBA          &rgba)
    {
      rgba[_RED]   = (double)(  red & 0x0ff)/Color24ChannelMax;
      rgba[_GREEN] = (double)(green & 0x0ff)/Color24ChannelMax;
      rgba[_BLUE]  = (double)( blue & 0x0ff)/Color24ChannelMax;

      if( alpha < 0.0 )
      {
        rgba[_ALPHA] = 0.0;
      }
      else if( alpha > 1.0 )
      {
        rgba[_ALPHA] = 1.0;
      }
      else
      {
        rgba[_ALPHA] = alpha;
      }
    }

    void intensitiesToRgb24(const EigenRGB &rgb,
                            unsigned int   &red,
                            unsigned int   &green,
                            unsigned int   &blue)
    {
      red   = (unsigned int)(rgb[_RED]   * Color24ChannelMax) & 0x0ff;
      green = (unsigned int)(rgb[_GREEN] * Color24ChannelMax) & 0x0ff;
      blue  = (unsigned int)(rgb[_BLUE]  * Color24ChannelMax) & 0x0ff;
    }

    void intensitiesToRgb24(const EigenRGBA &rgba,
                            unsigned int    &red,
                            unsigned int    &green,
                            unsigned int    &blue,
                            double          &alpha)
    {
      red   = (unsigned int)(rgba[_RED]   * Color24ChannelMax) & 0x0ff;
      green = (unsigned int)(rgba[_GREEN] * Color24ChannelMax) & 0x0ff;
      blue  = (unsigned int)(rgba[_BLUE]  * Color24ChannelMax) & 0x0ff;
      alpha = rgba[_ALPHA];
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

    ostream &operator<<(ostream &os, const EigenBBox3 &bbox)
    {
      os << "[" << bbox.m_min << ", " << bbox.m_max << "]";
      return os;
    }

    void createSceneObj(const Polygon64 &polygon,
                        const EigenRGBA &color,
                        const double    &fenceHeight,
                        EigenSceneObj   &sceneObj)
    {
      size_t            numPoints = polygon.points.size();
      EigenBBox3        bbox;
      EigenPlane3       plane;
      double            height;
      EigenPoint3       pt0, pt1, pt2;
      EigenPoint2       ptLim;
      size_t            i, j;

      // clear
      sceneObj.m_color = color;
      sceneObj.m_thetas.clear();
      sceneObj.m_planes.clear();
      sceneObj.m_bboxes.clear();

      // insufficient points in polygon
      if( numPoints < 2 )
      {
        return;
      }

      height = fenceHeight < FenceMinHeight? FenceMinHeight: fenceHeight;

      for(i = 0, j = 1; j < numPoints; ++i, ++j)
      {
        pt0.x() = polygon.points[i].x;
        pt0.y() = polygon.points[i].y;
        pt0.z() = 0.0;

        pt1.x() = polygon.points[j].x;
        pt1.y() = polygon.points[j].y;
        pt1.z() = 0.0;

        pt2.x() = pt0.x();
        pt2.y() = pt0.y();
        pt2.z() = height;

        mkThetaLimits(pt0, pt1, 0.001, ptLim);

        mkBBox(pt0, pt1, 0.0, height, 0.001, bbox);

        sceneObj.m_thetas.push_back(ptLim);
        sceneObj.m_planes.push_back(EigenPlane3::Through(pt0, pt1, pt2));
        sceneObj.m_bboxes.push_back(bbox);

        //cerr << "limits: " << ptLim << endl;
        //cerr << "bbox:   " << bbox << endl;
      }

      //cerr << "Created scene object: " << endl;
      //cerr << "  color:  " << sceneObj.m_color << endl;
      //cerr << "  height: " << fenceHeight << endl;
      //cerr << "  faces:  " << sceneObj.m_planes.size() << endl;
    }

    void scanScene(const double thetaMin, const double thetaMax,
                   const double phiMin,   const double phiMax,
                   const size_t width,    const size_t height,
                   const EigenScene       &scene,
                   EigenXYZRGBAList       &intersects,
                   uint32_t               options)
    {
      double      thetaFoV = thetaMax - thetaMin; // theta field of view
      double      phiFoV   = phiMax - phiMin;     // phi field of view
      double      theta, phi;                     // working angles
      size_t      i, j;                           // working indices

      //
      // Loop through scene top to bottom.
      //
      for(i = 0; i < height; ++i)
      {
        phi = phiMin + (phiFoV * (double)i) / (double)height;

        //
        // Loop through scene right to left.
        //
        for(j = 0; j < width; ++j)
        {
          theta = thetaMin + (thetaFoV * (double)j) / (double)width;
          
          traceRay(theta, phi, scene, intersects, options);
        }
      }
    }

    size_t traceRay(const double     theta,
                    const double     phi,
                    const EigenScene &scene,
                    EigenXYZRGBAList &intersects,
                    uint32_t         options)
    {
      EigenPoint3   pt;                         // working point
      bool          hasRay = false;             // lazy init boolean for ray
      EigenLine3    ray;                        // ray
      double        t;                          // intersection parameter value
      RayIntersect  ri;                         // ray intersect info
      RayOrderedIntersects  orderedIntersects;  // ordered list of intersects
      EigenRGBA     colorBlend;                 // alpha blended color
      size_t        i, j;                       // working indices

      //
      // Loop through all the scene objects to find all ray intersections.
      //
      for(i = 0; i < scene.size(); ++i)
      {
        const EigenSceneObj &sceneObj = scene[i];

        //
        // Loop through all of a scene object's polygonal faces to find all
        // ray intersections with this object.
        //
        for(j = 0; j < sceneObj.m_planes.size(); ++j)
        {
          // Pre-filter the ray for this face's min,max thetas. 
          if( (theta < sceneObj.m_thetas[j][_MIN]) ||
              (theta > sceneObj.m_thetas[j][_MAX]) )
          {
            continue;
          }

          // Lazy init of ray (performance tweak).
          if( !hasRay )
          {
            sphericalToCartesian(1.0, theta, phi, pt);
            ray = EigenLine3::Through(Origin3, pt);
            hasRay = true;
          }

          // Find intersection with 2D plane.
          t = ray.intersectionParameter(sceneObj.m_planes[j]);

          // Intersection found (if no intersection, t has value inf or nan).
          if( isnormal(t) )
          {
            // Calculate the point along ray at t (plane intersection).
            pt = ray.pointAt(t);

            // Test if the intersecting point is within the bounding box.
            if( within(pt, sceneObj.m_bboxes[j]) )
            {
              // equivalent ordering as distance
              t = fabs(t);

              // ray intersection
              ri.m_xyz  = pt;
              ri.m_rgba = sceneObj.m_color;

              // add to ascending ordered list
              orderedIntersects[t] = ri;
            }
          }
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
