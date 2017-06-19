// GEOFRENZY FILE HEADER HERE

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
      rgb[_R] = (double)(  red & 0x0ff)/255.0;
      rgb[_G] = (double)(green & 0x0ff)/255.0;
      rgb[_B] = (double)( blue & 0x0ff)/255.0;
    }

    void rgba24ToIntensities(const unsigned int red,
                            const unsigned int green,
                            const unsigned int blue,
                            const double       alpha,
                            EigenRGBA          &rgba)
    {
      rgba[_R] = (double)(  red & 0x0ff)/255.0;
      rgba[_G] = (double)(green & 0x0ff)/255.0;
      rgba[_B] = (double)( blue & 0x0ff)/255.0;

      if( alpha < 0.0 )
      {
        rgba[_A] = 0.0;
      }
      else if( alpha > 1.0 )
      {
        rgba[_A] = 1.0;
      }
      else
      {
        rgba[_A] = alpha;
      }
    }

    void intensitiesToRgb24(const EigenRGB &rgb,
                            unsigned int   &red,
                            unsigned int   &green,
                            unsigned int   &blue)
    {
      red   = (unsigned int)(rgb[_R] * 255.0) & 0x0ff;
      green = (unsigned int)(rgb[_G] * 255.0) & 0x0ff;
      blue  = (unsigned int)(rgb[_B] * 255.0) & 0x0ff;
    }

    void intensitiesToRgb24(const EigenRGBA &rgba,
                            unsigned int    &red,
                            unsigned int    &green,
                            unsigned int    &blue,
                            double          &alpha)
    {
      red   = (unsigned int)(rgba[_R] * 255.0) & 0x0ff;
      green = (unsigned int)(rgba[_G] * 255.0) & 0x0ff;
      blue  = (unsigned int)(rgba[_B] * 255.0) & 0x0ff;
      alpha = rgba[_A];
    }

    void alphaBlend(const EigenRGBA &colorFg,
                    const EigenRGBA &colorBg,
                    EigenRGBA       &colorOut)
    {
      double fgAlpha  = colorFg[_A];
      double bgAlpha  = colorBg[_A];
      double outAlpha = fgAlpha + bgAlpha * (1.0 - fgAlpha);

      if( outAlpha > 0.0 )
      {
        colorOut = (colorFg * fgAlpha +
                    colorBg * bgAlpha * (1.0 - fgAlpha)) / outAlpha; 
        colorOut[_A] = outAlpha;
      }
      else
      {
        mkBlack(colorOut);
      }
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
      size_t            i, j;

      sceneObj.m_color = color;
      sceneObj.m_bboxes.clear();
      sceneObj.m_planes.clear();

      if( numPoints < 2 )
      {
        return;
      }

      height = fenceHeight < FenceMinHeight? FenceMinHeight: fenceHeight;

      for(i = 0, j = 1; j < numPoints; ++i, ++j)
      {
        // min,max x
        if( polygon.points[i].x <= polygon.points[j].x )
        {
          bbox.m_min.x() = polygon.points[i].x;
          bbox.m_max.x() = polygon.points[j].x;
        }
        else
        {
          bbox.m_min.x() = polygon.points[j].x;
          bbox.m_max.x() = polygon.points[i].x;
        }
        
        // min,max y
        if( polygon.points[i].y <= polygon.points[j].y )
        {
          bbox.m_min.y() = polygon.points[i].y;
          bbox.m_max.y() = polygon.points[j].y;
        }
        else
        {
          bbox.m_min.y() = polygon.points[j].y;
          bbox.m_max.y() = polygon.points[i].y;
        }

        // min,max z
        bbox.m_min.z() = 0.0;
        bbox.m_max.z() = height;
        
        //cerr << "bbox: " << bbox << endl;

        sceneObj.m_bboxes.push_back(bbox);

        pt0.x() = polygon.points[i].x;
        pt0.y() = polygon.points[i].y;
        pt0.z() = 0.0;

        pt1.x() = polygon.points[j].x;
        pt1.y() = polygon.points[j].y;
        pt1.z() = 0.0;

        pt2.x() = pt1.x();
        pt2.y() = pt1.y();
        pt2.z() = height;

        sceneObj.m_planes.push_back(EigenPlane3::Through(pt0, pt1, pt2));
      }

      cerr << "Created scene object: " << endl;
      cerr << "  color:  " << sceneObj.m_color << endl;
      cerr << "  height: " << fenceHeight << endl;
      cerr << "  faces:  " << sceneObj.m_planes.size() << endl;
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
      EigenPoint3 pt0(0.0, 0.0, 0.0);             // observer's origin
      EigenPoint3 pt1;                            // working (x,y,z) point
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
          
          sphericalToCartesian(1.0, theta, phi, pt1);

          EigenLine3 ray = EigenLine3::Through(pt0, pt1);

          traceRay(ray, scene, intersects, options);
        }
      }
    }

    size_t traceRay(const EigenLine3 &ray,
                    const EigenScene &scene,
                    EigenXYZRGBAList &intersects,
                    uint32_t         options)
    {
      double        t;                          // intersection parameter value
      EigenPoint3   pt;                         // working point
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
      mkBlack(colorBlend);

      for(iter = orderedIntersects.begin();
          iter != orderedIntersects.end();
          ++iter)
      {
        // blend foreground and background to get new blended color
        alphaBlend(colorBlend, iter->second.m_rgba, colorBlend);

        // full opacity - further blending is unnecessary
        if( colorBlend[_A] >= 1.0 )
        {
          break;
        }
      }

      //
      // Add all intersecting points.
      //
      if( options & ScanCollinear )
      {
        for(iter = orderedIntersects.begin();
            iter != orderedIntersects.end();
            ++iter)
        {
          pushIntersect(intersects, iter->second.m_xyz, colorBlend);
        }
        return orderedIntersects.size();
      }

      //
      // Add only the nearest intersecting point.
      //
      else
      {
        // near point is at the front of ordered list
        iter = orderedIntersects.begin();

        pushIntersect(intersects, iter->second.m_xyz, colorBlend);

        return 1;
      }
    }

    size_t traceRay(EigenLine3 &ray, const EigenSceneObj &sceneObj,
                    EigenXYZRGBAList &intersects, uint32_t options)
    {
      return 0;
    }

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Unit Tests
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

#ifdef GF_MATH_UT
    void utMakeCannedPolygon(const int         polynum,
                             const EigenPoint3 &offset,
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
        {30.0,  10.0, 0.0}, {30.0,   0.0, 0.0},
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

      // 30m x 50m tee
      static const double tee[][3] =
      {
        { 0.0,  -5.0, 0.0},
        {40.0,  -5.0, 0.0},
        {40.0, -15.0, 0.0},
        {50.0, -15.0, 0.0},
        {50.0,  15.0, 0.0},
        {40.0,  15.0, 0.0},
        {40.0,   5.0, 0.0},
        { 0.0,   5.0, 0.0},
        { 0.0,  -5.0, 0.0},
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
        pt.x = poly[i][0] + offset.x();
        pt.y = poly[i][1] + offset.y();
        pt.z = poly[i][2] + offset.z();
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
