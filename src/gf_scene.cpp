////////////////////////////////////////////////////////////////////////////////
//
// Package:   Geofrenzy Robot ROS Package
//
// Link:      https://github.com/roadnarrows-robotics/geofrenzy
//
// File:      gf_scene.cpp
//
/*! \file
 *
 * \brief The Geofrenzy scene implementation.
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
#include "gf_poly.h"
#include "gf_scene.h"

using namespace std;
using namespace geofrenzy::gf_math;
using namespace geofrenzy::gf_math::gf_index;

namespace geofrenzy
{
  namespace gf_scene
  {
    // -------------------------------------------------------------------------
    // Private
    // -------------------------------------------------------------------------

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


    //--------------------------------------------------------------------------
    // Class Geofence
    //--------------------------------------------------------------------------

    Geofence::Geofence(const size_t id, const string &text)
      : m_id(id),
        m_text(text),
        m_hasCaps(false),
        m_polyhedron(id)
    {
      m_polyhedron.text(text);
    }

    Geofence::Geofence(const Geofence &src)
      : m_id(src.m_id),
        m_text(src.m_text),
        m_hasCaps(src.m_hasCaps),
        m_polyhedron(src.m_polyhedron)
    {
      m_polyhedron.text(src.m_text);
    }

    Geofence::~Geofence()
    {
    }

    void Geofence::clear()
    {
      m_id = 0;
      m_text.clear();
      m_hasCaps = false;
      m_polyhedron.clear();
    }

    ostream &operator<<(ostream &os, const Geofence &obj)
    {
      os << indent() << "{" << endl;

      bumpIndent(2);
      os << indent() << "m_id:         " << obj.m_id << endl;
      os << indent() << "m_hasCaps:    " << obj.m_hasCaps << endl;
      os << indent() << "m_text:       \"" << obj.m_text << "\"" << endl;
      os << indent() << "m_polyhedron:" << endl;
      os << obj.m_polyhedron << endl;
      bumpIndent(-2);

      os << indent() << "}";

      return os;
    }


    //--------------------------------------------------------------------------
    // Class GeofenceScene
    //--------------------------------------------------------------------------

    GeofenceScene::GeofenceScene()
      : m_geofences(4)
    {
    }

    GeofenceScene::~GeofenceScene()
    {
    }

    void GeofenceScene::addFence(size_t          fenceId,
                                 const Polygon64 &polygon,
                                 const LuxRGBA   &color,
                                 const double    fenceAlt,
                                 const double    fenceHeight,
                                 const bool      hasCaps,
                                 const string    &text)
    {
      size_t      numBaseVerts = polygon.points.size();
      double      altitude;
      double      height;
      EigenVertex v0, v1; 
      bool        isClosed = false;
      size_t      id;
      size_t      n;
      size_t      i, j;
      
      // insufficient points in polygon
      if( numBaseVerts < 2 )
      {
        return;
      }

      // fence base altitude - constrain as appropriate
      //altitude = fenceAlt >= 0.0? fenceAlt: 0.0;  // absolute altitude 
      altitude = fenceAlt;                          // relative to origin

      // fence height from base - constrain as appropriate
      height = fenceHeight >= FenceMinHeight? fenceHeight: FenceMinHeight;

      i = 0;
      j = numBaseVerts - 1;

      v0 = EigenVertex(polygon.points[i].x, polygon.points[i].y, altitude);
      v1 = EigenVertex(polygon.points[j].x, polygon.points[j].y, altitude);

      //
      // This polygon was specified with the ending vertex being a copy of the
      // first vertex. So this is a closed fence. Mark that fact, but ignore
      // the vertex.
      //
      if( isApprox(v0, v1, EpsilonLinear) )
      {
        isClosed = true;

        --numBaseVerts;

        // recheck minimum vertices
        if( numBaseVerts < 2 )
        {
          return;
        }
      }

      // add fence
      m_geofences.push_back(Geofence(fenceId, text));

      Geofence &fence = m_geofences.back();

      EigenPolyhedron &poly = fence.polyhedron();

      // each polyhedron manages its local pool of vertices
      poly.makePool();

      //
      // TODO
      // Need to find the base vertex ordering, such that, looking
      // from outside, the facets will all have a counterclockwise ordering.
      // This will then have the normals consistently point outwards.
      //
  
      // --- Fence polyhedron vertices ---
 
      //
      // Base of fence vertices
      //
      for(i = 0; i < numBaseVerts; ++i)
      {
        poly.addVertex(EigenVertex(polygon.points[i].x,
                                   polygon.points[i].y,
                                   altitude));
      }

      //
      // Top of fence
      //
      for(i = 0; i < numBaseVerts; ++i)
      {
        poly.addVertex(EigenVertex(polygon.points[i].x,
                                   polygon.points[i].y,
                                   altitude+height));
      }

      // --- Fence polyhedron facets ---
 
      //
      // Add side rectangular facets
      //
      n = isClosed? numBaseVerts: numBaseVerts-1;

      for(i = 0, j = 1; i < n; ++i)
      {
        //
        // Add vertically oriented rectangular face.
        // Vertex order: base start, base next, up to the top, top back.
        //
        id = poly.addRectangleFacet(i, j, j+numBaseVerts, i+numBaseVerts,
                  EigenFacet::VERTICAL);

        poly.facetAt(id).setAttr(color);
        poly.facetAt(id).text("wall");

        j = (j + 1) % numBaseVerts;
      }

      //
      // Make a polyhedron object by capping the closed, polyhedral surface.
      //
      if( hasCaps && isClosed )
      {
        //
        // Add ceiling
        //
        id = poly.addFacet();

        for(i = 0; i < numBaseVerts; ++i)
        {
          poly.addVertexToFacet(id, i+numBaseVerts);
        }

        poly.facetAt(id).setAttr(EigenFacet::POLYGON, EigenFacet::HORIZONTAL);
        poly.facetAt(id).setAttr(color);
        poly.facetAt(id).text("ceiling");

        poly.closeFacet(id);

        //
        // Add floor
        //
        id = poly.addFacet();

        for(ssize_t k = numBaseVerts-1; k >= 0; --k)
        {
          poly.addVertexToFacet(id, k);               // base start
        }

        poly.facetAt(id).setAttr(EigenFacet::POLYGON, EigenFacet::HORIZONTAL);
        poly.facetAt(id).setAttr(color);
        poly.facetAt(id).text("floor");

        poly.closeFacet(id);

        fence.hasCaps(true);
      }

      else
      {
        fence.hasCaps(false);
      }

      // validate and calculate properties
      poly.process();
    }

    void GeofenceScene::clear()
    {
      m_geofences.clear();
    }

    ostream &operator<<(ostream &os, const GeofenceScene &obj)
    {
      size_t  n = obj.m_geofences.size();

      os << indent() << "{" << endl;

      bumpIndent(2);
      os << indent() << "m_geofences:" << endl;
      os << indent() << "[" << endl;

      if( n-- > 0 )
      {
        bumpIndent(2);
        for(size_t i = 0; i < n; ++i)
        {
          os << obj.m_geofences[i] << "," << endl;
        }
        os << obj.m_geofences[n] << endl;
        bumpIndent(-2);
      }

      os << indent() << "]" << endl;
      bumpIndent(-2);

      os << indent() << "}" << endl;

      return os;
    }


    //--------------------------------------------------------------------------
    // Class SceneScanner
    //--------------------------------------------------------------------------

    ostream &operator<<(ostream &os, const SceneScanner &obj)
    {
      os << indent() << "{" << endl;

      bumpIndent(2);
      os << indent() << "m_name: \"" << obj.m_name << "\"" << endl;
      bumpIndent(-2);

      os << indent() << "}";

      return os;
    }


    //--------------------------------------------------------------------------
    // Class GridSceneScanner
    //--------------------------------------------------------------------------

    const double GridSceneScanner::GridSizeMin = 0.01;  // centimeter minimum
    const double GridSceneScanner::GridSizeDft = 0.20;  // default grid size

    GridSceneScanner::GridSceneScanner()
        : SceneScanner("GridScanner"),
          m_options(ScanOptionDft)
    {
      setProperties(GridSizeDft);
    }

    GridSceneScanner::GridSceneScanner(const double   gridSize,
                                       const uint32_t options)
        : SceneScanner("GridScanner"),
          m_options(options)
    {
      setProperties(gridSize);
    }

    GridSceneScanner::~GridSceneScanner()
    {
    }

    void GridSceneScanner::setProperties(const double gridSize)
    {
      m_gridSize = gridSize >= GridSizeMin? gridSize: GridSizeMin;
    }

    size_t GridSceneScanner::scan(const GeofenceScene &scene,
                                  PointCloudList      &intersects)
    {
      GeofenceCIter   iterFence;
      EigenFacetCIter iterFacet;

      //
      // Loop thruough scene objects.
      //
      for(iterFence = scene.iterFenceBegin();
          iterFence != scene.iterFenceEnd();
          ++iterFence)
      {
        const EigenPolyhedron &poly = iterFence->polyhedron();

        //
        // Loop through all of a scene object's polyhedral surfaces to grid.
        //
        for(iterFacet = poly.iterFacetBegin();
            iterFacet != poly.iterFacetEnd();
            ++iterFacet)
        {
          switch( iterFacet->orientation() )
          {
            case EigenFacet::VERTICAL:
              gridWall(*iterFacet, intersects);
              break;
            case EigenFacet::HORIZONTAL:
              gridCap(*iterFacet, intersects);
              break;
            default:
              // error - unsupported orientation
              break;
          }
        }
      }
    }

    size_t GridSceneScanner::gridWall(const EigenFacet &facet,
                                      PointCloudList   &intersects)
    {
      double        alpha;
      EigenPoint2   pt_xy;
      EigenPoint3   pt;
      EigenLine3    baseline;
      size_t        n = 0;
      double        len;

      // base
      const EigenPoint3 &pt0 = facet.vertexAt(0);
      const EigenPoint3 &pt1 = facet.vertexAt(1);

      // dimension
      const EigenPoint2 &dim = facet.dimensions();

      //
      // Determine the surface base line angle in the x-y plane (-pi, pi].
      //
      alpha = atan2(pt1.y() - pt0.y(), pt1.x() - pt0.x());

      //
      // Find the point gridSize units away from a line through the origin
      // and parallel to the baseline.
      //
      pt_xy = polarToCartesian(m_gridSize, alpha);

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
      for(len = 0.0; len < dim[_L]; len += m_gridSize)
      {
        // starting base point
        pt = baseline.pointAt(len);

        // grid surface vertical strip
        n += gridWallStripe(facet, pt, intersects);
      }

      //
      // If the last stripe is not sufficiently close to the surface far end,
      // then add an end strip of grid points.
      //
      if( !isApprox(len-m_gridSize, dim[_L], EpsilonLinear) )
      {
        n += gridWallStripe(facet, pt1, intersects);
      }

      return n;
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
    size_t GridSceneScanner::gridWallStripe(const EigenFacet  &facet,
                                            const EigenPoint3 &basept,
                                            PointCloudList    &intersects)
    {
      EigenPoint3   pt = basept;
      EigenXYZRGBA  xyzrgba;
      size_t        n = 0;
      double        h;

      // dimension
      const EigenPoint2 &dim = facet.dimensions();

      //
      // Loop from starting base point to surface top at the fixed x-y position.
      //
      for(h = pt.z(); h < basept.z() + dim[_W]; h += m_gridSize)
      {
        pt.z() = h;

        xyzrgba << pt, facet.color();

        intersects.push_back(xyzrgba);

        ++n;
      }

      //
      // If the last grid point is not sufficiently close to the surface top,
      // add the top point.
      //
      if( !isApprox(h-m_gridSize, dim[_W], EpsilonLinear) )
      {
        pt.z() = basept.z() + dim[_W];

        xyzrgba << pt, facet.color();

        intersects.push_back(xyzrgba);

        ++n;
      }

      return n;
    }

    size_t GridSceneScanner::gridCap(const EigenFacet &facet,
                                     PointCloudList   &intersects)
    {
      EigenPoint3   pt;         // working point
      EigenXYZRGBA  xyzrgba;    // xyz rgba point
      size_t        n = 0;      // points added

      const EigenBoundary3  &bounds = facet.xyzRange();

      pt.z() = (bounds.m_min.z() + bounds.m_max.z()) / 2.0;

      //
      // Loop over object's bounding box for calculate ceiling points.
      //
      for(pt.x() = bounds.m_min.x();
          pt.x() <= bounds.m_max.x();
          pt.x() += m_gridSize)
      {
        for(pt.y() = bounds.m_min.y();
            pt.y() <= bounds.m_max.y();
            pt.y() += m_gridSize)
        {
          if( pipCnZ(pt, facet.vertices().xyz) )
          {
            xyzrgba << pt, facet.color();
            intersects.push_back(xyzrgba);
            ++n;
          }
        }
      }

      return n;
    }

    ostream &operator<<(ostream &os, const GridSceneScanner &obj)
    {
      os << indent() << "{" << endl;

      bumpIndent(2);

      // base class
      os << indent() << "scanner:" << endl;
      os << (SceneScanner&)obj << endl;

      os << indent() << "m_gridSize: " << obj.m_gridSize << endl;
      os << indent() << "m_options:  " << "0x" << hex << obj.m_options << dec
        << endl;

      bumpIndent(-2);

      os << indent() << "}";

      return os;
    }


    //--------------------------------------------------------------------------
    // Class SensorSceneScanner
    //--------------------------------------------------------------------------

    const size_t SensorSceneScanner::WidthMin   = 2;
    const size_t SensorSceneScanner::WidthDft   = 160;

    const size_t SensorSceneScanner::HeightMin  = 2;
    const size_t SensorSceneScanner::HeightDft  = 120;

    const double SensorSceneScanner::HFoVMinMin = -M_PI;
    const double SensorSceneScanner::HFoVMaxMax = M_PI;
    const double SensorSceneScanner::HFoVMinDft = -M_PI_2;
    const double SensorSceneScanner::HFoVMaxDft = M_PI_2;

    const double SensorSceneScanner::VFoVMinMin = 0.0;
    const double SensorSceneScanner::VFoVMaxMax = M_PI;
    const double SensorSceneScanner::VFoVMinDft = M_PI_4;
    const double SensorSceneScanner::VFoVMaxDft = M_PI_2;

    SensorSceneScanner::SensorSceneScanner()
        : SceneScanner("SensorScanner"),
          m_options(ScanOptionDft)
    {
      setProperties(HFoVMinDft, HFoVMaxDft,
                    VFoVMinDft, VFoVMaxDft,
                    WidthDft, HeightDft);
    }

    SensorSceneScanner::SensorSceneScanner(const double   thetaMin,
                                           const double   thetaMax,
                                           const double   phiMin,
                                           const double   phiMax,
                                           const size_t   width,
                                           const size_t   height,
                                           const uint32_t options)
        : SceneScanner("SensorScanner"),
          m_options(options)
    {
      setProperties(thetaMin, thetaMax, phiMin, phiMax, width, height);
    }

    SensorSceneScanner::~SensorSceneScanner()
    {
    }

    void SensorSceneScanner::setProperties(const double thetaMin,
                                           const double thetaMax,
                                           const double phiMin,
                                           const double phiMax,
                                           const size_t width,
                                           const size_t height)
    {
      m_thetaMin = cap(thetaMin, HFoVMinMin, HFoVMaxMax);
      m_thetaMax = cap(thetaMax, m_thetaMin, HFoVMaxMax);

      m_phiMin = cap(phiMin, VFoVMinMin, VFoVMaxMax);
      m_phiMax = cap(phiMax, m_phiMin, VFoVMaxMax);

      m_width  = width >= WidthMin? width: WidthMin;
      m_height = height >= HeightMin? height: HeightMin;

      m_thetaStep = (m_thetaMax - m_thetaMin) / (double)(m_width - 1);
      m_phiStep   = (m_phiMax - m_phiMin) / (double)(m_height - 1);
    }

    size_t SensorSceneScanner::scan(const GeofenceScene &scene,
                                    PointCloudList      &intersects)
    {
      double        phi, theta;

      //
      // Loop through scene top to bottom.
      //
      for(phi = m_phiMin; phi <= m_phiMax; phi += m_phiStep)
      {
        //
        // Loop through scene counter-clockwise.
        //
        for(theta = m_thetaMin; theta <= m_thetaMax; theta += m_thetaStep)
        {
          traceRay(theta, phi, scene, intersects);
        }
      }
    }

    size_t SensorSceneScanner::traceRay(const double        theta,
                                        const double        phi,
                                        const GeofenceScene &scene,
                                        PointCloudList      &intersects)
    {
      bool            hasRay = false;   // lazy init boolean for ray
      GeofenceCIter   iterFence;
      EigenFacetCIter iterFacet;

      EigenPoint3   pt;               // working point
      EigenLine3    ray;              // vsensor ray
      double        t;                // intersection parameter value
      ScenePoI      poi;              // ray point of interest info
      size_t        i, j;             // working indices
      RayPoIMap     rayPoI;           // ordered map of ray points of interest

      //
      // Loop through all the scene geofences at set theta,phi.
      //
      for(iterFence  = scene.iterFenceBegin();
          iterFence != scene.iterFenceEnd();
          ++iterFence)
      {
        const Geofence &fence = *iterFence;

        // check if fence is in view
        if( !fence.polyhedron().inview(theta, phi) )
        {
          continue;
        }

        const EigenPolyhedron &poly = fence.polyhedron();

        //
        // Loop through all the fence facets to find all ray intersections.
        //
        for(iterFacet = poly.iterFacetBegin();
            iterFacet != poly.iterFacetEnd();
            ++iterFacet)
        {
          const EigenFacet &facet = *iterFacet;

          // check if facet is in view
          if( !facet.inview(theta, phi) )
          {
            continue;
          }

          // lazy init of ray (performance tweak)
          if( !hasRay )
          {
            pt  = sphericalToCartesian(1.0, theta, phi);
            ray = EigenLine3::Through(Origin3, pt);
            hasRay = true;
          }

          if( facet.intersection(ray, t, pt) )
          {
            // ray intersection info
            poi.m_rgba = facet.color();
            poi.m_xyz  = pt;

            // add to ascending distance ordered list
            rayPoI[t] = poi;
          }
        }
      }

      return postproc(rayPoI, intersects);
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
    size_t SensorSceneScanner::postproc(RayPoIMap      &rayPoI,
                                        PointCloudList &intersects)
    {
      //
      // No intersections.
      //
      if( rayPoI.empty() )
      {
        if( SCANOPT_2D(m_options) )
        {
          intersects.push_back(InfPt);
          return 1;
        }
        else
        {
          return 0;
        }
      }

      EigenRGBA           colorBlend;   // alpha blended color
      RayPoIMap::iterator iter;         // iterator

      //
      // Alpha blend color(s).
      //
      if( SCANOPT_ALPHA_BLEND(m_options) )
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
        if( SCANOPT_ALPHA_BLEND(m_options) )
        {
          pushIntersect(intersects, iter->second.m_xyz, colorBlend);
        }
        else
        {
          pushIntersect(intersects, iter->second);
        }

        // add only the nearest intersecting point
        if( SCANOPT_NEAREST_ONLY(m_options) )
        {
          return 1;
        }
      }

      // all points of interest were added
      return rayPoI.size();
    }

    ostream &operator<<(ostream &os, const SensorSceneScanner &obj)
    {
      os << indent() << "{" << endl;

      bumpIndent(2);

      // base class
      os << indent() << "scanner:" << endl;
      os << (SceneScanner&)obj << endl;

      os << indent() << "hfov:        " 
        << "[" << degrees(obj.m_thetaMin) << ", " << degrees(obj.m_thetaMax)
        << "]" << endl;
      os << indent() << "vfov:        " 
        << "[" << degrees(obj.m_phiMin) << ", " << degrees(obj.m_phiMax) << "]"
        << endl;
      os << indent() << "resolution:  " << obj.m_width << "x" << obj.m_height
        << endl;
      os << indent() << "m_thetaStep: " << obj.m_thetaStep << endl;
      os << indent() << "m_phiStep:   " << obj.m_phiStep << endl;
      os << indent() << "m_options:   " << "0x" << hex << obj.m_options << dec
        << endl;

      bumpIndent(-2);

      os << indent() << "}";

      return os;
    }


    //--------------------------------------------------------------------------
    // Unit Tests
    //--------------------------------------------------------------------------

#ifdef GF_SCENE_UT
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
#endif // GF_SCENE_UT

  } // namespace gf_scene

} // namespace geofrenzy
