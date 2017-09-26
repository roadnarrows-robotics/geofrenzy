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
// RDK Notes:
// 1. Use openscenegraph

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

namespace geofrenzy
{
  namespace gf_scene
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
      LuxRGBA     m_rgba;   ///< rgba color
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
    static inline void pushIntersect(PointCloudList &intersects,
                                     EigenPoint3    &xyz,
                                     LuxRGBA        &rgba)
    {
      EigenXYZRGBA  pt;

      pt << xyz, rgba;

      intersects.push_back(pt);
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


    //--------------------------------------------------------------------------
    // Class GeofenceScene
    //--------------------------------------------------------------------------

    GeofenceScene::GeofenceScene()
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
      altitude = fenceAlt >= 0.0? fenceAlt: 0.0;

      // fence height from base - constrain as appropriate
      height = fenceHeight >= FenceMinHeight? fenceHeight: FenceMinHeight;

      i = 0;
      j = numBaseVerts - 1;

      v0 = EigenVertex(polygon.points[i].x, polygon.points[i].y, height);
      v1 = EigenVertex(polygon.points[j].x, polygon.points[j].y, height);

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
      //GeofenceObj fenceObj;
      m_geofences.push_back(GeofenceObj());

      GeofenceObj &fenceObj = m_geofences.back();

      fenceObj.m_id   = fenceId;
      fenceObj.m_text = text;

      EigenPolyhedron &poly = fenceObj.m_polyhedron;

      //
      // TODO RDK
      // Need to find the base vertex ordering, such that, looking
      // from outside, the facets will all have a counterclockwise ordering.
      // This will then have the normals consistently point outwards.
      //
  
      //
      // Base of fence
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

      //
      // Add side rectangular facets
      //
      n = isClosed? numBaseVerts: numBaseVerts-1;

      for(i = 0, j = 1; i < n; ++i)
      {

#if 0 // RDK
        id = poly.addFacet();

        poly.addVertexToFacet(id, i);               // base start
        poly.addVertexToFacet(id, j);               // base next
        poly.addVertexToFacet(id, j+numBaseVerts);  // up to the top
        poly.addVertexToFacet(id, i+numBaseVerts);  // top back

        poly.closeFacet(id);
#endif // RDK

        //
        // Add vertically oriented rectangular face.
        // Vertex order: base start, base next, up to the top, top back.
        //
        id = poly.addRectangleFacet(i, j, j+numBaseVerts, i+numBaseVerts,
                  EigenFacet::VERTICAL);

        poly.facetAt(id).setAttr(color);

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
          poly.addVertexToFacet(id, i);               // base start
        }

        poly.closeFacet(id);

        poly.facetAt(id).setAttr(EigenFacet::POLYGON, EigenFacet::HORIZONTAL);
        poly.facetAt(id).setAttr(color);

        //
        // Add floor
        //
          id = poly.addFacet();

        for(ssize_t k = numBaseVerts-1; k >= 0; --k)
        {
          poly.addVertexToFacet(id, i);               // base start
        }

        poly.closeFacet(id);

        poly.facetAt(id).setAttr(EigenFacet::POLYGON, EigenFacet::HORIZONTAL);
        poly.facetAt(id).setAttr(color);

        fenceObj.m_hasCaps = true;
      }

      else
      {
        fenceObj.m_hasCaps = false;
      }
    }

    void GeofenceScene::clear()
    {
      m_geofences.clear();
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
      GeofenceObjCIter  iterFence;
      EigenFacetCIter   iterFacet;

      //
      // Loop thruough scene objects.
      //
      for(iterFence = scene.iterFenceBegin();
          iterFence != scene.iterFenceEnd();
          ++iterFence)
      {
        const EigenPolyhedron &poly = iterFence->m_polyhedron;

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

      // diminsion
      const EigenPoint2 &dim = facet.dim();

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
        //n += gridWallStripe(facet, pt, intersects);
      }

      //
      // If the last stripe is not sufficiently close to the surface far end,
      // then add an end strip of grid points.
      //
      if( !isApprox(len-m_gridSize, dim[_L], EpsilonLinear) )
      {
        //n += gridWallStripe(facet, pt1, intersects);
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

      // diminsion
      const EigenPoint2 &dim = facet.dim();

      //
      // Loop from starting base point to surface top at the fixed x-y position.
      //
      for(h = pt.z(); h < dim[_W]; h += m_gridSize)
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
        pt.z() = dim[_W];

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

      const EigenBoundary3  &bounds = facet.boundary();

      pt.z() = bounds.m_max.z();

      //
      // Loop over object's bounding box.
      //
      for(pt.x() = bounds.m_min.x();
          pt.x() <= bounds.m_max.x();
          pt.x() += m_gridSize)
      {
        for(pt.y() = bounds.m_min.y();
            pt.y() <= bounds.m_max.y();
            pt.y() += m_gridSize)
        {
#if 0 // RDK
          if( pipCn(xy(pt), sceneObj.m_footprint) )
          {
            xyzrgba << pt, sceneObj.m_color;
            intersects.push_back(xyzrgba);
            ++n;
          }
#endif // RDK
        }
      }

      // TODO: floor

      return n;
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
    }


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Make a Scene Functions
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .


    static void circumscribe(EigenSurface &surface)
    {
    }

    static void calcFoV(EigenSurface &surface)
    {
      double      thetaMin, thetaMax;
      double      phiMin, phiMax;
      EigenPoint3 rtp;

      rtp = cartesianToSpherical(surface.m_bounds.m_min.x(),
                                 surface.m_bounds.m_min.y(),
                                 surface.m_bounds.m_min.z());

      thetaMin = thetaMax = rtp[_THETA];
      phiMin = phiMax = rtp[_PHI];


    }

    /*!
     * \brief Make a bounding box around sets of x,y,z ranges.
     *
     * \param x0,x1       X range.
     * \param y0,y1       Y range.
     * \param z0,z1       Z range.
     * \param epsilon     Bounding region precision.
     * \param[out] bounds   Output bounding box.
     */
    static void mkBBox(const double x0, const double x1,
                       const double y0, const double y1,
                       const double z0, const double z1,
                       const double epsilon,
                       EigenBoundary3   &bounds)
    {
      // min,max x
      if( x0 <= x1 )
      {
        bounds.m_min.x() = x0 - epsilon;
        bounds.m_max.x() = x1 + epsilon;
      }
      else
      {
        bounds.m_min.x() = x1 - epsilon;
        bounds.m_max.x() = x0 + epsilon;
      }
      
      // min,max y
      if( y0 <= y1 )
      {
        bounds.m_min.y() = y0 - epsilon;
        bounds.m_max.y() = y1 + epsilon;
      }
      else
      {
        bounds.m_min.y() = y1 - epsilon;
        bounds.m_max.y() = y0 + epsilon;
      }
      
      // min,max z
      if( z0 <= z1 )
      {
        bounds.m_min.z() = z0 - epsilon;
        bounds.m_max.z() = z1 + epsilon;
      }
      else
      {
        bounds.m_min.z() = z1 - epsilon;
        bounds.m_max.z() = z0 + epsilon;
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
     * \brief Create scene object surface properties.
     *
     * \param           x0,x1    X range.
     * \param           y0,y1    Y range.
     * \param           z0,z1    Z range.
     * \param [in,out]  surface  Scene object surface.
     */
    static void createSurface(const double x0, const double x1,
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

      // surface base altitude
      surface.m_altitude = z0;

      // length of surface base line
      surface.m_length = L2Dist(surface.m_vertices[0], surface.m_vertices[1]);

      // total height of the surface
      surface.m_height = fabs(z1 - z0);

      // base line inclination
      surface.m_inclination = inclination(pt0, pt1);

      // origin projection (distance) from extended base line 
      surface.m_projection = projection(pt0, pt1);

      // surface bounding box
      mkBBox(x0, x1, y0, y1, z0, z1, EpsilonLinear, surface.m_bounds);

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
                             const LuxRGBA        &color,
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
        pt = ray.pointAt(t);                  // determine point along the ray
        pt = surface.m_plane.projection(pt);  // project the point into plane

        // add point of interest if within surface
        if( (isValid = inbounds(pt, surface.m_bounds)) )
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
    static size_t postprocPoI(RayOrderedPoI  &rayPoI,
                              PointCloudList &intersects,
                              uint32_t       options)
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
     * The spherical coordinates are as used in mathematics.
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
                           PointCloudList   &intersects,
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
          if( !withinOneOf(theta, surface.m_thetas) )
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
              projectRay(thetaNearest, phi, ParallelStepInit, ParallelStepRatio,
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
              pt  = sphericalToCartesian(1.0, theta, phi);
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
            if( !inbounds(pt, surface.m_bounds) )
            {
              continue;
            }

            // ray intersection info
            poi.m_rgba = color;
            poi.m_xyz  = pt;

            // add to ascending distance ordered list
            rayPoI[t] = poi;
          }

          if( sceneObj.m_hasCaps )
          {
            //traceRayCeiling(ray, sceneObj, rayPoI);
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
                                    PointCloudList     &intersects,
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
                              PointCloudList     &intersects,
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

    /*!
     * \brief Grid fence caps to generate a list of depth + color
     * points.
     *
     * \param       gridSize    Grid size.
     * \param       sceneObj    Scene object.
     * \param[out]  intersects  List of intersecting points.
     * \param       options     Options to control the grid generations.
     *
     * \return Number of points generated.
     */
    static size_t gridSceneCaps(const double        gridSize,
                                const EigenSceneObj &sceneObj,
                                PointCloudList      &intersects,
                                uint32_t            options)
    {
      const EigenBoundary3  &bounds = sceneObj.m_bounds;  // object's bounding box

      double        ceiling = bounds.m_max.z();   // ceiling z coordinate
      EigenPoint3   pt;                         // working point
      EigenXYZRGBA  xyzrgba;                    // xyz rgba point
      size_t        n = 0;                      // points added

      pt.z() = ceiling;

      //
      // Loop over object's bounding box.
      //
      for(pt.x() = bounds.m_min.x(); pt.x() <= bounds.m_max.x(); pt.x() += gridSize)
      {
        for(pt.y()=bounds.m_min.y(); pt.y() <= bounds.m_max.y(); pt.y() += gridSize)
        {
          if( pipCn(xy(pt), sceneObj.m_footprint) )
          {
            xyzrgba << pt, sceneObj.m_color;
            intersects.push_back(xyzrgba);
            ++n;
          }
        }
      }

      // TODO: floor

      return n;
    }


    // -------------------------------------------------------------------------
    // Public
    // -------------------------------------------------------------------------
          
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Stream operators.
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
 
    ostream &operator<<(ostream &os, const EigenSurface &surface)
    {
      size_t  nverts = surface.m_vertices.size();

      os << "    {" << endl;
      os << "      surface_num:  " << surface.m_num << endl;

      os << "      vertices:" << endl;
      os << "      [" << endl;
      for(size_t i = 0; i < nverts - 1; ++i)
      {
        os << "        "; print(os, surface.m_vertices[i]); os << "," << endl;
      }
      if( nverts > 0 )
      {
        os << "        "; print(os, surface.m_vertices[nverts-1]); os << endl;
      }
      os << "      ]" << endl;

      os << "      plane:        "; print(os, surface.m_plane); os << endl;
      os << "      altitude:     " << surface.m_altitude << endl;
      os << "      length:       " << surface.m_length << endl;
      os << "      height:       " << surface.m_height << endl;
      os << "      inclination:  " << degrees(surface.m_inclination) << endl;
      os << "      projection:   " << surface.m_projection << endl;
      os << "      bounds:       " << surface.m_bounds << endl;

      os << "      theta_limits: " 
          << "["
            << degrees(surface.m_thetas.m_min[0]) << ", "
            << degrees(surface.m_thetas.m_max[0])
          << "], "
          << "["
            << degrees(surface.m_thetas.m_min[1]) << ", "
            << degrees(surface.m_thetas.m_max[1])
          << "], "
          << endl;

      os << "      subtended:    " << degrees(surface.m_subtended) << endl;
      os << "    }";
          
      return os;
    }

    ostream &operator<<(ostream &os, const EigenSceneObj &sceneObj)
    {
      size_t  nsurfs = sceneObj.m_surfaces.size();
      size_t  i;

      os << "{" << endl;
      os << "  color:    "; print(os, sceneObj.m_color); os << endl;
      os << "  has_caps: " << sceneObj.m_hasCaps << endl;
      os << "  bounds:   " << sceneObj.m_bounds << endl;

      os << "  footprint:" << endl
         << "  [" << endl;
      for(i = 0; i< sceneObj.m_footprint.size(); ++i)
      {
        os << "    "; print(os, sceneObj.m_footprint[i]); os << "," << endl;
      }
      os << "  ]" << endl;

      os << "  surfaces:" << endl
         << "  [" << endl;
      for(i = 0; i < nsurfs - 1; ++i)
      {
        os << sceneObj.m_surfaces[i] << "," << endl;
      }
      if( nsurfs > 0 )
      {
        os << sceneObj.m_surfaces[i] << endl;
      }
      os  << "  ]" << endl;

      os << "}";

      return os;
    }


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Scene Functions
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    void createSceneObj(const Polygon64 &polygon,
                        const LuxRGBA   &color,
                        const double    &fenceAlt,
                        const double    &fenceHeight,
                        const bool      hasCaps,
                        EigenSceneObj   &sceneObj)
    {
      size_t            numPoints = polygon.points.size();
      EigenPoint2       pt;
      EigenSurface      surface;
      double            altitude;
      double            height;
      size_t            i, j;

      // clear
      sceneObj.clear();

      // insufficient points in polygon
      if( numPoints < 2 )
      {
        return;
      }

      // fence base altitude - constrain as appropriate
      altitude = fenceAlt >= 0.0? fenceAlt: 0.0;

      // fence height from base - constrain as appropriate
      height = fenceHeight >= FenceMinHeight? fenceHeight: FenceMinHeight;

      //
      // Attrubutes
      //
      sceneObj.m_color    = color;
      sceneObj.m_hasCaps  = hasCaps;

      //
      // Initialize object's bounding 3D box
      //
      mkBBox(polygon.points[0].x, polygon.points[0].x,
             polygon.points[0].y, polygon.points[0].y,
             altitude, altitude + height,
             0.0, sceneObj.m_bounds);

      //
      // Base polygonal footprint.
      //
      for(i = 0; i < numPoints; ++i)
      {
        pt.x() = polygon.points[i].x;
        pt.y() = polygon.points[i].y;

        // expand object's bounding box
        if( pt.x() < sceneObj.m_bounds.m_min.x() )
        {
          sceneObj.m_bounds.m_min.x() = pt.x();
        }
        if( pt.y() < sceneObj.m_bounds.m_min.y() )
        {
          sceneObj.m_bounds.m_min.y() = pt.y();
        }
        if( pt.x() > sceneObj.m_bounds.m_max.x() )
        {
          sceneObj.m_bounds.m_max.x() = pt.x();
        }
        if( pt.y() > sceneObj.m_bounds.m_max.y() )
        {
          sceneObj.m_bounds.m_max.y() = pt.y();
        }

        sceneObj.m_footprint.push_back(pt);
      }

      // make sure footprint is closed
      if( (sceneObj.m_footprint[0].x() != pt.x()) ||
          (sceneObj.m_footprint[0].y() != pt.y()) )
      {
        sceneObj.m_footprint.push_back(sceneObj.m_footprint[0]);
        ++numPoints;
      }

      //
      // Loop through polygon points to create scene object surfaces.
      //
      for(i = 0, j = 1; j < numPoints; ++i, ++j)
      {
        sceneObj.m_surfaces.push_back(surface);

        sceneObj.m_surfaces.back().m_num = i;

        createSurface(polygon.points[i].x, polygon.points[j].x,
                         polygon.points[i].y, polygon.points[j].y,
                         altitude, altitude+height,
                         sceneObj.m_surfaces.back());

        // cerr << sceneObj.m_surfaces.back() << endl;
      }

      cerr << "Created scene object: " << endl;
      cerr << sceneObj << endl;
    }

    void scanScene(const double thetaMin, const double thetaMax,
                   const double phiMin,   const double phiMax,
                   const size_t width,    const size_t height,
                   const EigenScene       &scene,
                   PointCloudList         &intersects,
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
                   PointCloudList   &intersects,
                   uint32_t         options)
    {
      size_t  i, j;

      //
      // Loop thruough scene objects.
      //
      for(i = 0; i < scene.size(); ++i)
      {
        const EigenSceneObj &sceneObj = scene[i];
        const LuxRGBA       &color    = sceneObj.m_color; 

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

        if( sceneObj.m_hasCaps )
        {
          gridSceneCaps(gridSize, sceneObj, intersects, options);
        }
      }
    }


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Unit Tests
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

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

    void utScanPolygon(const Polygon64 &polygon)
    {
      EigenScene      scene;
      EigenSceneObj   sceneObj;
      PointCloudList  intersects;

      scene.push_back(sceneObj);

      cout << "Create scene" << endl;

      createSceneObj(polygon, FenceColorDft, 0.0, 100.0, false, scene.back());

      cout << "Scan scene" << endl;

      scanScene(radians(-90.0), radians(90.0),
                radians(45.0),  radians(90.0),
                160, 120,
                scene, intersects, 0);

      cout << "Out ";
      cout << intersects.size() << endl;
    }
#endif // GF_SCENE_UT

  } // namespace gf_scene

} // namespace geofrenzy
