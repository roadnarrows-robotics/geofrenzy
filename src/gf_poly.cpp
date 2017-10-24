////////////////////////////////////////////////////////////////////////////////
//
// Package:   Geofrenzy Robot ROS Package
//
// Link:      https://github.com/roadnarrows-robotics/geofrenzy
//
// File:      gf_poly.cpp
//
/*! \file
 *
 * \brief The Geofrenzy polygon and polyhedron math definitions.
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
#include "gf_poly.h"

using namespace std;
using namespace Eigen;


//------------------------------------------------------------------------------
// Eigen Polygons and Polyhedron
//------------------------------------------------------------------------------

namespace geofrenzy
{
  namespace gf_math
  {
    using namespace gf_index;

    //--------------------------------------------------------------------------
    // Struct EigenExVertexList
    //--------------------------------------------------------------------------

    void EigenExVertexList::push_back(const EigenVertex &v,
                                      const Coordinates coord)
    {
      switch( coord )
      {
        case Cartesian:
          xyz.push_back(v);
          rtp.push_back(cartesianToSpherical(v));
          break;
        case Spherical:
          rtp.push_back(v);
          xyz.push_back(sphericalToCartesian(v));
          break;
        default:
          DBG_POLY("Coordinates " << coord << " not supported");
          break;
      }
    }

    ostream &operator<<(ostream &os, const EigenExVertexList &obj)
    {
      os << indent() << "{" << endl;

      bumpIndent(2);

      os << indent() << "xyz:" << endl;
      os << indent() << "[";

      bumpIndent(2);
      print(os, obj.xyz, 2);
      bumpIndent(-2);

      os << indent() << "]" << endl;

      os << indent() << "rtp:" << endl;
      os << indent() << "[";

      bumpIndent(2);
      print(os, obj.rtp, 2, UnitsDegrees);
      bumpIndent(-2);

      os << indent() << "]" << endl;

      bumpIndent(-2);

      os << indent() << "}";

      return os;
    }


    //--------------------------------------------------------------------------
    // Class EigenVertexPool
    //--------------------------------------------------------------------------

    EigenVertexPool::EigenVertexPool(size_t id)
    {
      DBG_POLY("id = " << id);

      m_id        = id;
      m_refcnt    = 0;
      m_capacity  = PoolCapacityDft;

      m_vertices.capacity(m_capacity);
    }

    EigenVertexPool::EigenVertexPool(const EigenVertexPool &src)
    {
      DBG_POLY("src(id) = " << src.m_id);

      m_id        = src.m_id;
      m_refcnt    = 0;

      m_vertices.capacity(src.m_capacity);
      m_vertices = src.m_vertices;
    }

    EigenVertexPool::~EigenVertexPool()
    {
      DBG_POLY("id = " << m_id);

      if( m_refcnt > 0 )
      {
        cerr << "Warning: Pool reference count of "
          << m_refcnt << " indicates pool is in use." << endl;
      }
    }

    ostream &operator<<(ostream &os, const EigenVertexPool &obj)
    {
      os << indent() << "{" << endl;

      bumpIndent(2);
      os << indent() << "m_id:       " << obj.m_id << endl;
      os << indent() << "m_refcnt:   " << obj.m_refcnt << endl;
      os << indent() << "m_capactiy: " << obj.m_capacity << endl;
      os << indent() << "m_vertices: " << endl;
      os << obj.m_vertices << endl;
      bumpIndent(-2);

      os << indent() << "}";

      return os;
    }


    //--------------------------------------------------------------------------
    // Class EigenGeoObj
    //--------------------------------------------------------------------------

    EigenGeoObj::EigenGeoObj(const size_t id, EigenVertexPool *pool)
        : m_id(id),
          m_madePool(false),
          m_pool(pool)
    {
      DBG_POLY("id = " << m_id);

      m_isValid  = false;

      bindPool();
      capacity(VerticesCapacityDft);

      seedBounds(EigenPoint3(0, 0, 0), m_xyzRange);
      seedBounds(EigenPoint3(0, 0, 0), m_rtpRange);
    }

    EigenGeoObj::EigenGeoObj(const EigenGeoObj &src)
    {
      DBG_POLY("src(id) = " << src.m_id);

      m_id        = src.m_id;
      m_text      = src.m_text;
      m_isValid   = src.m_isValid;
      m_madePool  = false;
      m_pool      = src.m_pool;
      m_vertices  = src.m_vertices;
      m_xyzRange  = src.m_xyzRange;
      m_rtpRange  = src.m_rtpRange;
      m_subtended = src.m_subtended;

      bindPool();
      capacity(src.m_capacity);
    }

    EigenGeoObj::~EigenGeoObj()
    {
      DBG_POLY("id = " << m_id);

      unbindPool();
    }

    void EigenGeoObj::clear()
    {
      m_isValid = false;

      m_iindices.clear();
      m_vertices.clear();
      seedBounds(EigenPoint3(0, 0, 0), m_xyzRange);
      seedBounds(EigenPoint3(0, 0, 0), m_rtpRange);
      m_subtended.clear();
    }

    IndexList EigenGeoObj::addVertices(const IndexList &indices)
    {
      size_t n = indices.size();

      IndexList vindices;

      if( n > 0 )
      {
        m_isValid = false;

        vindices.reserve(n);

        for(size_t i = 0; i < n; ++i)
        {
          vindices.push_back(copyFromPool(indices[i]));
        }  
      }

      return vindices;
    }

    IndexList EigenGeoObj::addVertices(const EigenVertexList &vertices)
    {
      size_t n = vertices.size();

      IndexList vindices;

      if( n > 0 )
      {
        m_isValid = false;

        vindices.reserve(n);

        for(size_t i = 0; i < n; ++i)
        {
          vindices.push_back(writeThrough(vertices[i]));
        }  
      }

      return vindices;
    }

    void EigenGeoObj::process()
    {
      seedBounds(m_vertices.xyz[0], m_xyzRange);
      seedBounds(m_vertices.rtp[0], m_rtpRange);

      for(size_t i = 1; i < m_vertices.size(); ++i)
      {
        growBounds(m_vertices.xyz[i], m_xyzRange);
        growBounds(m_vertices.rtp[i], m_rtpRange);
      }
    }

    void EigenGeoObj::makePool()
    {
      // unbind from any existing bound pool
      unbindPool();

      // allocate new local pool
      m_pool = new EigenVertexPool(m_id);

      // bind to new local pool
      bindPool();

      // clear indirect indices map
      m_iindices.clear();

      //
      // Copy object's vertices to new pool.
      //
      for(size_t vi = 0; vi < m_vertices.size(); ++vi)
      {
        size_t pi = m_pool->addVertex(m_vertices[vi]);

        m_iindices[vi] = pi;
      }

      m_madePool = true;
    }

    size_t EigenGeoObj::copyFromPool(const size_t i)
    {
      assert(m_pool != NULL);

      size_t  vi = m_vertices.size();

      m_vertices.push_back(m_pool->vertexAt(i), m_pool->sphericalAt(i));
      
      m_iindices[vi] = i;

      return vi;
    }

    size_t EigenGeoObj::writeThrough(const EigenVertex &v)
    {
      // bound pool - add to pool and copy to object's vertices
      if( m_pool != NULL )
      {
        size_t pi = m_pool->addVertex(v);
        return copyFromPool(pi);
      }

      // no bound pool - simply add to object's vertices
      else
      {
        size_t  vi = m_vertices.size();

        m_vertices.push_back(v, cartesianToSpherical(v));

        return vi;
      }
    }

    ostream &operator<<(ostream &os, const EigenGeoObj &obj)
    {
      os << indent() << "{" << endl;

      bumpIndent(2);
      os << indent() << "m_id:       " << obj.m_id << endl;
      os << indent() << "m_text:     \"" << obj.m_text << "\"" << endl;
      os << indent() << "m_isValid:  " << obj.m_isValid << endl;

      os << indent() << "m_pool:     " << (void *)obj.m_pool << endl;

      // only print once
      if( obj.m_madePool )
      {
        os << *obj.m_pool << endl;
      }

      os << indent() << "m_capacity: " << obj.m_capacity << endl;

      os << indent() << "m_iindices:" << endl;
      os << indent() << "[";

      size_t  n = obj.m_iindices.size();

      if( n > 0 )
      {
        bumpIndent(2);

        size_t i;
        IIndexMap::const_iterator iter;

        for(i = 0, iter = obj.m_iindices.begin();
            iter != obj.m_iindices.end();
            ++i, ++iter)
        {
          if( (i % 16) == 0 )
          {
            os << endl << indent();
          }
          os << iter->second;
          if( i < n-1 )
          {
            os << ", ";
          }
        }
        os << endl;

        bumpIndent(-2);
      }
      os << indent() << "]" << endl;

      os << indent() << "m_vertices: " << endl;
      os << obj.m_vertices << endl;

      os << indent() << "m_xyzRange:  " << obj.m_xyzRange << endl;
      os << indent() << "m_rtpRange:  " << degrees(obj.m_rtpRange) << endl;
      os << indent() << "m_subtended: " << degrees(obj.m_subtended) << endl;
      bumpIndent(-2);

      os << indent() << "}";

      return os;
    }


    //--------------------------------------------------------------------------
    // Class EigenFacet
    //--------------------------------------------------------------------------

    EigenFacet::EigenFacet(const size_t id, EigenVertexPool *pool)
        : EigenGeoObj(id, pool),
          m_color(0.0, 0.0, 0.0, 0.0),
          m_shape(POLYGON),
          m_orient(ANGLED)
    {
      DBG_POLY("id = " << m_id);
    }
    
    EigenFacet::EigenFacet(const EigenFacet &src)
        : EigenGeoObj(src)
    {
      DBG_POLY("src(id) = " << m_id);

      m_color   = src.m_color;
      m_shape   = src.m_shape;
      m_orient  = src.m_orient;
      m_plane   = src.m_plane;
      m_dim     = src.m_dim;
    }

    EigenFacet::~EigenFacet()
    {
      DBG_POLY("id = " << m_id);
    }
    
    void EigenFacet::makeTriangle(const size_t i, 
                                  const size_t j, 
                                  const size_t k, 
                                  const Orientation orient)
    {
      clear();

      addVertex(i);
      addVertex(j);
      addVertex(k);

      m_shape   = TRIANGLE;
      m_orient  = orient;

      process();
    }

    void EigenFacet::makeTriangle(const EigenVertex &v0,
                                  const EigenVertex &v1,
                                  const EigenVertex &v2,
                                  const Orientation orient)
    {
      clear();

      addVertex(v0);
      addVertex(v1);
      addVertex(v2);

      m_shape   = TRIANGLE;
      m_orient  = orient;

      process();
    }

    void EigenFacet::makeRectangle(const size_t i, const size_t j,
                                   const size_t k, const size_t l,
                                   const Orientation orient)
    {
      clear();

      addVertex(i);
      addVertex(j);
      addVertex(k);
      addVertex(l);

      m_shape   = RECTANGLE;
      m_orient  = orient;

      process();
    }

    void EigenFacet::makeRectangle(const EigenVertex &v0, const EigenVertex &v1,
                                   const EigenVertex &v2, const EigenVertex &v3,
                                   const Orientation orient)
    {
      clear();

      addVertex(v0);
      addVertex(v1);
      addVertex(v2);
      addVertex(v3);

      m_shape   = RECTANGLE;
      m_orient  = orient;

      process();
    }

    void EigenFacet::makePolygon(const IndexList   &indices,
                                 const Shape       shape,
                                 const Orientation orient)
    {
      switch( shape )
      {
        case TRIANGLE:
          assert(m_vertices.size() == 3);
          break;
        case RECTANGLE:
          assert(m_vertices.size() == 4);
          break;
        case POLYGON:
        default:
          assert(m_vertices.size() >= 3);
          break;
      }

      clear();

      addVertices(indices);

      m_shape    = shape;
      m_orient   = orient;

      process();
    }

    void EigenFacet::makePolygon(const EigenVertexList &vertices,
                                 const Shape           shape,
                                 const Orientation     orient)
    {
      switch( shape )
      {
        case TRIANGLE:
          assert(vertices.size() == 3);
          break;
        case RECTANGLE:
          assert(vertices.size() == 4);
          break;
        case POLYGON:
        default:
          assert(vertices.size() >= 3);
          break;
      }

      clear();

      addVertices(vertices);

      m_shape    = shape;
      m_orient   = orient;

      process();
    }

    void EigenFacet::process()
    {
      // closed polygons have a minimum of 3 vertices (triangle)
      assert(m_vertices.size() >= 3);
    
      m_isValid = true;

      // ranges
      EigenGeoObj::process();

      // grow a nudge more in the x-y-z range
      growBounds(PrecisionDft, m_xyzRange);
      //growBounds(PrecisionDft, m_rtpRange);

      //
      // 3 points define the polygon coplane.
      // Note that the vertex order defines the normal direction.
      //
      m_plane = EigenPlane3::Through(m_vertices.xyz[0],
                                     m_vertices.xyz[1],
                                     m_vertices.xyz[2]);

      // calculte subtended solid angle
      calcSubtended(m_subtended);

      // calculate length x width
      calcDimensions(m_dim);
    }

    void EigenFacet::calcSubtended(EigenSubtend2 &subtend)
    {
      //
      // Subtended theta related variables
      //
      double thetaMin   = m_rtpRange.m_min[_THETA];   // minimum vertex theta
      double thetaMax   = m_rtpRange.m_max[_THETA];   // maximum vertex theta
      double a0_        = m_vertices.rtp[0][_THETA];  // starting angle for sup
      double thetaMin_  = supplementary(a0_);         // supplementary minimum
      double thetaMax_  = thetaMin_;                  // supplementary maximum
      bool   xNegX      = false;    // facet does [not] cross the -x axis
      bool   gotTheta   = false;    // theta calculation is [not] complete
      double theta, theta_;         // working theta's

      //
      // Subtended phi related variables
      //
      double phiMin = m_rtpRange.m_min[_PHI];   // minimum vertex phi
      double phiMax = m_rtpRange.m_max[_PHI];   // maximum vertex phi
      double phi;                               // working phi

      EigenPoint3 pt;                           // working point
      double      x0, x1, x;                    // working x coordinates

      //
      // First check if origin falls within the facet polygon as projected onto
      // the x-y plane.
      //
      switch( m_orient )
      {
        case VERTICAL:
          break;
        case HORIZONTAL:
        case ANGLED:
        default:
          if( pipCnXY(Origin3, m_vertices.xyz) )
          {
            thetaMin  = 0.0;
            thetaMax  = M_TAU;
            gotTheta  = true;

            EigenLine3  zUp = EigenLine3::Through(Origin3, Khat3);
            double      t;

            if( intersection(zUp, t, pt) )
            {
              // facet above the origin
              if( t >= 0.0 )
              {
                phiMin = 0.0;
              }
              // facet below the origin
              else
              {
                phiMax = M_PI;
              }
            }
          }
          break;
      }

      //
      // Loop through facet vertices to find subtended angles.
      //
      for(size_t i = 1; i < m_vertices.size(); ++i)
      {
        // edge x endpoints
        x0 = m_vertices.xyz[i-1][_X];
        x1 = m_vertices.xyz[i][_X];

        // swap as necessary
        if( x0 > x1 )
        {
          double xt = x0;
          x0 = x1;
          x1 = xt;
        }

        //
        // Theta calculations.
        //
        if( !gotTheta )
        {
          theta   = m_vertices.rtp[i][_THETA];
          theta_  = supplementary(theta);

          // 
          // Adjust any new minimum, maximum supplementary thetas.
          //
          if( theta_ < thetaMin_ )
          {
            thetaMin_ = theta_;
          }
          else if( theta_ > thetaMax_ )
          {
            a0_ = theta;
            thetaMax_ = theta_;
          }

          //
          // Check for -x axis crossing. Once crossed, no need to keep checking.
          //
          if( !xNegX )
          {
            // find projected x-intercept of the facet edge
            x = x_intercept(xy(m_vertices.xyz[i-1]), xy(m_vertices.xyz[i]));

            // negative intercept
            if( !isinf(x) && (x < 0.0) )
            {
              // withing edge
              xNegX = within(x, x0, x1);
            }
          }
        }

        //
        // Phi calculations.
        //
       
        // find projection of origin onto facet edge
        pt = projection(m_vertices.xyz[i-1], m_vertices.xyz[i]);

        // the projection is within the edge endpoints
        if( within(pt.x(), x0, x1) )
        {
          pt = cartesianToSpherical(pt);

          phi = pt[_PHI];

          // 
          // Adjust any new minimum, maximum phi's.
          //
          if( phi < phiMin )
          {
            phiMin = phi;
          }
          else if( phi > phiMax )
          {
            phiMax = phi;
          }
        }
      }

      if( xNegX )
      {
        subtend.theta(a0_, thetaMax_ - thetaMin_);
      }
      else
      {
        subtend.theta(thetaMin, thetaMax - thetaMin);
      }
      subtend.phi(phiMin, phiMax - phiMin);

      DBG_POLY("facet " << m_id << ": " << m_text << ": subtended.theta: "
          << degrees(subtend.theta()));
      DBG_POLY("facet " << m_id << ": " << m_text << ": subtended.phi:   "
          << degrees(subtend.phi()));
    }

    void EigenFacet::calcDimensions(EigenPoint2 &dim)
    {
      switch( m_shape )
      {
        case TRIANGLE:
        {
          double a, b, c;
          double A;

          b = L2Dist(m_vertices.xyz[0], m_vertices.xyz[1]); // base
          c = L2Dist(m_vertices.xyz[0], m_vertices.xyz[2]); // adjacent
          a = L2Dist(m_vertices.xyz[1], m_vertices.xyz[2]); // opposite

          // law of cosines
          A = acos( (pow(b,2) + pow(c,2) - pow(a,2)) / (2 * b * c) );

          dim[_L] = b;
          dim[_W] = c * sin(A);
        }
        break;

      case RECTANGLE:
        dim[_L] = L2Dist(m_vertices.xyz[0], m_vertices.xyz[1]);
        dim[_W] = L2Dist(m_vertices.xyz[0], m_vertices.xyz[3]);
        break;

      case POLYGON:
      default:
        switch( m_orient )
        {
          case HORIZONTAL:
            dim[_L] = m_xyzRange.m_max.x() - m_xyzRange.m_min.x();
            dim[_W] = m_xyzRange.m_max.y() - m_xyzRange.m_min.y();
            break;
          case VERTICAL:
          case ANGLED:
          default:
            // TODO init
            dim[_L] = 0.0;
            dim[_W] = 0.0;
            break;
        }
        break;
      }
    }
    
    bool EigenFacet::intersection(const EigenLine3 &line,
                                  double           &t,
                                  EigenPoint3      &pt) const
    {
      assert(m_isValid);

      t = line.intersectionParameter(m_plane);

      // check if the line actually intersects the facet surface plane
      if( isnan(t) || isinf(t) )
      {
        return false;
      }
      
      // calculate the point along the ray at t (plane intersection)
      pt = line.pointAt(t);

      //
      // Check if the intersecting plane point is in polygonal facet.
      //
      switch( m_shape ) 
      {
        case RECTANGLE:
          return inCuboid(pt);  // if in bounds
        default:
          switch( m_orient ) 
          {
            case HORIZONTAL:  // optimized polygon test
              return pipCnZ(pt, m_vertices.xyz);
            case VERTICAL:    // general polygon test
            case ANGLED:
            default:
              return pipCn(pt, m_vertices.xyz);
          }
          break;
      }
    }
    
    void EigenFacet::setAttr(const Shape shape, const Orientation orient)
    {
      double  length, width, height;

      switch( shape )
      {
        case TRIANGLE:
          assert(m_vertices.size() == 3);
          m_shape = TRIANGLE;
          break;

        case RECTANGLE:
          assert(m_vertices.size() == 4);
          m_shape = RECTANGLE;
          break;

        case POLYGON:
        default:
          m_shape = POLYGON;
          break;
      }

      m_orient = orient;

      // reprocess facet
      if( m_isValid )
      {
        process();
      }
    }

    ostream &operator<<(ostream &os, const EigenFacet &obj)
    {
      os << indent() << "{" << endl;

      bumpIndent(2);

      // base class
      os << indent() << "geoobj:" << endl;
      os << (EigenGeoObj&)obj << endl;

      os << indent() << "m_color:   "; print(os, obj.m_color); os << endl;
      os << indent() << "m_shape:   " << obj.m_shape << endl;
      os << indent() << "m_orient:  " << obj.m_orient << endl;
      os << indent() << "m_plane:   "; print(os, obj.m_plane); os << endl;
      os << indent() << "m_dim:     " << obj.m_dim << endl;

      bumpIndent(-2);

      os << indent() << "}";

      return os;
    }
    
    
    //--------------------------------------------------------------------------
    // Class EigenPolyhedron
    //--------------------------------------------------------------------------
    
    EigenPolyhedron::EigenPolyhedron(const size_t id, EigenVertexPool *pool)
        : EigenGeoObj(id, pool)
    {
      DBG_POLY("id = " << m_id);

      facetCapacity(FacetsCapacityDft);
    }
    
    EigenPolyhedron::EigenPolyhedron(const EigenPolyhedron &src)
        : EigenGeoObj(src)
    {
      DBG_POLY("src(id) = " << m_id);

      m_facets    = src.m_facets;

      facetCapacity(src.m_capacity);
    }

    EigenPolyhedron::~EigenPolyhedron()
    {
      DBG_POLY("id = " << m_id);
    }
    
    void EigenPolyhedron::clear()
    {
      m_facets.clear();
      EigenGeoObj::clear();
    }

    size_t EigenPolyhedron::addFacet()
    {
      size_t  id = m_facets.size();

      m_facets.push_back(EigenFacet(id, m_pool));

      return id;
    }
    
    size_t EigenPolyhedron::addTriangleFacet(const size_t i, const size_t j,
                                             const size_t k,
                                          const EigenFacet::Orientation orient)
    {
      assert(i < m_vertices.size());
      assert(j < m_vertices.size());
      assert(k < m_vertices.size());

      size_t id = m_facets.size();

      m_facets.push_back(EigenFacet(id, m_pool));

      m_facets.back().makeTriangle(m_iindices[i], m_iindices[j], m_iindices[k],
                                   orient);

      return id;
    }
    
    size_t EigenPolyhedron::addRectangleFacet(const size_t i, const size_t j,
                                              const size_t k, const size_t l,
                                          const EigenFacet::Orientation orient)
    {
      assert(i < m_vertices.size());
      assert(j < m_vertices.size());
      assert(k < m_vertices.size());
      assert(l < m_vertices.size());

      size_t  id = m_facets.size();

      m_facets.push_back(EigenFacet(id, m_pool));

      m_facets.back().makeRectangle(m_iindices[i], m_iindices[j],
                                    m_iindices[k], m_iindices[l],
                                    orient);

      return id;
    }
    
    size_t EigenPolyhedron::addPolygonFacet(const IndexList &indices,
                                          const EigenFacet::Shape shape,
                                          const EigenFacet::Orientation orient)
    {
      size_t  n = indices.size();

      assert(n > 0);

      IndexList poolIndices(n);

      for(size_t i = 0; i < n; ++i)
      {
        assert(indices[i] < m_vertices.size());
        poolIndices.push_back(m_iindices[indices[i]]);
      }

      size_t  id = m_facets.size();

      m_facets.push_back(EigenFacet(id, m_pool));

      m_facets.back().makePolygon(poolIndices, shape, orient);

      return id;
    }
    
    size_t EigenPolyhedron::addVertexToFacet(const size_t id, const size_t i)
    {
      assert(id < m_facets.size());
      assert(i < m_vertices.size());
    
      return m_facets[id].addVertex(m_iindices[i]);
    }
    
    void EigenPolyhedron::closeFacet(const size_t id)
    {
      assert(id < m_facets.size());
    
      m_facets[id].process();
    }
    
    void EigenPolyhedron::process()
    {
      // polyhedral surfaces must have a minimum of 1 facet
      assert(m_facets.size() > 0);
    
      m_isValid = true;

      EigenGeoObj::process();

      growBounds(PrecisionDft, m_xyzRange);
      //growBounds(PrecisionDft, m_rtpRange);

      // calculte subtended solid angle
      calcSubtended(m_subtended);

      // calculate length x width x height
      // TODO dimensions(m_dim);
    }

    void EigenPolyhedron::calcSubtended(EigenSubtend2 &subtend)
    {
      EigenSubtend1 s;                  // working subtended angle
      double        d;                  // difference
      bool          gotTheta  = false;  // [not] working on theta
      bool          gotPhi    = false;  // [not] working on phi

      subtend.theta(m_facets[0].subtended().theta());
      subtend.phi(m_facets[0].subtended().phi());

      for(size_t i = 1; i < m_facets.size(); ++i)
      {
        if( !gotTheta )
        {
          // facet subtended theta
          s = m_facets[i].subtended().theta();

          // swap as necessary
          if( s.a0() < subtend.theta().a0() )
          {
            EigenSubtend1 t(s);
            s.subtend(subtend.theta());
            subtend.theta(t);
          }

          // difference of starting subtended angles 
          d = zero2tau(s.a0() - subtend.theta().a0());

          // new range
          if( d + s.omega() > subtend.theta().omega() )
          {
            subtend.theta().omega(d + s.omega());
          }

          // full range
          if( subtend.theta().omega() >= M_TAU )
          {
            subtend.theta(0.0, M_TAU);
            gotTheta = true;
          }
        }

        if( !gotPhi )
        {
          // facet subtended phi
          s = m_facets[i].subtended().phi();

          // swap as necessary
          if( s.a0() < subtend.phi().a0() )
          {
            EigenSubtend1 t(s);
            s.subtend(subtend.phi());
            subtend.phi(t);
          }

          // difference of starting subtended angles 
          d = zero2tau(s.a0() - subtend.phi().a0());

          // new range
          if( d + s.omega() > subtend.phi().omega() )
          {
            subtend.phi().omega(d + s.omega());
          }

          // full range
          if( subtend.phi().omega() >= M_PI )
          {
            subtend.phi(0.0, M_PI);
            gotPhi = true;
          }
        }
      }

      DBG_POLY("polyhedron " << m_id << ": " << m_text << ": subtended.theta: "
          << degrees(subtend.theta()));
      DBG_POLY("polyhedron " << m_id << ": " << m_text << ": subtended.phi:   "
          << degrees(subtend.phi()));
    }

    ostream &operator<<(ostream &os, const EigenPolyhedron &obj)
    {
      size_t  n = obj.m_facets.size();

      os << indent() << "{" << endl;

      bumpIndent(2);

      // base class
      os << indent() << "geoobj:" << endl;
      os << (EigenGeoObj&)obj << endl;

      os << indent() << "m_capacity: " << obj.m_capacity << endl;
      os << indent() << "m_facets: " << endl;
      os << indent() << "[" << endl;

      if( n-- > 0 )
      {
        bumpIndent(2);
        for(size_t i = 0; i < n; ++i)
        {
          os << obj.m_facets[i] << "," << endl;
        }
        os << obj.m_facets[n] << endl;
        bumpIndent(-2);
      }

      os << indent() << "]" << endl;
      bumpIndent(-2);

      os << indent() << "}";

      return os;
    }

  } // namespace gf_math

} // namespace geofrenzy
