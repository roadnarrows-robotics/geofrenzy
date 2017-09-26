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
// Basic Eigen Operators and Geometry Functions
//------------------------------------------------------------------------------
namespace geofrenzy
{
  namespace gf_math
  {
    //--------------------------------------------------------------------------
    // Class EigenFacet
    //--------------------------------------------------------------------------

    EigenFacet::EigenFacet(const size_t id)
        : m_id(id),
          m_color(0.0, 0.0, 0.0, 0.0),
          m_shape(POLYGON),
          m_orient(ANGLED)
    {
      m_isValid = false;
    }
    
    EigenFacet::~EigenFacet()
    {
    }
    
    void EigenFacet::clear()
    {
      m_vertices.clear();
      m_isValid = false;
    }
  
    void EigenFacet::makeTriangle(const EigenVertex &v0,
                                  const EigenVertex &v1,
                                  const EigenVertex &v2,
                                  const Orientation orient)
    {
      m_isValid = false;

      m_vertices.clear();

      m_vertices.push_back(v0); m_vertices.push_back(v1);
      m_vertices.push_back(v2);

      m_shape   = TRIANGLE;
      m_orient  = orient;

      processVertices();
    }

    void EigenFacet::makeRectangle(const EigenVertex &v0, const EigenVertex &v1,
                                   const EigenVertex &v2, const EigenVertex &v3,
                                   const Orientation orient)
    {
      m_isValid = false;

      m_vertices.clear();

      m_vertices.push_back(v0); m_vertices.push_back(v1);
      m_vertices.push_back(v2); m_vertices.push_back(v3);

      m_shape   = RECTANGLE;
      m_orient  = orient;

      processVertices();
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

      m_isValid = false;

      m_vertices.clear();

      m_vertices = vertices;
      m_shape    = shape;
      m_orient   = orient;

      processVertices();
    }

    size_t EigenFacet::addVertex(const EigenVertex &v)
    {
      // RDK do some vertex checking

      m_vertices.push_back(v);

      m_isValid = false;

      return m_vertices.size() - 1;
    }
    
    void EigenFacet::processVertices()
    {
      // closed polygons have a minimum of 3 vertices (triangle)
      assert(m_vertices.size() >= 3);
    
      //
      // 3 points define the polygon coplane.
      // Note that the vertex order defines the normal direction.
      //
      m_plane = EigenPlane3::Through(m_vertices[0],
                                     m_vertices[1],
                                     m_vertices[2]);

      seedBounds(m_vertices[0], m_bounds);

      for(size_t i = 1; i < m_vertices.size(); ++i)
      {
        growBounds(m_vertices[i], m_bounds);
      }

      growBounds(PrecisionDft, m_bounds);

      switch( m_shape )
      {
        case TRIANGLE:
          {
            double a, b, c;
            double A;

            b = L2Dist(m_vertices[0], m_vertices[1]); // base
            c = L2Dist(m_vertices[0], m_vertices[2]); // adjacent
            a = L2Dist(m_vertices[1], m_vertices[2]); // opposite

            // law of cosines
            A = acos( (pow(b,2) + pow(c,2) - pow(a,2)) / (2 * b * c) );

            m_dim[_L] = b;
            m_dim[_W] = c * sin(A);
          }
          break;

        case RECTANGLE:
          m_dim[_L] = L2Dist(m_vertices[0], m_vertices[1]);
          m_dim[_W] = L2Dist(m_vertices[0], m_vertices[3]);
          break;

        case POLYGON: // lazy init
        default:
          m_dim[_L] = 0.0;
          m_dim[_W] = 0.0;
          break;
      }

      m_isValid = true;
    }
    
    bool EigenFacet::intersection(const EigenLine3 &line, EigenPoint3 &pt)
    {
      assert(m_isValid);
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
    }

    
    //--------------------------------------------------------------------------
    // Class EigenFacet
    //--------------------------------------------------------------------------
    
    EigenPolyhedron::EigenPolyhedron()
    {
    }
    
    EigenPolyhedron::~EigenPolyhedron()
    {
    }
    
    void EigenPolyhedron::clear()
    {
      m_vertices.clear();
      m_facets.clear();
    }

    size_t EigenPolyhedron::addVertex(const EigenVertex &v)
    {
      m_vertices.push_back(v);

      if( m_vertices.size() == 1 )
      {
        seedBounds(v, m_bounds);
      }

      else
      {
        growBounds(v, m_bounds);
      }

      return m_vertices.size() - 1;
    }
    
    size_t EigenPolyhedron::addFacet()
    {
      size_t  id = m_facets.size();

      EigenFacet f(id);

      m_facets.push_back(f);

      return id;
    }
    
    size_t EigenPolyhedron::addTriangleFacet(const size_t i, const size_t j,
                                             const size_t k,
                                          const EigenFacet::Orientation orient)
    {
      assert(i < m_vertices.size());
      assert(j < m_vertices.size());
      assert(k < m_vertices.size());

      size_t  id = m_facets.size();

      EigenFacet f(id);

      m_facets.push_back(f);

      m_facets.back().makeTriangle(m_vertices[i], m_vertices[j], m_vertices[k],
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

      EigenFacet f(id);

      m_facets.push_back(f);

      m_facets.back().makeRectangle(m_vertices[i], m_vertices[j],
                                    m_vertices[k], m_vertices[l],
                                    orient);

      return id;
    }
    
    size_t EigenPolyhedron::addPolygonFacet(vector<size_t> indices,
                                          const EigenFacet::Shape shape,
                                          const EigenFacet::Orientation orient)
    {
      EigenVertexList vertices;

      for(size_t i = 0; i < indices.size(); ++i)
      {
        assert(indices[i] < m_vertices.size());
        vertices.push_back(m_vertices[indices[i]]);
      }

      size_t  id = m_facets.size();

      EigenFacet f(id);

      m_facets.push_back(f);

      m_facets.back().makePolygon(vertices, shape, orient);

      return id;
    }
    
    size_t EigenPolyhedron::addVertexToFacet(const size_t id, const size_t i)
    {
      assert(id < m_facets.size());
      assert(i < m_vertices.size());
    
      return m_facets[id].addVertex(m_vertices[i]);
    }
    
    void EigenPolyhedron::closeFacet(const size_t id)
    {
      assert(id < m_facets.size());
    
      m_facets[id].processVertices();
    }
    
    #if 0 // RDK
    void EigenPolyhedron::x()
    {
      //m_spherical.push_back(cartesianToSpherical(v));
    }
    #endif // RDK

  } // namespace gf_math

} // namespace geofrenzy
