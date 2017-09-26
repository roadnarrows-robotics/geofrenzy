////////////////////////////////////////////////////////////////////////////////
//
// Package:   Geofrenzy Robot ROS Package
//
// Link:      https://github.com/roadnarrows-robotics/geofrenzy
//
// File:      gf_poly.h
//
/*! \file
 *
 * \brief The Geofrenzy math for polygons and polyhedrons.
 *
 * For a more complete and excellent libray, see:\n
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

#ifndef _GF_POLY_H
#define _GF_POLY_H

#include <math.h>

#include <string>
#include <limits>
#include <ostream>
#include <cassert>

#include <Eigen/Eigen>
#include <Eigen/Geometry>

#include "gf_math.h"

namespace geofrenzy
{
  namespace gf_math
  {
    /*!
     * \ingroup gfmath
     * \brief Polyhedrons
     * \{
     */

    //
    // Vertex in ambient 3D space types.
    //
    typedef EigenPoint3                     EigenVertex;      ///< vertex
    typedef EigenPoint3List                 EigenVertexList;  ///< vertex list
    typedef EigenVertexList::iterator       EigenVertexIter;  ///< vertex iter
    typedef EigenVertexList::const_iterator EigenVertexCIter; ///< vertex citer


    //--------------------------------------------------------------------------
    // Class EigenFacet
    //--------------------------------------------------------------------------
 
    /*!
     * \brief The EigenFacet container class.
     *
     * Facets are planar simple, closed polygons in ambient 3D space. Simple
     * in that a facet has no hole nor self crossings. The facet can be
     * convex or concave.
     */
    class EigenFacet
    {
    public:
      /*!
       * \brief Facet orientations with respect to the x-y plane.
       */
      enum Orientation
      {
        HORIZONTAL,   ///< horizontal, parallel to the x-y plane
        VERTICAL,     ///< vertical, orthogonal to the x-y plane
        ANGLED        ///< unspecified or non-orthogonal angle to the x-y plane
      };

      /*!
       * \brief Facet shapes.
       */
      enum Shape
      {
        TRIANGLE,   ///< triangle
        RECTANGLE,  ///< rectangle
        POLYGON     ///< unspecified polygon
      };

      /*!
       * \brief Default constructor.
       *
       * \param id  Facet id.
       */
      EigenFacet(const size_t id = 0);

      /*!
       * \brief Desstructor.
       */
      virtual ~EigenFacet();

      /*!
       * \brief Clear the polyhedron of all data.
       */
      void clear();

      /*!
       * \brief Make triangular facet.
       *
       * The first two vertices form the base to calculate facet length.
       * The width is the standard triangle height calculation.
       *
       * \param v0      Base vertex 0.
       * \param v1      Base vertex 1.
       * \param v2      Third vertex of triangle.
       * \param orient  Orientation w.r.t. the x-y plane.
       */
      void makeTriangle(const EigenVertex &v0,
                        const EigenVertex &v1,
                        const EigenVertex &v2,
                        const Orientation orient = ANGLED);

      /*!
       * \brief Make rectangular facet.
       *
       * The first two verices form the base to calculate facet length.
       * The width is L2Dist(v0, v3) (or L2Dist(v1, v2)).
       *
       * \param v0      Base vertex 0.
       * \param v1      Base vertex 1.
       * \param v2      Opposing vertex to vertex 1.
       * \param v3      Opposing vertex to vertex 0.
       * \param orient  Orientation w.r.t. the x-y plane.
       */
      void makeRectangle(const EigenVertex &v0, const EigenVertex &v1,
                         const EigenVertex &v2, const EigenVertex &v3,
                         const Orientation orient = ANGLED);

      /*!
       * \brief Make general polygonal facet.
       *
       * \param vertices  List of vertices.
       * \param shape     Shape of polygon.
       * \param orient    Orientation w.r.t. the x-y plane.
       */
      void makePolygon(const EigenVertexList &vertices,
                       const Shape shape = POLYGON,
                       const Orientation orient = ANGLED);

      /*!
       * \brief Add facet vertex.
       *
       * The order of the vertices are important. It is the caller's
       * responsibility to define the ordering by sequentially calling this
       * function. The ordering defines the normal.
       *
       * \param v   Facet vertex.
       *
       * \return Returns the index of the added vertex.
       */
      size_t addVertex(const EigenVertex &v);

      /*!
       * \brief Process vertices to make a facet.
       *
       * A minimum of 3 vertices are required.
       *
       * Facet properties such  as bounds, length, widths are also calculated.
       */
      void processVertices();

      /*!
       * \brief Test if face is within view from a hypothetical ray emitting
       * from the origin at direction theta, phi.
       *
       * \param theta Azimuthal angle from x+ axis (-pi, pi].
       * \param pt    Point in polar coordinates.
       *
       * \return Returns true or false.
       */
      bool inview(const double theta, const double phi)
      {
        return false; // todo
      }

      /*!
       * \brief Test if a point is within the facet boundary.
       *
       * The boundary is a cuboid with sides parallel to the x-y-z axes planes
       * that minimally contain the facet surface.
       *
       * Being within the boundary is a necessary, but not sufficient
       * requirement for a point to be on the surface of the facet.
       *
       * Its simply a fast filter on points of interests.
       *
       * \param pt  Point to test.
       *
       * \return Returns true or false.
       */
      bool inbounds(const EigenPoint3 &pt)
      {
        return gf_math::inbounds(pt, m_bounds);
      }

      /*!
       * \brief Find the intersection between a line and this facet.
       *
       * \param       line    Parameterized line in ambient 3D space.
       * \param[out]  pt      Point of intersection.
       *
       * \return Returns true if there is an intersection, false otherwise.
       */
      bool intersection(const EigenLine3 &line, EigenPoint3 &pt);

      /*!
       * \brief Return the reference to the unit normal vector of the facet.
       *
       * \return Unit normal.
       */
      const EigenPoint3 normal() const
      {
        assert(m_isValid);
        return m_plane.normal();
      }

      /*!
       * \brief Return the circumscribed rectangle dimensions of the facet.
       *
       * The rotated sides of the rectangle are either parallel to the x or y
       * axis.
       *
       * \return Dimension point length x width.
       */
      const EigenPoint2 &dim() const
      {
        assert(m_isValid);
        return m_dim;
      }

      /*!
       * \brief Return the cuboid bounding box of the facet.
       *
       * \return Boundary.
       */
      const EigenBoundary3 &boundary() const
      {
        assert(m_isValid);
        return m_bounds;
      }

      /*!
       * \brief Return the subtended theta and phi angles of the facet.
       *
       * The subtended angels are calculate for view point of the origin.
       *
       * \return Subtended angles.
       */
      const EigenMinMax2 &subtended() const
      {
        return m_subtended;
      }

      /*!
       * \brief Return the color attribute of the facet.
       *
       * \return Color.
       */
      const EigenRGBA &color() const
      {
        return m_color;
      }

      /*!
       * \brief Return the shape enum of the facet.
       *
       * \return Shape.
       */
      const Shape shape() const
      {
        return m_shape;
      }

      /*!
       * \brief Return the orientation enum of the facet.
       *
       * \return Orientation.
       */
      const Orientation orientation() const
      {
        return m_orient;
      }

      /*!
       * \brief Set facet color attribute.
       *
       * \param color Red-green-blue-alpha color.
       */
      void setAttr(const EigenRGBA &color)
      {
        m_color = color;
      }

      /*!
       * \brief Specify facet shape and orientation attributes.
       *
       * \param shape   (Special) shape of polygon.
       * \param orient  (Special) orientation of polygon.
       */
      void setAttr(const Shape shape, const Orientation orient);

      /*!
       * \brief Returns the number of facet vertices.
       *
       * \return Number of vertices.
       */
      size_t numOfVertices() const
      {
        return m_vertices.size();
      }

      /*!
       * \brief Return the vertex at index i.
       *
       * \param i   Index.
       *
       * \return Vertex.
       */
      EigenVertex &vertexAt(size_t i)
      {
        assert(i < m_vertices.size());
        return m_vertices[i];
      }

      /*!
       * \brief Return the vertex at index i.
       *
       * \param i   Index.
       *
       * \return Constant Vertex.
       */
      const EigenVertex &vertexAt(size_t i) const
      {
        assert(i < m_vertices.size());
        return m_vertices[i];
      }

      /*!
       * \brief Return an iterator pointing to the first element of the list
       * of facet vertices.
       *
       * \return Iterator.
       */
      EigenVertexIter iterVertexBegin()
      {
        return m_vertices.begin();
      }

      /*!
       * \brief Return an iterator referring to the past-the-end element of
       * the list of facet vertices.
       *
       * \return Iterator.
       */
      EigenVertexIter iterVertexEnd()
      {
        return m_vertices.end();
      }

      /*!
       * \brief Return a constant iterator pointing to the first element of
       * the list of facet vertices.
       *
       * \return Constant Iterator.
       */
      EigenVertexCIter iterVertexBegin() const
      {
        return m_vertices.begin();
      }

      /*!
       * \brief Return a constant iterator referring to the past-the-end
       * element of the list of facet vertices.
       *
       * \return Constant Iterator.
       */
      EigenVertexCIter iterVertexEnd() const
      {
        return m_vertices.end();
      }

    protected:
      //
      // The Facet
      //
      size_t            m_id;         ///< facet id
      EigenVertexList   m_vertices;   ///< ordered facet vertices
      bool              m_isValid;    ///< facet is [not] valid and processed

      //
      // Attributes
      //
      EigenRGBA         m_color;      ///< color of facet
      Shape             m_shape;      ///< (special) shape
      Orientation       m_orient;     ///< (special) orientation

      //
      // Properties
      //
      EigenPlane3       m_plane;      ///< infinite coplanar surface to facet
      EigenPoint2       m_dim;        ///< length x width dimension
      EigenBoundary3    m_bounds;     ///< cuboid bounds of facet
      EigenMinMax2      m_subtended;  ///< subtended angles

    }; // class EigenFacet

    //
    // Polygonal facet list types.
    //
    typedef std::vector<EigenFacet>         EigenFacetList;   ///< facet list
    typedef EigenFacetList::iterator        EigenFacetIter;   ///< facet iter
    typedef EigenFacetList::const_iterator  EigenFacetCIter;  ///< facet citer


    //--------------------------------------------------------------------------
    // Class EigenPolyhedron
    //--------------------------------------------------------------------------

    //
    // \brief The EigenPolyhedral container class.
    //
    // Polyhedral surfaces in ambient 3D space are composed of vertices,
    // edges, facets and an orientation relationship on them. If the facets
    // form a closed interior, then the polyhedral surface is a polyhedron.
    //
    // By convention, the vertices  are oriented counterclockwise
    // around facets as seen from the outside of the polyhedron. That is,
    // the normals point outward.
    //
    class EigenPolyhedron
    {
    public:
      /*!
       * \brief Default constructor.
       */
      EigenPolyhedron();

      /*!
       * \brief Destructor.
       */
      virtual ~EigenPolyhedron();

      /*!
       * \brief Clear the polyhedron of all data.
       */
      void clear();

      /*!
       * \brief Add polyhedron vertex.
       *
       * The vertices are an unordered list. It is the facets the provide
       * orientation order.
       *
       * \param v   Vertex.
       *
       * \return Returns the index of the added vertex.
       */
      size_t addVertex(const EigenVertex &v);

      /*!
       * \brief Add a polygonal polyhedron facet.
       *
       * The new facet has no assigned vertices.
       *
       * Facet Building:
       * \code{.cpp}
       * size_t id = addFacet();  // create new empty facet
       *
       * addVertexToFacet(id, i); // add polyhedron vertex i to facet
       * addVertexToFacet(id, j); // add polyhedron vertex j to facet
       * addVertexToFacet(id, k); // add polyhedron vertex k to facet
       * // add other vertices here
       *
       * closeFacet(id);          // mark the facet closed with props calc'd
       \endcode

       * \return Returns the id of the added facet.
       */
      size_t addFacet();

      /*!
       * \brief Make triangular facet.
       *
       * The first two vertices form the base to calculate facet length.
       * The width is the standard triangle height calculation.
       *
       * Vertices must be already added to polyhedron.
       *
       * \param i       Base vertex index 0.
       * \param j       Base vertex index 1.
       * \param k       Third vertex index of triangle.
       * \param orient  Orientation w.r.t. the x-y plane.
       */
      size_t addTriangleFacet(const size_t i, const size_t j, const size_t k,
                        EigenFacet::Orientation orient = EigenFacet::ANGLED);

      /*!
       * \brief Make rectangular facet.
       *
       * The first two vertices form the base to calculate facet length.
       * The width is the standard triangle height calculation.
       *
       * Vertices must be already added to polyhedron.
       *
       * \param i       Base vertex index 0.
       * \param j       Base vertex index 1.
       * \param k       Opposing vertex index to vertex index 1.
       * \param l       Opposing vertex index to vertex index 0.
       * \param orient  Orientation w.r.t. the x-y plane.
       */
      size_t addRectangleFacet(const size_t i, const size_t j,
                               const size_t k, const size_t l,
                        EigenFacet::Orientation orient = EigenFacet::ANGLED);

      /*!
       * \brief Make general polygonal facet.
       *
       * \param vertices  List of vertex indices.
       * \param shape     Shape of polygon.
       * \param orient    Orientation w.r.t. the x-y plane.
       */
      size_t addPolygonFacet(std::vector<size_t> indices,
                        EigenFacet::Shape shape = EigenFacet::POLYGON,
                        EigenFacet::Orientation orient = EigenFacet::ANGLED);

      /*!
       * \brief Add a vertex to the identified facet.
       *
       * Orientation order here is define by the sequence of calls to this
       * function.
       *
       * Vertices must be already added to polyhedron.
       *
       * \param id  The facet identifier.
       * \param i   Index of the vertex.
       *
       * \return Returns the index of the added facet vertex.
       */
      size_t addVertexToFacet(const size_t id, const size_t i);

      /*!
       * \brief Close the facet.
       *
       * This call is required after adding all of the vertices to the facet.
       * Facet validation and post-processing with then occur.
       *
       * \param id  The facet identifier.
       */
      void closeFacet(const size_t id);

      /*!
       * \brief Test if point is within the polyhedron boundary.
       *
       * The boundary is a cuboid with sides parallel to the x-y-z axes planes.
       * that minimally contain the polyhedron.
       *
       * Being within the boundary is a necessary, but not sufficient
       * requirement for a point to be on or within the polyhedron.
       *
       * Its simply a fast filter on points of interests.
       *
       * \param pt      Point to test.
       *
       * \return Returns true or false.
       */
      bool inbounds(const EigenPoint3 &pt)
      {
        return gf_math::inbounds(pt, m_bounds);
      }

      /*!
       * \brief Return the cuboid bounding box of the polyhedron.
       *
       * \return Boundary.
       */
      const EigenBoundary3 &boundary() const
      {
        return m_bounds;
      }

      /*!
       * \brief Returns the number of polyhedron vertices.
       *
       * \return Number of vertices.
       */
      size_t numOfVertices() const
      {
        return m_vertices.size();
      }

      /*!
       * \brief Return the vertex at index i.
       *
       * \param i   Index.
       *
       * \return Vertex.
       */
      EigenVertex &vertexAt(size_t i)
      {
        assert(i < m_vertices.size());
        return m_vertices[i];
      }

      /*!
       * \brief Return the vertex at index i.
       *
       * \param i   Index.
       *
       * \return Constant Vertex.
       */
      const EigenVertex &vertexAt(size_t i) const
      {
        assert(i < m_vertices.size());
        return m_vertices[i];
      }

      
      /*!
       * \brief Return an iterator pointing to the first element of the list
       * of polyhedron vertices.
       *
       * \return Iterator.
       */
      EigenVertexIter iterVertexBegin()
      {
        return m_vertices.begin();
      }

      /*!
       * \brief Return an iterator referring to the past-the-end element of
       * the list of polyhedron vertices.
       *
       * \return Iterator.
       */
      EigenVertexIter iterVertexEnd()
      {
        return m_vertices.end();
      }

      /*!
       * \brief Return a constant iterator pointing to the first element of
       * the list of polyhedron vertices.
       *
       * \return Constant Iterator.
       */
      EigenVertexCIter iterVertexBegin() const
      {
        return m_vertices.begin();
      }

      /*!
       * \brief Return a constant iterator referring to the past-the-end
       * element of the list of polyhedron vertices.
       *
       * \return Constant Iterator.
       */
      EigenVertexCIter iterVertexEnd() const
      {
        return m_vertices.end();
      }

      /*!
       * \brief Return the number of polyhedron facets.
       *
       * \return Number of facets.
       */
      size_t numOfFacets() const
      {
        return m_facets.size();
      }

      /*!
       * \brief Return the facet with associated id.
       *
       * \param id  The facet identifier.
       *
       * \return Facet.
       */
      EigenFacet &facetAt(size_t id)
      {
        assert(id < m_facets.size());
        return m_facets[id];
      }

      /*!
       * \brief Return the facet with associated id.
       *
       * \param id  The facet identifier.
       *
       * \return Constant Facet.
       */
      const EigenFacet &facetAt(size_t id) const
      {
        assert(id < m_facets.size());
        return m_facets[id];
      }

      /*!
       * \brief Return an iterator pointing to the first element of the list
       * of polyhedron facets.
       *
       * \return Iterator.
       */
      EigenFacetIter iterFacetBegin()
      {
        return m_facets.begin();
      }

      /*!
       * \brief Return an iterator referring to the past-the-end element of
       * the list of polyhedron facets.
       *
       * \return Iterator.
       */
      EigenFacetIter iterFacetEnd()
      {
        return m_facets.end();
      }

      /*!
       * \brief Return a constant iterator pointing to the first element of
       * the list of polyhedron facets.
       *
       * \return Constant Iterator.
       */
      EigenFacetCIter iterFacetBegin() const
      {
        return m_facets.begin();
      }

      /*!
       * \brief Return a constant iterator referring to the past-the-end
       * element of the list of polyhedron facets.
       *
       * \return Constant Iterator.
       */
      EigenFacetCIter iterFacetEnd() const
      {
        return m_facets.end();
      }

    protected:
      EigenVertexList   m_vertices;   ///< polyhedral vertices
      EigenFacetList    m_facets;
      EigenBoundary3    m_bounds;

    }; // class EigenPolyhedron

    /*! \} */ // end of gfmath doxy group

  } // namespace gf_math

} // namespace geofrenzy


#endif // _GF_POLY_H
