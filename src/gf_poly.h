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

#define NEWYEW

//
// Debug macro
//
#undef DBG_POLY_ENABLE   ///< define or undef
#ifdef DBG_POLY_ENABLE
#define DBG_POLY(_obj)  \
  std::cerr << "DBG: " << __func__ << "() " << _obj << std::endl
#else
#define DBG_POLY(_obj)
#endif // DBG_POLY_ENABLE


namespace geofrenzy
{
  namespace gf_math
  {
    /*!
     * \ingroup gfmath
     * \brief Of polygons and polyhedrons.
     * \{
     */

    //--------------------------------------------------------------------------
    // Basic Vertex Types
    //--------------------------------------------------------------------------

    //
    // Vertices
    //
    typedef EigenPoint3                     EigenVertex;      ///< vertex
    typedef std::vector<EigenVertex>        EigenVertexList;  ///< vertex list
    typedef EigenVertexList::iterator       EigenVertexIter;  ///< vertex iter
    typedef EigenVertexList::const_iterator EigenVertexCIter; ///< vertex citer

    //--------------------------------------------------------------------------
    // Extended Vertex Structures
    //--------------------------------------------------------------------------

    /*!
     * \brief Extend vertex structure.
     */
    struct EigenExVertex
    {
      EigenVertex xyz;  ///< vertex in Cartesian x,y,z coordinates
      EigenVertex rtp;  ///< equivalent in spherical rho,theta,phi coordinates
    }; // struct EigenExVertex

    /*!
     * \brief Extended vertex list structure.
     *
     * Vertices are specified in both Cartesion and spherical coordinates. The
     * push_back methods keep the vertices in sync.
     *
     * \note The extend vertex list is actual two parallel lists. This
     * arrangement is for optimizing calls to geometric algoritms.
     */
    struct EigenExVertexList
    {
      EigenVertexList xyz;  ///< vertices in Cartesian x,y,z coordinates
      EigenVertexList rtp;  ///< equivalent in spherical rho,theta,phi coord.

      /*!
       * \brief Clear vertices.
       */
      void clear()
      {
        xyz.clear();
        rtp.clear();
      }

      /*!
       * \brief Assignment operator.
       *
       * \param rhs   Right-hand side object.
       *
       * \return *this
       */
      EigenExVertexList &operator=(const EigenExVertexList &rhs)
      {
        xyz = rhs.xyz;
        rtp = rhs.rtp;
      }

      /*!
       * \brief Return the size (number of vertices) of extended vertex list.
       *
       * \return Size.
       */
      size_t size() const
      {
        return xyz.size();
      }

      /*!
       * \brief Return the current capacity of the vertex list.
       *
       * \return Current capacity.
       */
      size_t capacity() const
      {
        return xyz.capacity();
      }

      /*!
       * \brief Set the reserve capacity of the vertex list.
       *
       * \param n   New capacity in number of elements.
       *
       * \return New capacity.
       */
      size_t capacity(size_t n)
      {
        if( capacity() < n )
        {
          xyz.reserve(n);
          rtp.reserve(n);
        }
        return capacity();
      }

      /*!
       * \brief Array operator.
       *
       * \param i   Index into array.
       *
       * \return Constant to the i'th element.
       */
      const EigenExVertex operator[](const size_t i) const
      {
        assert(i < size());

        EigenExVertex v;

        v.xyz = xyz[i];
        v.rtp = rtp[i];

        return v;
      }

      /*!
       * \brief Add a new vertex to the end of the list.
       *
       * \param vxyz    Vertex in Cartesian coordinates.
       * \param vrtp    Vertex in equivalent spherical coordinates.
       */
      void push_back(const EigenVertex &vxyz, const EigenVertex &vrtp)
      {
        xyz.push_back(vxyz);
        rtp.push_back(vrtp);
      }

      /*!
       * \brief Add a new vertex to end of the list.
       *
       * The vertex is specified in either Cartesian or spherical coordinates.
       * The equivalent spherical (Cartesian) coordinate is automatically
       * calculated and added.
       *
       * \param v     Vertex to add.
       * \param coord Coordinate system.
       */
      void push_back(const EigenVertex &v, const Coordinates coord = Cartesian);

      /*!
       * \brief Adds a new Cartesian vertex to end of the list.
       *
       * The equivalent spherical coordinate is automatically calculated and
       * added.
       *
       * \param v Vertex in Cartesian x,y,z coordinates.
       */
      void push_back_xyz(const EigenVertex &v)
      {
        xyz.push_back(v);
        rtp.push_back(cartesianToSpherical(v));
      }

      /*!
       * \brief Adds a new spherical vertex to end of the list.
       *
       * The equivalent Cartesian coordinate is automatically calculated and
       * added.
       *
       * \param v Vertex in spherical rho,theta,phi coordinates.
       */
      void push_back_rtp(const EigenVertex &v)
      {
        rtp.push_back(v);
        xyz.push_back(sphericalToCartesian(v));
      }

      /*!
       * \brief Stream insertion operator.
       *
       * \param os  Output stream.
       * \param obj Object to insert.
       *
       * \return Reference to output stream.
       */
      friend std::ostream &operator<<(std::ostream &os,
                                      const EigenExVertexList &obj);

    }; // struct EigenExVertexList


    //--------------------------------------------------------------------------
    // Class EigenVertexPool
    //--------------------------------------------------------------------------

    /*!
     * \brief Pool of vertices class.
     *
     * Many geometric objects share a subset of vertices, such as the edges
     * of a facet and the facets of a polyhedron.
     *
     * The pool provides fast access to shared vertices, while minimizing
     * data copies and algebraic operations.
     */
    class EigenVertexPool
    {
    public:
      /*! default pool reserved capacity */
      static const size_t  PoolCapacityDft = 16;

      /*!
       * \brief Default constructor.
       *
       * \param id   Pool identifier.
       */
      EigenVertexPool(size_t id = 0);

      /*!
       * \brief Copy constructor.
       *
       * \param src   Source object.
       */
      EigenVertexPool(const EigenVertexPool &src);

      /*!
       * \brief Destructor.
       */
      virtual ~EigenVertexPool();

      /*!
       * \brief Clear pool.
       */
      void clear()
      {
        m_vertices.clear();
      }

      /*!
       * \brief Bind calling object to this pool.
       */
      void bind()
      {
        ++m_refcnt;
      }

      /*!
       * \brief Unbind calling object from this pool.
       */
      void unbind()
      {
        if( m_refcnt > 0 )
        {
          --m_refcnt;
        }
      }

      /*!
       * \brief Return the pool id.
       *
       * \return Id.
       */
      size_t id() const
      {
        return m_id;
      }

      /*!
       * \brief Return the pool reference count.
       *
       * \return Count.
       */
      size_t refcnt() const
      {
        return m_refcnt;
      }

      /*!
       * \brief Size (number of vertices) of pool.
       */
      size_t size() const
      {
        return m_vertices.size();
      }

      /*!
       * \brief Return the current capacity of the pool vertex list.
       *
       * \return Current capacity.
       */
      size_t capacity() const
      {
        return m_vertices.capacity();
      }

      /*!
       * \brief Set the reserve capacity of the pool vertex list.
       *
       * \param n   New capacity in number of elements.
       *
       * \return New capacity.
       */
      size_t capacity(size_t n)
      {
        m_capacity = n;
        return m_vertices.capacity(m_capacity);
      }

      /*!
       * \brief Add an extended vertex to the pool.
       *
       * \param v   Extended vertex to add.
       *
       * \return Returns the index of the added vertex.
       */
      size_t addVertex(const EigenExVertex &v)
      {
        m_vertices.push_back(v.xyz, v.rtp);
        return m_vertices.size() - 1;
      }

      /*!
       * \brief Add a vertex to the pool.
       *
       * The vertex is specified in x,y,z Cartesian coordinates.
       * The equivalent spherical coordinates are automatically calculated.
       *
       * \param v   Vertex to add.
       *
       * \return Returns the index of the added vertex.
       */
      size_t addVertex(const EigenVertex &v)
      {
        m_vertices.push_back_xyz(v);
        return m_vertices.size() - 1;
      }

      /*!
       * \brief Add a vertex to the pool.
       *
       * The vertex is specified in rho,theta,phi spherical coordinates.
       * The equivalent Cartesian coordinates are automatically calculated.
       *
       * \param v   Vertex to add.
       *
       * \return Returns the index of the added vertex.
       */
      size_t addSpherical(const EigenVertex &v)
      {
        m_vertices.push_back_rtp(v);
        return m_vertices.size() - 1;
      }

      /*!
       * \brief Return list of extended vertices.
       *
       * \return Reference to vertices.
       */
      const EigenExVertexList &vertices() const
      {
        return m_vertices;
      }

      /*!
       * \brief Return list of vertices in Cartesian coordinates.
       *
       * \return Reference to vertices.
       */
      const EigenVertexList &cartesians() const
      {
        return m_vertices.xyz;
      }

      /*!
       * \brief Return list of vertices in spherical coordinates.
       *
       * \return Reference to vertices.
       */
      const EigenVertexList &sphericals() const
      {
        return m_vertices.rtp;
      }

      /*!
       * \brief Returns the number of vertices.
       * 
       * Same as size().
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
       *
       * \param i   Pool vertex index.
       *
       * \return Vertex in Cartesian coordinates.
       */
      EigenVertex &vertexAt(size_t i)
      {
        assert(i < m_vertices.size());
        return m_vertices.xyz[i];
      }

      /*!
       * \brief Return the vertex at index i.
       *
       * \param i   Pool vertex index.
       *
       * \return Constant vertex in Cartesian coordinates.
       */
      const EigenVertex &vertexAt(size_t i) const
      {
        assert(i < m_vertices.size());
        return m_vertices.xyz[i];
      }

      /*!
       * \brief Return the vertex at index i.
       *
       * \param i   Pool vertex index.
       *
       * \return Vertex in spherical coordinates.
       */
      EigenVertex &sphericalAt(size_t i)
      {
        assert(i < m_vertices.rtp.size());
        return m_vertices.rtp[i];
      }

      /*!
       * \brief Return the vertex at index i.
       *
       * \param i   Pool vertex index.
       *
       * \return Constant vertex in spherical coordinates.
       */
      const EigenVertex &sphericalAt(size_t i) const
      {
        assert(i < m_vertices.rtp.size());
        return m_vertices.rtp[i];
      }

      /*!
       * \brief Stream insertion operator.
       *
       * \param os  Output stream.
       * \param obj Object to insert.
       *
       * \return Reference to output stream.
       */
      friend std::ostream &operator<<(std::ostream &os,
                                      const EigenVertexPool &obj);

    protected:
      size_t            m_id;       ///< pool id
      size_t            m_refcnt;   ///< pool reference count
      size_t            m_capacity; ///< pool reserved capacity
      EigenExVertexList m_vertices; ///< pool of vertices

    }; // class EigenVertexPool


    //--------------------------------------------------------------------------
    // Class EigenGeoObj
    //--------------------------------------------------------------------------

    /*!
     * Vertex indices.
     *
     * Direct indices apply to both the pool or the object's vertices,
     * depending on context.
     *
     * Indirect indices map from the geometric object's vertex indices to the
     * vertex pool indices.
     */
    typedef std::vector<size_t>       IndexList;  ///< indices list
    typedef std::map<size_t, size_t>  IIndexMap;  ///< indirect indices map

    /*!
     * \brief Geometric object base class.
     *
     * Any Eigen geometric object with vertices should derive from this class.
     */
    class EigenGeoObj
    {
    public:
      /*! default reserved capacity of object's list of vertices */
      static const size_t VerticesCapacityDft  = 16;

      /*!
       * \brief Default constructor.
       *
       * \param id    Facet id.
       * \param pool  Pointer to vertex pool.
       */
      EigenGeoObj(const size_t id = 0, EigenVertexPool *pool = NULL);

      /*!
       * \brief Copy constructor.
       *
       * \param src Source geometric object.
       */
      EigenGeoObj(const EigenGeoObj &src);

      /*!
       * \brief Destructor.
       */
      virtual ~EigenGeoObj();

      /*!
       * \brief Clear the object's vertices and properties.
       */
      virtual void clear();

      ///@{
      /*!
       * \brief Family of methods to add a vertex or list of vertices to the
       * geometric object.
       *
       * For methods addVertex(i) and addVertices(indices), the indices specify
       * vertices residing in the vertex pool.
       *
       * For methods addVertex(v) and addVertices(vertices), the vertices
       * are specified in x,y,z Cartesian coordinates. Any bound vertex pool is 
       * updated automatically.
       *
       * The order of the vertices is important. It is the caller's
       * responsibility to define the ordering by sequentially calling these
       * function. The ordering defines properties such as normals.
       *
       * Adding vertices to a geometric object invalidates the object until it
       * is processed. See process().
       *
       * \param i         Vertex pool index.
       * \param indices   Vertex pool list of indices.
       * \param v         Facet vertex in x,y,z coordinates.
       * \param vertices  Facet list of vertices in x,y,z coordinates.
       *
       * \return
       * Returns the index or list of indices of the added facet vertex or
       * facet vertices.
       */
      virtual size_t addVertex(const size_t i)
      {
        m_isValid = false;
        return copyFromPool(i);
      }

      virtual IndexList addVertices(const IndexList &indices);

      virtual size_t addVertex(const EigenVertex &v)
      {
        m_isValid = false;
        return writeThrough(v);
      }

      virtual IndexList addVertices(const EigenVertexList &vertices);
      ///@}

      /*!
       * \brief Process object to determine basic geometric properties.
       *
       * Basic properties do not require specific geometric shape information.
       * Currently, only xyz and rho,theta,phi min,max ranges are calculated.
       */
      virtual void process();

      /*!
       * \brief Test if the object is within view of a hypothetical ray
       * emitting from the origin at direction theta, phi.
       *
       * \param theta Azimuthal angle from x+ axis (-pi, pi].
       * \param phi   Polar angle from z+ [0, pi].
       *
       * \return Returns true or false.
       */
      virtual bool inview(const double theta, const double phi) const
      {
        assert(m_isValid);
        return m_subtended.within(theta, phi);
      }

      /*!
       * \brief Test if a point is within the object's cuboid box.
       *
       * The cuboid box is a boundary that has sides parallel to the x-y-z axes
       * planes that minimally contain the object.
       *
       * Being within the boundary is a necessary, but not sufficient
       * requirement for a point to be on or within the object.
       *
       * It's simply a fast filter on points of interests.
       *
       * \param pt  Cartesion point x,y,z to test.
       *
       * \return Returns true or false.
       */
      virtual bool inCuboid(const EigenPoint3 &pt) const
      {
        assert(m_isValid);
        return gf_math::inbounds(pt, m_xyzRange);
      }

      /*!
       * \brief Test if a point is within the object's spherical cap.
       *
       * The spherical cap is a boundary that is is defined by the minimum and
       * maximum radius, theta, and phi.
       *
       * \param pt  Spherical point rho,theta,phi to test.
       *
       * \return Returns true or false.
       */
      virtual bool inSphericalCap(const EigenPoint3 &pt) const
      {
        assert(m_isValid);
        return gf_math::inbounds(pt, m_rtpRange);
      }

      /*!
       * \brief Return the object's id.
       *
       * \return Id.
       */
      size_t id() const
      {
        return m_id;
      }

      /*!
       * \brief Return the descriptive text.
       *
       * \return Text string reference.
       */
      const std::string &text() const
      {
        return m_text;
      }

      /*!
       * \brief Set the descriptive text.
       *
       * \param text  Text string.
       */
      void text(const std::string &text)
      {
        m_text = text;
      }

      /*!
       * \brief Return the size (number of vertices) of the object.
       *
       * \return Size.
       */
      size_t size() const
      {
        return m_vertices.size();
      }

      /*!
       * \brief Return the current capacity of the object's vertex list.
       *
       * \return Current capacity.
       */
      size_t capacity() const
      {
        return m_vertices.capacity();
      }

      /*!
       * \brief Set the reserve capacity of the object's vertex list.
       *
       * \param n   New capacity in number of elements.
       *
       * \return New capacity.
       */
      size_t capacity(size_t n)
      {
        m_capacity = n;
        return m_vertices.capacity(m_capacity);
      }

      /*!
       * \brief Return the x,y,z ranges of the object.
       *
       * \return Min,max ranges.
       */
      const EigenBoundary3 &xyzRange() const
      {
        return m_xyzRange;
      }

      /*!
       * \brief Return the rho,theta,phi ranges of the object.
       *
       * \return Min,max ranges.
       */
      const EigenBoundary3 &rtpRange() const
      {
        return m_rtpRange;
      }

      /*!
       * \brief Return the subtended theta and phi angle bounds of the object.
       *
       * The subtended angels are calculated for the view point from the origin.
       *
       * \return Object's theta and phi start,subtended angles.
       */
      const EigenSubtend2 &subtended() const
      {
        return m_subtended;
      }

      /*!
       * \brief Return list of extended vertices.
       *
       * \return Reference to vertices.
       */
      const EigenExVertexList &vertices() const
      {
        return m_vertices;
      }

      /*!
       * \brief Return list of vertices in Cartesian coordinates.
       *
       * \return Reference to vertices.
       */
      const EigenVertexList &cartesians() const
      {
        return m_vertices.xyz;
      }

      /*!
       * \brief Return list of vertices in spherical coordinates.
       *
       * \return Reference to vertices.
       */
      const EigenVertexList &sphericals() const
      {
        return m_vertices.rtp;
      }

      /*!
       * \brief Returns the number of vertices.
       * 
       * Same as size().
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
       * \param i   Object vertex index.
       *
       * \return Vertex in Cartesian coordinates.
       */
      EigenVertex &vertexAt(size_t i)
      {
        assert(i < m_vertices.size());
        return m_vertices.xyz[i];
      }

      /*!
       * \brief Return the vertex at index i.
       *
       * \param i   Object vertex index.
       *
       * \return Constant vertex in Cartesian coordinates.
       */
      const EigenVertex &vertexAt(size_t i) const
      {
        assert(i < m_vertices.size());
        return m_vertices.xyz[i];
      }

      /*!
       * \brief Return the vertex at index i.
       *
       * \param i   Object vertex index.
       *
       * \return Vertex in spherical coordinates.
       */
      EigenVertex &sphericalAt(size_t i)
      {
        assert(i < m_vertices.rtp.size());
        return m_vertices.rtp[i];
      }

      /*!
       * \brief Return the vertex at index i.
       *
       * \param i   Object vertex index.
       *
       * \return Constant vertex in spherical coordinates.
       */
      const EigenVertex &sphericalAt(size_t i) const
      {
        assert(i < m_vertices.rtp.size());
        return m_vertices.rtp[i];
      }

      /*!
       * \brief Return an iterator pointing to the first element of the list
       * of Cartesian vertices of the object.
       *
       * \return Iterator.
       */
      EigenVertexIter iterVertexBegin()
      {
        return m_vertices.xyz.begin();
      }

      /*!
       * \brief Return an iterator referring to the past-the-end element of
       * the list of Cartesian vertices of the object.
       *
       * \return Iterator.
       */
      EigenVertexIter iterVertexEnd()
      {
        return m_vertices.xyz.end();
      }

      /*!
       * \brief Return a constant iterator pointing to the first element of
       * the list of Cartesian vertices of the object.
       *
       * \return Constant Iterator.
       */
      EigenVertexCIter iterVertexBegin() const
      {
        return m_vertices.xyz.begin();
      }

      /*!
       * \brief Return a constant iterator referring to the past-the-end
       * element of the list of Cartesian vertices of the object.
       *
       * \return Constant Iterator.
       */
      EigenVertexCIter iterVertexEnd() const
      {
        return m_vertices.xyz.end();
      }

      /*!
       * \brief Make a vertex pool for this geometric object.
       */
      void makePool();

      /*!
       * \brief Stream insertion operator.
       *
       * \param os  Output stream.
       * \param obj Object to insert.
       *
       * \return Reference to output stream.
       */
      friend std::ostream &operator<<(std::ostream &os, const EigenGeoObj &obj);

    protected:
      //
      // Meta-data and accounting
      //
      size_t            m_id;         ///< geometric object id
      std::string       m_text;       ///< descriptive text
      bool              m_isValid;    ///< object [not] processed or valid
      bool              m_madePool;   ///< this object did [not] make the pool
      EigenVertexPool  *m_pool;       ///< pool of shared vertices
      IIndexMap         m_iindices;   ///< map from vertices to pool
      size_t            m_capacity;   ///< vertices reserved capacity

      //
      // Object
      //
      EigenExVertexList m_vertices;   ///< object vertices

      //
      // Properties
      //
      EigenBoundary3    m_xyzRange;   ///< cuboid x-y-z range of object
      EigenBoundary3    m_rtpRange;   ///< spherical shell arc range of object
      EigenSubtend2     m_subtended;  ///< subtended solid angle of object

      /*!
       * \breif Bind to vertex pool.
       */
      void bindPool()
      {
        if( m_pool != NULL )
        {
          m_pool->bind();
        }
      }

      /*!
       * \breif Unbind from vertex pool.
       */
      void unbindPool()
      {
        if( m_pool != NULL )
        {
          m_pool->unbind();

          // last man out of the pool
          if( m_pool->refcnt() == 0 )
          {
            delete m_pool;
            m_pool = NULL;
          }
        }
      }

      /*!
       * \brief Helper function to copy a vertex from the pool to the object's
       * list of vertices.
       *
       * \param i   Pool vertex index.
       *
       * \return Added vertex index in object's list of vertices.
       */
      size_t copyFromPool(const size_t i);

      /*!
       * \brief Helper function to add new vertex to the object's list of
       * vertices.
       *
       * If a pool is bound, then the vertex is also written through to the
       * vertex pool.
       *
       * \param v   New vertex.
       *
       * \return Added vertex index in object's list of vertices.
       */
      size_t writeThrough(const EigenVertex &v);

    };  // class EigenGeoObj


    //--------------------------------------------------------------------------
    // Class EigenFacet
    //--------------------------------------------------------------------------
 
    /*!
     * \brief The EigenFacet container class.
     *
     * Facets are planar simple, closed polygons in ambient 3D space. Simple
     * facets have no holes nor self edge crossings. The facet can be convex or
     * concave.
     */
    class EigenFacet : public EigenGeoObj
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
        POLYGON     ///< general polygon
      };

      /*!
       * \brief Default constructor.
       *
       * \param id    Facet id.
       * \param pool  Pointer to shared vertex pool.
       */
      EigenFacet(const size_t id = 100, EigenVertexPool *pool = NULL);

      /*!
       * \brief Copy constructor.
       *
       * \param src   Source object.
       */
      EigenFacet(const EigenFacet &src);

      /*!
       * \brief Destructor.
       */
      virtual ~EigenFacet();

      /*!
       * \brief Make a triangular facet.
       *
       * The source of the vertices are from the vertex pool.
       *
       * The first two vertices form the base to calculate facet length.
       * The width is the standard triangle height calculation.
       *
       * \param i       Base vertex pool index 0.
       * \param j       Base vertex pool index 1.
       * \param k       Third vertex of triangle pool index.
       * \param orient  Orientation w.r.t. the x-y plane.
       */
      void makeTriangle(const size_t i,
                        const size_t j,
                        const size_t k,
                        const Orientation orient = ANGLED);

      /*!
       * \brief Make a triangular facet.
       *
       * The first two vertices form the base to calculate facet length.
       * The width is the standard triangle height calculation.
       *
       * The vertices are specified in x,y,z Cartesian coordinates.
       *
       * The vertices are also added to any bound vertex pool.
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
       * \brief Make a rectangular facet.
       *
       * The source of the vertices are from the vertex pool.
       *
       * The first two vertices form the base to calculate facet length.
       * The width is L2Dist(v0, v3) (or L2Dist(v1, v2)).
       *
       * \param i       Base vertex pool index 0.
       * \param j       Base vertex pool index 1.
       * \param k       Opposing vertex to vertex 1 pool index.
       * \param l       Opposing vertex to vertex 0 pool index.
       * \param orient  Orientation w.r.t. the x-y plane.
       */
      void makeRectangle(const size_t i, const size_t j,
                         const size_t k, const size_t l,
                         const Orientation orient = ANGLED);

      /*!
       * \brief Make a rectangular facet.
       *
       * The first two vertices form the base to calculate facet length.
       * The width is L2Dist(v0, v3) (or L2Dist(v1, v2)).
       *
       * The vertices are specified in x,y,z Cartesian coordinates.
       *
       * The vertices are also added to any bound vertex pool.
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
       * \brief Make a general polygonal facet.
       *
       * The source of the vertices are from the vertex pool.
       *
       * \param indices   List of vertex pool indices.
       * \param shape     Shape of polygon.
       * \param orient    Orientation w.r.t. the x-y plane.
       */
      void makePolygon(const IndexList   &indices,
                       const Shape       shape  = POLYGON,
                       const Orientation orient = ANGLED);

      /*!
       * \brief Make a general polygonal facet.
       *
       * The vertices are specified in x,y,z Cartesian coordinates.
       *
       * The vertices are also added to any bound vertex pool.
       *
       * \param vertices  List of vertices.
       * \param shape     Shape of polygon.
       * \param orient    Orientation w.r.t. the x-y plane.
       */
      void makePolygon(const EigenVertexList &vertices,
                       const Shape shape = POLYGON,
                       const Orientation orient = ANGLED);

      /*!
       * \brief Process facet to validate and determine geometric properties.
       *
       * A minimum of 3 vertices are required.
       *
       * Facet properties such as bounds and dimensions are calculated.
       */
      virtual void process();

      /*!
       * \brief Find the line parameter and the corresponding point of the
       * intersection between a line and this facet.
       *
       * \param       line    Parameterized line in ambient 3D space.
       * \param[out]  t       Intersection parameter value.
       * \param[out]  pt      Intersection point.
       *
       * \return Returns true if there is an intersection, false otherwise.
       */
      bool intersection(const EigenLine3 &line,
                        double           &t,
                        EigenPoint3      &pt) const;

      /*!
       * \brief Find the intersection between a line and this facet.
       *
       * \param       line    Parameterized line in ambient 3D space.
       * \param[out]  pt      Intersection point.
       *
       * \return Returns true if there is an intersection, false otherwise.
       */
      bool intersection(const EigenLine3 &line, EigenPoint3 &pt) const
      {
        double t;

        return intersection(line, t, pt);
      }

      /*!
       * \brief Return the reference to the coplanar infinite plane.
       *
       * \return Reference to plane.
       */
      const EigenPlane3 &plane() const
      {
        assert(m_isValid);
        return m_plane;
      }

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
       * \return The length x width dimensions.
       */
      const EigenPoint2 &dimensions() const
      {
        assert(m_isValid);
        return m_dim;
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
       * \brief Set the facet color attribute.
       *
       * \param color Red-green-blue-alpha color.
       */
      void setAttr(const EigenRGBA &color)
      {
        m_color = color;
      }

      /*!
       * \brief Specify the facet shape and orientation attributes.
       *
       * \param shape   (Special) shape of polygon.
       * \param orient  (Special) orientation of polygon.
       */
      void setAttr(const Shape shape, const Orientation orient);

      /*!
       * \brief Stream insertion operator.
       *
       * \param os  Output stream.
       * \param obj Object to insert.
       *
       * \return Reference to output stream.
       */
      friend std::ostream &operator<<(std::ostream &os, const EigenFacet &obj);

    protected:
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
      EigenPoint2       m_dim;        ///< length x width dimensions

      /*!
       * \brief Calculate the solid subtended angle of the facet.
       *
       * \param[out] subtend  Subtended solid angle theta, phi.
       */
      virtual void calcSubtended(EigenSubtend2 &subtend);

      /*!
       * \brief Calculate the length x width dimensions of the facet.
       *
       * The dimensions do not define the minimum circumscribed rectangle about
       * the facet. For the most part, the length is colinear to the line
       * defined by the first two vertices. The width is orthogonal to the
       * length. 
       *
       * \param[out] dim  The length x width dimensions.
       */
      virtual void calcDimensions(EigenPoint2 &dim);

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
    // edges, facets, and an orientation relationship between them. If the
    // facets form a closed interior, then the polyhedral surface is a
    // polyhedron.
    //
    // By convention, the vertices  are oriented counterclockwise
    // around facets as seen from the outside of the polyhedron. That is,
    // the normals point outward.
    //
    class EigenPolyhedron : public EigenGeoObj
    {
    public:
      /*! default reserved capacity of polyhedron's list of facets */
      static const size_t FacetsCapacityDft = 10;

      /*!
       * \brief Default constructor.
       *
       * \param id    Polyhedron id.
       * \param pool  Pointer to vertex pool.
       */
      EigenPolyhedron(const size_t id = 0, EigenVertexPool *pool = NULL);

      /*!
       * \brief Copy constructor.
       *
       * \param src   Source object.
       */
      EigenPolyhedron(const EigenPolyhedron &src);

      /*!
       * \brief Destructor.
       */
      virtual ~EigenPolyhedron();

      /*!
       * \brief Clear the polyhedron vertices and facets.
       */
      virtual void clear();

      /*!
       * \brief Set the reserve capacity of the polyhedron facet list.
       *
       * \param n   New capacity in the number of elements.
       *
       * \return New capacity.
       */
      size_t facetCapacity(size_t n)
      {
        if( m_facets.size() < n )
        {
          m_capacity = n;
          m_facets.reserve(m_capacity);
        }
        return m_facets.capacity();
      }

      /*!
       * \brief Process polyhedron to validate and determine geometric
       * properties.
       *
       * A minimum of 1 facet is required.
       *
       * Polygonal properties such as bounds and dimensions are calculated.
       */
      virtual void process();

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
       * \endcode
       *
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
       * \param i       Polyhedron vertex index i. 1st base point of triangle.
       * \param j       Polyhedron vertex index j. 2nd base point of triangle.
       * \param k       Polyhedron vertex index k. 3rd point of triangle.
       * \param orient  Orientation w.r.t. the x-y plane.
       */
      size_t addTriangleFacet(const size_t i, const size_t j, const size_t k,
                     const EigenFacet::Orientation orient = EigenFacet::ANGLED);

      /*!
       * \brief Make rectangular facet.
       *
       * The first two vertices form the base to calculate facet length.
       * The width is the L2 distance from vertices j,k (or i,l).
       *
       * Vertices must be already added to polyhedron.
       *
       * \param i       Polyhedron vertex index i. 1st base point of rectangle.
       * \param j       Polyhedron vertex index j. 2nd base point of rectangle.
       * \param k       Polyhedron vertex index k. Opposing point to j.
       * \param l       Polyhedron vertex index l. Opposing point to i.
       * \param orient  Orientation w.r.t. the x-y plane.
       */
      size_t addRectangleFacet(const size_t i, const size_t j,
                               const size_t k, const size_t l,
                    const EigenFacet::Orientation orient = EigenFacet::ANGLED);

      /*!
       * \brief Make general polygonal facet.
       *
       * \param vertices  List of polyhedron vertex indices.
       * \param shape     Shape of polygon.
       * \param orient    Orientation w.r.t. the x-y plane.
       */
      size_t addPolygonFacet(const IndexList &indices,
                    const EigenFacet::Shape shape        = EigenFacet::POLYGON,
                    const EigenFacet::Orientation orient = EigenFacet::ANGLED);

      /*!
       * \brief Add a vertex to the identified facet.
       *
       * Orientation order here is define by the sequence of calls to this
       * function.
       *
       * Vertices must be already added to polyhedron.
       *
       * \param id  The facet identifier.
       * \param i   Polyhedron index of the vertex.
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
       * \brief Return list of facets.
       *
       * \return Reference to vertices.
       */
      const EigenFacetList &facets() const
      {
        return m_facets;
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

      /*!
       * \brief Stream insertion operator.
       *
       * \param os  Output stream.
       * \param obj Object to insert.
       *
       * \return Reference to output stream.
       */
      friend std::ostream &operator<<(std::ostream &os,
                                      const EigenPolyhedron &obj);

    protected:
      size_t            m_capacity;   ///< facet reserved capacity
      EigenFacetList    m_facets;     ///< polyhedral facets

      /*!
       * \brief Calculate the solid subtended angle of the polyhedron.
       *
       * \param[out] subtend  Subtended solid angle theta, phi.
       */
      virtual void calcSubtended(EigenSubtend2 &subtend);

    }; // class EigenPolyhedron

    /*! \} */ // end of gfmath doxy group

  } // namespace gf_math

} // namespace geofrenzy


#endif // _GF_POLY_H
