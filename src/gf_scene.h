////////////////////////////////////////////////////////////////////////////////
//
// Package:   Geofrenzy Robot ROS Package
//
// Link:      https://github.com/roadnarrows-robotics/geofrenzy
//
// File:      gf_scene.h
//
/*! \file
 *
 * \brief The Geofrenzy scene interface.
 *
 * A scene is composed of one or more geofences along with attached attributes.
 *
 * From the scene features, a synthetic set of depth plus color points may be
 * generated. These data points can be converted to a standard set of
 * ROS messages to create virtual sensors such as structured light sensors and
 * laser scanners. The output are point clouds.
 *
 * \note Simple geometric objects are used. If there is a requriement for more 
 * realism and/or features, then more extensive graphics libraries will be
 * needed, such as openscenegraph, cgal, and opengl.
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

#ifndef _GF_SCENE_H
#define _GF_SCENE_H

#include <math.h>

#include <string>
#include <limits>
#include <ostream>

#include <Eigen/Eigen>
#include <Eigen/Geometry>

#include "geofrenzy/Polygon64.h"

#include "gf_math.h"
#include "gf_poly.h"

namespace geofrenzy
{
  namespace gf_scene
  {
    /*!
     * \defgroup gfscene Geofrence Scene
     * 
     * Geofences make up a virtual scene.
     * An Geofence scene is built up of objects, with each object
     * a set of attributes and surfaces.
     * \{
     */

    //--------------------------------------------------------------------------
    // Scene Data Types, Constants, and Defines
    //--------------------------------------------------------------------------
 
    //
    // Type shorteners. (Mama's little baby loves short'nin' bread)
    //
    typedef gf_math::EigenRGBA        LuxRGBA;        ///< RGB+Alpha intensities
    typedef gf_math::EigenXYZRGBAList PointCloudList; ///< point cloud list

    /*!
     * \brief Minimum fence height (meters).
     */
    const double FenceMinHeight = 0.10;

    /*!
     * \brief Default fence color. 50% transparent blueish gray.
     */
    const LuxRGBA FenceColorDft(0.30, 0.30, 0.45, 0.50);


    //////////////// OLD ///////////////////
    /*!
     * \brief Eigen planar surface rendering properties.
     *
     * The properties are used to speed computer rendering of a surface.
     */
    struct EigenSurface
    {
      unsigned int              m_num;        ///< surface number in object
      gf_math::EigenPoint3List  m_vertices;   ///< surface vertices
      gf_math::EigenPlane3      m_plane;      ///< infinite surface plane
      double                    m_altitude;   ///< surface base altitude
      double                    m_length;     ///< surface length
      double                    m_height;     ///< surface height from base
      double                    m_inclination;///< surface inclination angle from x-axis
      double                    m_projection; ///< projected origin on base 2D line
      gf_math::EigenBoundary3   m_bounds;       ///< clipping bounding box
      gf_math::EigenMinMax2     m_thetas;     ///< horizontal apparent theta limits
      double                m_subtended;  ///< subtended angle of surface from viewer
    };

    /*!
     * \brief List (vector) of surfaces container type.
     */
    typedef std::vector<EigenSurface> EigenSurfaceList;

    /*! 
     * \brief Eigen scene object data type.
     *
     * Each object has a a set of rendering attributes (e.g. color, texture)
     * plus a set of surfaces. The set of surfaces typically specify a fully
     * connected, close shape (e.g. polyhedron), but this is not a requirement.
     *
     * A scene object must contain at least one surface.
     */
    struct EigenSceneObj
    {
      LuxRGBA                  m_color;      ///< RGBA color attribute
      bool                     m_hasCaps;    ///< object has top and bottom caps
      gf_math::EigenBoundary3  m_bounds;     ///< object bounding 3D box
      gf_math::EigenPoint2List m_footprint;  ///< object base footprint
      EigenSurfaceList         m_surfaces;   ///< the object surface properties

      /*!
       * \brief Clear scene object of surfaces and set attribute defaults.
       */
      void clear()
      {
        m_color   = FenceColorDft;
        m_hasCaps = false;
        m_footprint.clear();
        m_surfaces.clear();
      }
    };

    /*! 
     * \brief Eigen scene data type is a vector of scene objects.
     */
    typedef std::vector<EigenSceneObj> EigenScene;

    /*!
     * \brief Scanning bit-or'ed options.
     */
    enum ScanOptions
    {
      /*!
       * The default scanning option.
       *  - Include all instersecting points along any traced ray. This option
       *    excludes use of the 2D option.
       *  - Do not produce any 2D structure. That is, if no intersection along
       *    a traced ray is detected, no point is added.
       *  - Do not alpha blend colors along ray.
       */
      ScanOptionDft = 0x00,

      /*!
       * Generate a full height x width 2D structure of points. Any "no object"
       * point has a value of inf.
       */
      ScanOption2D  = 0x01,

      /*!
       * Include only the nearest point of a set of intersections along a
       * traced ray.
       */
      ScanOptionNearest = 0x02,

      /*!
       * Alpha blend colors.
       */
      ScanOptionAlphaBlend = 0x04
    };

    ///@{
    /*!
     * \brief Useful scan option macros.
     *
     * \param _opt  Option bits
     *
     * \return Boolean
     */
    #define SCANOPT_2D(_opt)            ((_opt) & ScanOption2D)
    #define SCANOPT_ALPHA_BLEND(_opt)   ((_opt) & ScanOptionAlphaBlend)
    #define SCANOPT_NEAREST_ONLY(_opt)  ((_opt) & ScanOptionNearest)
    #define SCANOPT_XRAY_VIS(_opt)      !SCANOPT_NEAREST_ONLY(_opt)
    ///@}


    //--------------------------------------------------------------------------
    // Struct GeofenceObj
    //--------------------------------------------------------------------------

    /*!
     * \brief A simple wrapper around a geofence.
     * 
     * If a geofence requires more metadata, this wrapper readily allows for
     * that data expansion.
     */
    struct GeofenceObj
    {
      //
      // Metadata
      //
      size_t      m_id;       ///< geofence id
      bool        m_hasCaps;  ///< object does [not] have caps
      std::string m_text;     ///< geofence name or owner or description

      //
      // The fence
      //
      gf_math::EigenPolyhedron  m_polyhedron; ///< geofence boundary
    };

    //
    // Geofence objects list types.
    //
    typedef std::vector<GeofenceObj>        GeofenceObjList;  ///< fence list
    typedef GeofenceObjList::iterator       GeofenceObjIter;  ///< fence iter
    typedef GeofenceObjList::const_iterator GeofenceObjCIter; ///< fence citer


    //--------------------------------------------------------------------------
    // Class GeofenceScene
    //--------------------------------------------------------------------------

    /*!
     * \brief GeofenceScene descriptor container class.
     *
     * A geofence scene contains a set of geofences along with associated
     * attributes. A geofence is a a virtual geographic boundary.
     *
     * A geofence usually defines a closed space, but it may be open (think
     * the Great Wall of China).
     *
     * A geofence perimeter is composed of connected, vertically oriented,
     * rectangular polygons. This perimeter then forms a polyhedral surface.
     * Horizontal ceiling and floor caps may added to a closed geofence to
     * form a right prism polyhedron. A right prism is a prism in which the
     * joining edges and faces are perpendicular to the base faces.
     *
     * The base footprint of the geofence may outline either a convex or concave
     * polygon.
     */
    class GeofenceScene
    {
    public:
      /*!
       * \brief Default constructor.
       */
      GeofenceScene();

      /*!
       * \brief Destructor.
       */
      virtual ~GeofenceScene();

      /*!
       * \brief Add a geofence to the scene.
       *
       * \param id          Geofence identifier.
       * \param polygon     ROS polygon defining a fence. Each (x,y,z) point
       *                    specifies a distance from a reference point or an
       *                    observer (meters).
       * \param color       Color attribute applied to the fence.
       *                    (red-green-blue intensities + alpha).
       * \param fenceAlt    Base altitude from ground (meters).
       * \param fenceHeight Height of fence from base (meters).  
       * \param hasCaps     The object is a polyhedron. That is, it has both
       *                    top(ceiling) and bottom(floor) horizontal caps.
       * \param text        Optional informative text.
       */
      void addFence(const size_t      fenceId,
                    const Polygon64   &polygon,
                    const LuxRGBA     &color,
                    const double      fenceAlt,
                    const double      fenceHeight,
                    const bool        hasCaps,
                    const std::string &text = "");

      /*!
       * \brief Clear scene of all data.
       */
      void clear();

      /*!
       * \brief Return the number of geofences in scene.
       *
       * \return Number of fences.
       */
      size_t numOfFences() const
      {
        return m_geofences.size();
      }

      /*!
       * \brief Return the geofence at index i.
       *
       * \param i Index.
       *
       * \return Geofence object.
       */
      GeofenceObj &fenceAt(size_t i)
      {
        assert(i < m_geofences.size());
        return m_geofences[i];
      }

      /*!
       * \brief Return the geofence at index i.
       *
       * \param i Index.
       *
       * \return Constant Geofence object.
       */
      const GeofenceObj &fencetAt(size_t i) const
      {
        assert(i < m_geofences.size());
        return m_geofences[i];
      }


      /*!
       * \brief Return an iterator pointing to the first element of the list
       * of geofences.
       *
       * \return Iterator.
       */
      GeofenceObjIter iterFenceBegin()
      {
        return m_geofences.begin();
      }

      /*!
       * \brief Return an iterator referring to the past-the-end element of
       * the list of goefences.
       *
       * \return Iterator.
       */
      GeofenceObjIter iterFenceEnd()
      {
        return m_geofences.end();
      }

      /*!
       * \brief Return a constant iterator pointing to the first element of
       * the list of goefences.
       *
       * \return Constant Iterator.
       */
      GeofenceObjCIter iterFenceBegin() const
      {
        return m_geofences.begin();
      }

      /*!
       * \brief Return a constant iterator referring to the past-the-end
       * element of the list of goefences.
       *
       * \return Constant Iterator.
       */
      GeofenceObjCIter iterFenceEnd() const
      {
        return m_geofences.end();
      }

    protected:
      GeofenceObjList m_geofences;    ///< list of geofence objects

    }; // class GeofenceScene


    //--------------------------------------------------------------------------
    // Class SceneScanner
    //--------------------------------------------------------------------------

    /*!
     * \brief Scene scanner virtual base class.
     */
    class SceneScanner
    {
    public:
      /*!
       * \brief Default constructor.
       *
       * \param strScannerName  Name of scanner.
       */
      SceneScanner(const std::string &strScannerName) :
          m_name(strScannerName)
      {
      }

      /*!
       * \brief Destructor.
       */
      virtual ~SceneScanner()
      {
      }

      /*!
       * \brief Scan virtual member function.
       *
       * \param       scene       Scene to scan.
       * \param[out]  intersects  Output list of xyz rgba points.
       *
       * \return Number of points.
       */
      size_t virtual scan(const GeofenceScene &scene,
                          PointCloudList      &intersects) = 0;

      /*!
       * \brief Return name of this scanner.
       *
       * \return String.
       */
      const std::string &name() const
      {
        return m_name;
      }

    protected:
      std::string m_name;   ///< scanner name
    }; // class SceneScanner


    //--------------------------------------------------------------------------
    // Class GridSceneScanner
    //--------------------------------------------------------------------------

    /*!
     * \brief Geofence scene grid scanner class.
     */
    class GridSceneScanner : public SceneScanner
    {
    public:
      static const double GridSizeMin;    ///< grid size minimum (meters)
      static const double GridSizeDft;    ///< grid size default (meters)

      /*!
       * \brief Default constructor.
       */
      GridSceneScanner();

      /*!
       * \brief Initialization constructor.
       * 
       * \param       gridSize    Grid size (meters).
       * \param       options     Options to control the scan.
       */
      GridSceneScanner(const double   gridSize,
                       const uint32_t options = ScanOptionDft);

      /*!
       * \brief Destructor.
       */
      virtual ~GridSceneScanner();

      /*!
       * \brief (Re)Set scanner properties.
       *
       * \param gridSize    Grid size (meters).
       */
      void setProperties(const double gridSize);

      /*!
       * \brief Scan virtual scene to generate a list of intersecting depth plus
       * color points.
       *
       * With this scanner, the scene is a priori known and a fast grid of the
       * fences is performed.
       *
       * \param       scene       Scene to scan.
       * \param[out]  intersects  Output list of xyzrgba points where xyz values
       *                          are in meters, rgb in intensities [0.0, 1.0],
       *                          and alpha [0.0, 1.0] transparent to opaque.
       *
       * \return Number of points.
       */
      size_t virtual scan(const GeofenceScene &scene,
                          PointCloudList      &intersects);

    protected:
      double    m_gridSize;   ///< grid size (m)
      uint32_t  m_options;    ///< options to control the scan.

      size_t gridWall(const gf_math::EigenFacet &facet,
                      PointCloudList            &intersects);

      size_t gridWallStripe(const gf_math::EigenFacet  &facet,
                            const gf_math::EigenPoint3 &basept,
                            PointCloudList             &intersects);

      size_t gridCap(const gf_math::EigenFacet &facet,
                     PointCloudList            &intersects);

    }; // class GridSceneScanner


    //--------------------------------------------------------------------------
    // Class SensorSceneScanner
    //--------------------------------------------------------------------------

    /*!
     * \brief Geofence scene sensor scanner class.
     */
    class SensorSceneScanner : public SceneScanner
    {
    public:
      //
      // Width resolution
      //
      static const size_t WidthMin;   ///< minimum scan width (steps)
      static const size_t WidthDft;   ///< default scan width (steps)

      //
      // Height resolution
      //
      static const size_t HeightMin;  ///< minimum scan height (steps)
      static const size_t HeightDft;  ///< default scan height (steps)

      //
      // Horizontal Field of View
      //
      static const double HFoVMinMin; ///< horizontal minimum minimum (radians)
      static const double HFoVMaxMax; ///< horizontal maximum maximum (radians)
      static const double HFoVMinDft; ///< horizontal default minimum (radians)
      static const double HFoVMaxDft; ///< horizontal default maximum (radians)

      //
      // Vertical Field of View
      //
      static const double VFoVMinMin; ///< vertical minimum minimum (radians)
      static const double VFoVMaxMax; ///< vertical maximum maximum (radians)
      static const double VFoVMinDft; ///< vertical default minimum (radians)
      static const double VFoVMaxDft; ///< vertical default maximum (radians)

      /*!
       * \brief Default constructor.
       */
      SensorSceneScanner();

      /*!
       * \brief Initialization constructor.
       *
       * \param thetaMin  Azimuthal minimum angle from x+ axis (-pi, pi].
       * \param thetaMax  Azimuthal max angle from x+ axis (-pi, pi].
       * \param phiMin    Polar minimum angle from z+ [0, pi].
       * \param phiMax    Polar maximum angle from z+ [0, pi].
       * \param width     Width resolution. Number of horizontal points.
       * \param height    Height resoluion. Number of vertical points.
       */
      SensorSceneScanner(const double thetaMin, const double thetaMax,
                         const double phiMin,   const double phiMax,
                         const size_t width,    const size_t height,
                         const uint32_t         options = ScanOptionDft);

      /*!
       * \brief Destructor.
       */
      virtual ~SensorSceneScanner();

      /*!
       * \brief (Re)Set scanner properties.
       *
       * \param thetaMin  Azimuthal minimum angle from x+ axis (-pi, pi].
       * \param thetaMax  Azimuthal max angle from x+ axis (-pi, pi].
       * \param phiMin    Polar minimum angle from z+ [0, pi].
       * \param phiMax    Polar maximum angle from z+ [0, pi].
       * \param width     Width resolution. Number of horizontal points.
       * \param height    Height resoluion. Number of vertical points.
       */
      void setProperties(const double thetaMin, const double thetaMax,
                         const double phiMin,   const double phiMax,
                         const size_t width,    const size_t height);

      /*!
       * \brief Scan virtual scene to generate a list of intersecting depth plus
       * color points.
       *
       * With this scanner, the scene is a posteriori detected by a virtual
       * scanning sensor.
       *
       * Scanning proceeds from the minimum to maximum phi in height steps, with
       * each step sweeping from the minimum to maximum theta in width steps.
       *
       * The spherical angles are as used in mathematics.
       *
       * \param       scene       Scene to scan.
       * \param[out]  intersects  Output list of xyzrgba points where xyz values
       *                          are in meters, rgb in intensities [0.0, 1.0],
       *                          and alpha [0.0, 1.0] transparent to opaque.
       *                          Order of the points is determined by the
       *                          options.
       *
       * \return Number of points.
       */
      size_t virtual scan(const GeofenceScene &scene,
                          PointCloudList      &intersects);

    protected:
      double    m_thetaMin;   ///< azimuthal min angle from x+ axis (-pi, pi]
      double    m_thetaMax;   ///< azimuthal max angle from x+ axis (-pi, pi]
      double    m_phiMin;     ///< polar minimum angle from z+ [0, pi]
      double    m_phiMax;     ///< polar maximum angle from z+ [0, pi]
      size_t    m_width;      ///< width resolution
      size_t    m_height;     ///< height resoluion
      double    m_thetaStep;  ///< theta step size (radians)
      double    m_phiStep;    ///< phi step size (radians)
      uint32_t  m_options;    ///< options to control the scan.

    }; // class SensorSceneScanner


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Ouput types  
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Geofrenzy ouput data types.
     *
     * From Geofrenzy features, a synthetic set of depth plus color points 
     * are generated. These data points can be converted to standard set of
     * ROS messages to create virtual sensors such as point cloud structured
     * light sensors and laser scanners. 
     */

    /*!
     * \brief Create a scene object.
     *
     * \param polygon       ROS polygon defining a fence. Each (x,y,z) point
     *                      specifies a distance from a reference point or an
     *                      observer (meters).
     * \param color         Color attribute applied to the fence.
     *                      (red-green-blue intensities + alpha).
     * \param fenceAlt      Base altitude from ground (meters).
     * \param fenceHeight   Height of fence from base (meters).  
     * \param hasCaps       The object is a polyhedron. That is, it has both
     *                      top(ceiling) and bottom(floor) horizontal caps.
     * \param[out] sceneObj Created scene object.
     */
    void createSceneObj(const Polygon64 &polygon,
                        const LuxRGBA   &color,
                        const double    &fenceAlt,
                        const double    &fenceHeight,
                        const bool      hasCaps,
                        EigenSceneObj   &sceneObj);

    /*!
     * \brief Scan virtual scene to generate a list of intersecting depth +
     * color points.
     *
     * In this operation mode, the scene is a posteriori detected by a virtual
     * scanning sensor.
     *
     * Scanning proceeds from the minimum to maximum phi in height steps, with
     * each step sweeping from the minimum to maximum theta in width steps.
     *
     * The spherical angles are as used in mathematics.
     *
     * \param       thetaMin    Azimuthal minimum angle from x+ axis (-pi, pi].
     * \param       thetaMax    Azimuthal max angle from x+ axis (-pi, pi].
     * \param       phiMin      Polar minimum angle from z+ [0, pi].
     * \param       phiMax      Polar maximum angle from z+ [0, pi].
     * \param       width       Width resolution. Number of horizontal points.
     * \param       height      Height resoluion. Number of vertical points.
     * \param       scene       The scene to scan.
     * \param[out]  intersects  List of intersecting points.
     * \param       options     Options to control the scan.
     */
    void scanScene(const double thetaMin, const double thetaMax,
                   const double phiMin,   const double phiMax,
                   const size_t width,    const size_t height,
                   const EigenScene       &scene,
                   PointCloudList         &intersects,
                   uint32_t               options = ScanOptionDft);

    /*!
     * \brief Grid virtual scene fences to generate a list of depth + color
     * points.
     *
     * In this operation mode, the scene is a priori known and a fast grid
     * of the fences is performed.
     *
     * \param       gridSize    Grid size.
     * \param       scene       The scene to scan.
     * \param[out]  intersects  List of intersecting points.
     * \param       options     Options to control the grid generations.
     */
    void gridScene(const double       gridSize,
                   const EigenScene   &scene,
                   PointCloudList     &intersects,
                   uint32_t           options = ScanOptionDft);

    /*! \} */ // gfmath_scene

    ///@{
    /*!
     * \brief Stream insertion operators.
     *
     * \param os  Output stream.
     * \param arg Object to insert.
     *
     * \return Reference to output stream.
     */
    std::ostream &operator<<(std::ostream &os, const EigenSurface &surface);

    std::ostream &operator<<(std::ostream &os, const EigenSceneObj &sceneObj);
    ///@}

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Unit Tests
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

#define GF_SCENE_UT  ///< define/undef to enable/disable unit test functions.

#ifdef GF_SCENE_UT
    /*!
     * \defgroup gfscene_ut Unit Test
     * \{
     */

    const int UtPolynumTriangle   = 0;  ///< equalateral triangle with 50m sides
    const int UtPolynumRectangle  = 1;  ///< 20m x 30m rectangle
    const int UtPolynumHexagon    = 2;  ///< hexagon with 10m sides
    const int UtPolynumTee        = 3;  ///< 40m x 50m tee 

    /*!
     * \brief Make a ROS polygon message from a canned shape.
     *
     * \param polynum       Canned shape number. See above.
     * \param offset        Offset add to polygon position.
     * \param scale         Polygon size scale multiplier. 
     * \param [out] polygon Output polygon message.
     */
    void utMakeCannedPolygon(const int         polynum,
                             const gf_math::EigenPoint3 &offset,
                             const double      scale,
                             Polygon64         &polygon);

    /*!
     * \brief Scan a single polygon.
     *
     * \param polygon Polygon to scan.
     */
    void utScanPolygon(const Polygon64 &polygon);
#endif // GF_SCENE_UT

    /*! \} */ // end of gfscene_ut group
    
    /*! \} */ // end of gfmath group

  } // namespace gf_scene

} // namespace geofrenzy

#endif // _GF_SCENE_H
