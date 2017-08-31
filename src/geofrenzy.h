////////////////////////////////////////////////////////////////////////////////
//
// Package:   Geofrenzy Robot ROS Package
//
// Link:      https://github.com/roadnarrows-robotics/geofrenzy
//
// File:      geofrenzy.h
//
/*! \file
 *
 * \brief The Geofrenzy portal interface.
 *
 * This should be defined somewhere in geofrenzy land, but here for now.
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

#ifndef _GEOFRENZY_H
#define _GEOFRENZY_H

// Note: should be in a geofrenzy header file
extern "C" 
{
  char *ambient_fences_geojson_zoom(double lng,
                                    double lat,
                                    int lvl,
                                    int myclass);

  char *ambient_fences_geojson_roi(double lng,
                                   double lat,
                                   int lvl,
                                   int myclass);

  char *class_entitlements_properties_json(int myclass);
}

#endif // _GEOFRENZY_H

