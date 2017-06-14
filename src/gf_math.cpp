// GEOFRENZY FILE HEADER HERE

#include <iostream>
#include <string>

#include <Eigen/Eigen>
#include <Eigen/Geometry>

#include "gf_math.h"

using namespace std;

namespace geofrenzy
{
  namespace gf_math
  {
    void f()
    {
      double focalDistance = -418.5;
      Eigen::Vector3d p1(0,0,focalDistance);
      Eigen::Vector3d p2(1,0,focalDistance);
      Eigen::Vector3d p3(0,1,focalDistance);

// Here is my plane, it passes through 3 points
      Eigen::Hyperplane<double,3> focalPlane = Eigen::Hyperplane<double,3>::Through( p1,p2,p3 );

      Eigen::Vector3d n1(0,0,0);
      Eigen::Vector3d n2(0,1,1);

      // I initialize the line with two points passing through it
      Eigen::ParametrizedLine<double,3> pline = Eigen::ParametrizedLine<double,3>::Through(n1,n2);


// So if I'm correct this is the t at which pline intersect the last plane "focalPlane"    
      double intersection = pline.intersection( focalPlane ) ;

// Now, how to compute the coordinates of such intersection? I think this is the correct way...

      cerr << intersection*((n2-n1).normalized()) + n1  << endl;
    }

  } // namespace gf_math
} // namespace geofrenzy
