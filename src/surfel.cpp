/*
 * surfel.cpp
 *
 *  Created on: Sep 4, 2012
 *      Author: asher
 */


#include <osgpcl/impl/surfel.hpp>

namespace osgpcl{
template  class SurfelFactory<pcl::PointXYZ, pcl::Normal>;
template  class SurfelFactory<pcl::PointNormal, pcl::PointNormal>;
}
