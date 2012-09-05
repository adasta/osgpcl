/*
 * surfel.cpp
 *
 *  Created on: Sep 4, 2012
 *      Author: asher
 */


#include <osgpcl/impl/surfel.hpp>
#include <osgpcl/utility_point_types.h>

namespace osgpcl{
template  class SurfelFactory<pcl::PointXYZ, pcl::Normal>;
template  class SurfelFactory<pcl::PointNormal, pcl::PointNormal>;

template class SurfelFactoryFF<pcl::PointXYZ, pcl::Normal, osgpcl::RadiusPointT>;

}
