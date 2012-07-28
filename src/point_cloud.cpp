/*
 * point_cloud.cpp
 *
 *  Created on: Jun 23, 2012
 *      Author: asher
 */

#include <osgpcl/impl/point_cloud.hpp>
#include <osg/Point>


#include<pcl/point_types.h>

namespace osgPCL
{

  template class PointCloudColoredFactory<pcl::PointXYZ>;
  template class PointCloudRGBFactory<pcl::PointXYZ, pcl::RGB>;
  template class PointCloudCRangeFactory<pcl::PointXYZ, pcl::PointXYZ>;

} /* namespace osgPCL */
