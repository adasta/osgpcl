/*
 * point_cloud.cpp
 *
 *  Created on: Jun 23, 2012
 *      Author: asher
 */

#include <osgpcl/impl/point_cloud.hpp>
#include <osg/Point>
#include<pcl/point_types.h>

#include <osg/Geode>

namespace osgPCL
{

  template class PointCloudColoredFactory<pcl::PointXYZ>;
  template class PointCloudRGBFactory<pcl::PointXYZ, pcl::RGB>;
  template class PointCloudCRangeFactory<pcl::PointXYZ, pcl::PointXYZ>;

}

osg::Node* osgPCL::PointCloudFactory::buildNode ()
{
  osg::Geode* geode = new osg::Geode;
  geode->getDescriptions().push_back("PointCloud");
  geode->addDrawable(buildGeometry());
  return geode;
}
/* namespace osgPCL */
