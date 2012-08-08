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

namespace osgpcl
{

  template class PointCloudColoredFactory<pcl::PointXYZ>;
  template class PointCloudRGBFactory<pcl::PointXYZ, pcl::RGB>;
  template class PointCloudCRangeFactory<pcl::PointXYZ, pcl::PointXYZ>;
  template class PointCloudLabelFactory<pcl::PointXYZ, pcl::Label>;
}

osgpcl::PointCloudFactory::PointCloudFactory(){
  stateset_ = new osg::StateSet;
   osg::Point* p = new osg::Point;
   p->setSize(4);
   stateset_->setAttribute(p);
   stateset_->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
}

osg::Node* osgpcl::PointCloudFactory::buildNode ()
{
  osg::Geode* geode = new osg::Geode;
  geode->getDescriptions().push_back("PointCloud");
  geode->addDrawable(buildGeometry());
  return geode;
}
/* namespace osgPCL */
