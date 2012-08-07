/*
 * oc_viewer.cpp
 *
 *  Created on: Aug 3, 2012
 *      Author: Adam Stambler
 */


#include <pcl/point_types.h>
#include <pcl/outofcore/outofcore.h>
#include <pcl/outofcore/impl/octree_base.hpp>
#include <pcl/outofcore/impl/octree_disk_container.hpp>
#include <pcl/outofcore/impl/octree_base_node.hpp>
#include <pcl/outofcore/impl/octree_ram_container.hpp>
#include <osgpcl/point_cloud.h>

typedef pcl::PointXYZ PointT;
typedef pcl::outofcore::octree_base<pcl::outofcore::octree_disk_container<PointT> , PointT> octree_disk;
typedef pcl::outofcore::octree_base_node<pcl::outofcore::octree_disk_container<PointT> , PointT> octree_disk_node;
typedef Eigen::aligned_allocator<PointT> AlignedPointT;

#include <osg/Geode>
#include <osgViewer/Viewer>


#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>

#include <osgpcl/outofcore_octree_reader.h>


#include <osgDB/ReadFile>
#include <osgDB/Registry>


int main(int argc, char** argv){

  std::string tree_root =argv[1] ;
  double sample = atof(argv[2]);
  int depth = atoi(argv[3]);

  osg::ref_ptr< osgpcl::PointCloudCRangeFactory<> >   cfactory(new osgpcl::PointCloudCRangeFactory<>);
  cfactory->setField("z");
  cfactory->setRangle(1000,1100);

  osgpcl::OutofCoreOctreeReader::OutOfCoreOptions* options = new osgpcl::OutofCoreOctreeReader::OutOfCoreOptions(cfactory,0.1);

  osgViewer::Viewer viewer;
  viewer.setUpViewInWindow(0,0,500,500,0);

  osgDB::Options* opts = new osgDB::Options;
  opts->setUserData(options);

  viewer.setSceneData( osgDB::readNodeFile(tree_root, opts));

 viewer.getCamera()->setClearColor( osg::Vec4(0,0,0,1));
 return viewer.run();

}
