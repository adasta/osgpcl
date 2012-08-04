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

  /*
  octree_disk octree (argv[1], false);


  Eigen::Vector3d bbmin, bbmax;

  octree.getBB(bbmin.data(), bbmax.data());

  std::cout << "Bounding box is " << bbmin.transpose() << "  to  " << bbmax.transpose() << " \n";

  pcl::PointCloud<pcl::PointXYZ>::Ptr  pcloud(new   pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr  ccloud(new   pcl::PointCloud<pcl::PointXYZ>);


 std::cout << "Coord system is " <<  octree.getCoordSystem() << " \n";



  std::cout << "Tree reports " << octree.getNumPointsAtDepth(depth) << " points \n";
  octree.queryBBIncludes_subsample(bbmin.data(),bbmax.data(),depth,sample, pcloud->points);
 // octree.queryBBIncludes(bbmin.data(),bbmax.data(),depth, pcloud->points);
  std::cout << "There are " << pcloud->points.size() << " points \n ";



  int count=0;
  std::vector<int> bad_idx;
  ccloud->points.reserve(pcloud->points.size());
  for(int i=0; i<pcloud->size(); i++){
    if  ( ( pcloud->points[i].x > bbmax[0]) || ( pcloud->points[i].y > bbmax[1] ) || (pcloud->points[i].z > bbmax[2])
        || ( pcloud->points[i].x < bbmin[0]) || ( pcloud->points[i].y < bbmin[1] ) || (pcloud->points[i].z < bbmin[2])  )  {
      continue;
    }
    ccloud->points.push_back(pcloud->points[i]);
  }
  std::cout << " ccloud " << ccloud->points.size() << " \n";
  {
    Eigen::Vector4f mipt, mapt;
    pcl::getMinMax3D(*ccloud, mipt, mapt);
    std::cout << "Mipt : " << mipt.transpose()<< "\n";
    std::cout << "Mapt : " << mapt.transpose()<< "\n";
  }

  std::cout << " \n";

  osg::Geode* geode = new osg::Geode;

  osgPCL::PointCloudCRangeFactory<>   cfactory;
  cfactory.setInputCloud<pcl::PointXYZ>(ccloud);
  cfactory.setField("z");

  geode->addDrawable(cfactory.buildGeometry());
  geode->setDataVariance(osg::Node::STATIC);

  osg::Group* root = new osg::Group;
  root->addChild(geode);
  */
  osgViewer::Viewer viewer;
  viewer.setUpViewInWindow(0,0,500,500,0);


  viewer.setSceneData( osgDB::readNodeFile(tree_root));

 viewer.getCamera()->setClearColor( osg::Vec4(0,0,0,1));
 return viewer.run();

}
