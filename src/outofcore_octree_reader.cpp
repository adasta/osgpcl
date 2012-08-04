/*
 * OutofCoreOctreeReader.cpp
 *
 *  Created on: Aug 4, 2012
 *      Author: Adam Stambler
 */

#include <osgpcl/outofcore_octree_reader.h>

#include <pcl/outofcore/impl/octree_base.hpp>
#include <pcl/outofcore/impl/octree_disk_container.hpp>
#include <pcl/outofcore/impl/octree_base_node.hpp>
#include <pcl/outofcore/impl/octree_ram_container.hpp>


namespace osgPCL
{
  REGISTER_OSGPLUGIN( oct_idx ,  OutofCoreOctreeReader);


  OutOfCoreOctree::~OutOfCoreOctree ()
  {
  }

  OutofCoreOctreeReader::OutofCoreOctreeReader ()
  {
    supportsExtension("oct_idx","PCL Point Cloud OutofCore Octree Format");
  }

  OutofCoreOctreeReader::OutofCoreOctreeReader (const OutofCoreOctreeReader& rw,
      const osg::CopyOp& copyop)
  {
  }

  OutofCoreOctreeReader::~OutofCoreOctreeReader ()
  {

  }

  osgDB::ReaderWriter::Features OutofCoreOctreeReader::supportedFeatures () const
  {
    return osgDB::ReaderWriter::FEATURE_READ_NODE;
  }

  osgDB::ReaderWriter::ReadResult OutofCoreOctreeReader::readNode (const std::string& fileName,
      const osgDB::ReaderWriter::Options* options) const
  {
    OutOfCoreOctree::Ptr octree;
    int read_depth;
    Eigen::Vector3d minpt, maxpt;
    float subsample;
    osg::ref_ptr<osgPCL::PointCloudFactory> factory;

    if (dynamic_cast<const OutOfCoreOptions*>(options) != NULL){
      const OutOfCoreOptions* oopts =   dynamic_cast<const OutOfCoreOptions*>(options);
      octree = oopts->octree;
      read_depth = oopts->root_depth;
      subsample = oopts->sample;
      minpt = oopts->bbmin;
      maxpt = oopts->bbmax;
    }
    else{
      if ( ! boost::filesystem::exists(fileName))  return osgDB::ReaderWriter::ReadResult::FILE_NOT_FOUND;
      OutofCoreOctreeT<pcl::PointXYZ>::OctreePtr ot (new OutofCoreOctreeT<pcl::PointXYZ>::Octree(fileName, false));
      OutofCoreOctreeT<pcl::PointXYZ>::Ptr tree (new OutofCoreOctreeT<pcl::PointXYZ>(ot));
      octree = tree;
      factory = new PointCloudColoredFactory<>;
      read_depth = 0;
      subsample=1;
      ot->getBB(minpt.data(), maxpt.data());
    }

    sensor_msgs::PointCloud2::Ptr cloud(new sensor_msgs::PointCloud2);
    if (subsample > 0.999){
      octree->queryBBIncludes(minpt.data(), maxpt.data(),read_depth, cloud);
    }
    else{
      octree->queryBBIncludes_subsample(minpt.data(), maxpt.data(),read_depth,subsample, cloud);
    }

    factory->setInputCloud(cloud);

    osg::Geode* geode = new osg::Geode;
    geode->setName(fileName.c_str());
    geode->getDescriptions().push_back("PointCloud");
    geode->addDrawable(factory->buildGeometry());
    return geode;
  }



} /* namespace osgPCL */
