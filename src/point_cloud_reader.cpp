/*
 * point_cloud_reader.cpp
 *
 *  Created on: Jul 27, 2012
 *      Author: Adam Stambler
 */

#include "osgpcl/point_cloud_reader.h"
#include <boost/filesystem.hpp>

#include <pcl/io/pcd_io.h>
#include <osg/Geode>



namespace osgPCL
{
  REGISTER_OSGPLUGIN( pcd ,  PointCloudReader);

  PointCloudReader::PointCloudReader ()
  {
    supportsExtension("pcd","PCL Point Cloud Format");
  }

  PointCloudReader::PointCloudReader (const ReaderWriter& rw,
      const osg::CopyOp& copyop)
  {
    supportsExtension("pcd","PCL Point Cloud Format");
  }

  PointCloudReader::~PointCloudReader ()
  {
    // TODO Auto-generated destructor stub
  }


  osgDB::ReaderWriter::ReadResult PointCloudReader::readNode (const std::string& filename,
      const osgDB::ReaderWriter::Options* options) const
  {
    osg::ref_ptr< CloudLoadingOptions>  coptions = new CloudLoadingOptions;

    if ( dynamic_cast< const CloudLoadingOptions*>(options)  == NULL ){
      coptions->factory = new PointCloudCRangeFactory<>;
    }
    else{
      coptions->factory =  dynamic_cast< const CloudLoadingOptions*>(options)->factory ;
      coptions->sampe_percentage =  dynamic_cast< const CloudLoadingOptions*>(options)->sampe_percentage ;
      coptions->indices =  dynamic_cast< const CloudLoadingOptions*>(options)->indices ;
    }

    boost::filesystem::path fpath(filename);

    if (!boost::filesystem::exists(fpath)){
      return ReadResult(ReaderWriter::ReadResult::FILE_NOT_FOUND);
    }

    sensor_msgs::PointCloud2Ptr cloud(new sensor_msgs::PointCloud2);

    pcl::PCDReader reader;
    if ( reader.read(filename, *cloud) <0 ){
      return ReadResult("Failed to read point cloud\n");
    }

    coptions->factory->setInputCloud(cloud);
    PointCloudGeometry* geom = coptions->factory->buildGeometry();

    if (geom ==NULL){
      return ReadResult("Failed to build point cloud geometry\n");
    }

    osg::Geode* geode = new osg::Geode;
    geode->setName(filename.c_str());
    geode->getDescriptions().push_back("PointCloud");
    geode->addDrawable(geom);
    return geode;
  }

  osgDB::ReaderWriter::Features PointCloudReader::supportedFeatures () const
  {
    return FEATURE_READ_NODE;
  }

  void PointCloudReader::initsupport ()
  {
  }

} /* namespace osgPCL */
