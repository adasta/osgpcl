/*
 * point_cloud_reader.h
 *
 *  Created on: Jul 27, 2012
 *      Author: Adam Stambler
 */

#ifndef POINT_CLOUD_READER_H_
#define POINT_CLOUD_READER_H_

#include <osgpcl/point_cloud.h>

#include <osgDB/ReaderWriter>
#include <osgDB/Options>

#include <osgDB/Registry>



namespace osgPCL
{

  class PointCloudReader : public osgDB::ReaderWriter
  {
    public:
      PointCloudReader ();
      PointCloudReader(const ReaderWriter& rw,const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY);
      virtual ~PointCloudReader ();

      META_Object(osgPCL,PointCloudReader);

      /** Return available features*/
      virtual Features supportedFeatures() const;

      /** Return true if ReaderWriter accepts specified file extension.*/
     // virtual bool acceptsExtension(const std::string& extension) const;


      virtual ReadResult readNode(const std::string& fileName, const osgDB::ReaderWriter::Options* options) const;

   //   virtual ReadResult readNode(std::istream& fin, const Options* options) const;

   //  /virtual  ReadResult readObject(const std::string& filename,const Options* options =NULL) const ;

      class CloudLoadingOptions : public osgDB::Options {
        public:
          pcl::IndicesConstPtr indices;
          float sampe_percentage;
          osg::ref_ptr<osgPCL::PointCloudFactory> factory;
          CloudLoadingOptions() : sampe_percentage(100.0f){}
          CloudLoadingOptions(const CloudLoadingOptions& options){
            indices = options.indices;
            sampe_percentage = options.sampe_percentage;
            factory = options.factory;
          }
      } ;


    protected:
      void initsupport();
  };

} /* namespace osgPCL */

USE_OSGPLUGIN(pcd)

#endif /* POINT_CLOUD_READER_H_ */
