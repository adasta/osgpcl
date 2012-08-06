/*
 * OutofCoreOctreeReader.h
 *
 *  Created on: Aug 4, 2012
 *      Author: Adam Stambler
 */

#ifndef OUTOFCOREOCTREEREADER_H_
#define OUTOFCOREOCTREEREADER_H_

#include <osgpcl/point_cloud.h>

#include <osgDB/ReaderWriter>
#include <osgDB/Options>
#include <osgDB/Registry>

#include <pcl/outofcore/outofcore.h>

#include <boost/random.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/bernoulli_distribution.hpp>
#include <pcl/common/io.h>

namespace osgPCL
{

  class OutOfCoreOctree
  {
    public:
      virtual ~OutOfCoreOctree();

      typedef boost::shared_ptr<OutOfCoreOctree> Ptr;

      virtual boost::uint64_t getTreeDepth() const =0;
      virtual void getBoundingBox(  double* min,   double* max)=0;

     virtual  void queryBBIncludes (const double min[3], const double max[3], size_t query_depth,
          const sensor_msgs::PointCloud2::Ptr& dst_blob) const =0;

     virtual  void queryBBIncludes_subsample (const double min[3], const double max[3], size_t query_depth,
         float subsample, const sensor_msgs::PointCloud2::Ptr& dst_blob) const =0;
  };


  template<typename PointT>
  class OutofCoreOctreeT : public OutOfCoreOctree{
    public:
      typedef pcl::outofcore::octree_base<pcl::outofcore::octree_disk_container<PointT> , PointT> Octree;
      typedef pcl::outofcore::octree_base_node<pcl::outofcore::octree_disk_container<PointT> , PointT> octree_disk_node;
      typedef boost::shared_ptr<Octree> OctreePtr;
      typedef boost::shared_ptr<OutofCoreOctreeT<PointT> > Ptr;

      OutofCoreOctreeT(const OctreePtr& octree);
      virtual  void queryBBIncludes (const double min[3], const double max[3], size_t query_depth,
               const sensor_msgs::PointCloud2::Ptr& dst_blob) const;
      virtual  void queryBBIncludes_subsample (const double min[3], const double max[3], size_t query_depth,
          float subsample, const sensor_msgs::PointCloud2::Ptr& dst_blob) const;
    protected:
      OctreePtr octree_;
    public:
      virtual boost::uint64_t getTreeDepth() const { return octree_->getDepth();};
      virtual void getBoundingBox(  double * min,   double * max){octree_->getBB(min, max);}

  };


  class OutofCoreOctreeReader : public osgDB::ReaderWriter
  {
    public:
      OutofCoreOctreeReader ();
      OutofCoreOctreeReader(const OutofCoreOctreeReader& rw,const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY);
      virtual ~OutofCoreOctreeReader ();

      META_Object(osgPCL,OutofCoreOctreeReader);

    /** Return available features*/
    virtual Features supportedFeatures() const;

    virtual ReadResult readNode(const std::string& fileName, const osgDB::ReaderWriter::Options* options) const;


      class OutOfCoreOptions : public osg::Referenced {
        public:
        OutOfCoreOptions(float sample = 1.0);
        OutOfCoreOptions(osgPCL::PointCloudFactory* _factory, float sample =1.0f);
        OutOfCoreOptions( const OutOfCoreOctree::Ptr& _octree,
                            osgPCL::PointCloudFactory* _factory);
        OutOfCoreOptions(const OutOfCoreOptions& options,const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY);

        bool init( const OutOfCoreOctree::Ptr & octree );

        void setDepth( boost::uint64_t depth, boost::uint64_t max_depth);
        bool depthIsSet();

        boost::uint64_t getDepth();
        boost::uint64_t getMaxDepth();

        bool isRoot();
        void setRoot(bool enable);



        float getSampling();
        void setSampling(float sample);

        void setBoundingBox(const osg::Vec3d & bbmin, const osg::Vec3d& bbmax);
        void getBoundingBox( osg::Vec3d & bbmin,       osg::Vec3d& bbmax);


        private:
          bool depth_set_;
          OutOfCoreOctree::Ptr octree_;
          osg::ref_ptr<osgPCL::PointCloudFactory> factory_;
          boost::uint64_t depth_;
          boost::uint64_t max_depth_;
          float sample_;
          osg::Vec3d bbmin_, bbmax_;
          bool isRoot_;
          bool isLeaf_;

        public:
          OutOfCoreOctree::Ptr getOctree(){return octree_;}
          osgPCL::PointCloudFactory * getFactory(){return factory_.get();}
          const osg::Vec3d& getBBmax(){return bbmax_;}
          const osg::Vec3d& getBBmin(){return bbmin_;}
          bool isLeaf(){return isLeaf_;}
          void setLeaf(bool enable){isLeaf_=enable;}

      };

  };

  }

  template<typename PointT>
  inline osgPCL::OutofCoreOctreeT<PointT>::OutofCoreOctreeT (
      const OctreePtr& octree)
  {
    octree_ = octree;
  }

  template<typename PointT>
  inline void osgPCL::OutofCoreOctreeT<PointT>::queryBBIncludes (
      const double min[3], const double max[3], size_t query_depth,
      const sensor_msgs::PointCloud2::Ptr& dst_blob) const
  {
    octree_->queryBBIncludes(min, max, query_depth, dst_blob);
  }

  template<typename PointT>
  inline void osgPCL::OutofCoreOctreeT<PointT>::queryBBIncludes_subsample (
      const double min[3], const double max[3], size_t query_depth,
      float subsample, const sensor_msgs::PointCloud2::Ptr& dst_blob) const
  {
    sensor_msgs::PointCloud2::Ptr rblob(new sensor_msgs::PointCloud2);

    if (subsample>0.999) {
      octree_->queryBBIncludes(min, max, query_depth, dst_blob);
      return;
    }
    octree_->queryBBIncludes(min, max, query_depth, rblob);

    std::vector<int> sub_indices;
    sub_indices.resize(rblob->width*rblob->height);

    boost::mt19937 rand_gen( std::time(NULL));
    boost::uniform_int < uint64_t > filedist (0, sub_indices.size() - 1);
    boost::variate_generator<boost::mt19937&, boost::uniform_int<uint64_t> > filedie (rand_gen, filedist);

    for(int i=0; i< sub_indices.size(); i++){
      sub_indices[i] = filedie();
    }
    pcl::copyPointCloud(*rblob, sub_indices, *dst_blob);
  }

  USE_OSGPLUGIN(oct_idx)

  /* namespace osgPCL */
#endif /* OUTOFCOREOCTREEREADER_H_ */
