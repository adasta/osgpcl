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

#include <boost/assign.hpp>
#include <boost/assign/std/vector.hpp>
using namespace boost::assign;

#include <osgpcl/pagedlod.h>

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

  void printBB( std::ostream& cout , OutofCoreOctreeReader::OutOfCoreOptions& opts){
    cout << " " << opts.getBBmin()[0] <<" " << opts.getBBmin()[1] << " " <<  opts.getBBmin()[2] << " to  ";
        std::cout <<   opts.getBBmax()[0] <<" " << opts.getBBmax()[1] << " " << opts.getBBmax()[2] << " \n";
  }

  osgDB::ReaderWriter::ReadResult OutofCoreOctreeReader::readNode (const std::string& fileName,
      const osgDB::ReaderWriter::Options* options) const
  {

    osg::ref_ptr< OutOfCoreOptions>   coptions = dynamic_cast< OutOfCoreOptions*>( const_cast<osg::Referenced*>(options->getUserData() ));

    if (coptions  != NULL){
      coptions = new OutOfCoreOptions( *coptions, osg::CopyOp::DEEP_COPY_ALL);
    }
    else{
      coptions = new OutOfCoreOptions(  new PointCloudColoredFactory<>);
    }
    if (coptions->getOctree() == NULL){
      if ( ! boost::filesystem::exists(fileName))  return osgDB::ReaderWriter::ReadResult::FILE_NOT_FOUND;
      OutofCoreOctreeT<pcl::PointXYZ>::OctreePtr ot (new OutofCoreOctreeT<pcl::PointXYZ>::Octree(fileName, false));
      OutofCoreOctreeT<pcl::PointXYZ>::Ptr tree (new OutofCoreOctreeT<pcl::PointXYZ>(ot));
      coptions->init( tree);
    }


    const osg::Vec3d & bbmin =coptions->getBBmin();
    const osg::Vec3d& bbmax =coptions->getBBmax();
    osg::Vec3d size = coptions->getBBmax() -coptions->getBBmin();
    osg::Vec3d sh = size/2;
    double radius = size.length();

    osg::ref_ptr<osgPCL::PagedLOD> lod = new osgPCL::PagedLOD;
    lod->setCenterMode( osg::LOD::USER_DEFINED_CENTER );
    osg::Vec3d center = (bbmax + bbmin)/2.0f ;
    lod->setCenter( center );
    lod->setRadius( radius );

    sensor_msgs::PointCloud2::Ptr cloud(new sensor_msgs::PointCloud2);
    if (coptions->getSampling() > 0.999){
      coptions->getOctree()->queryBBIncludes(coptions->getBBmin()._v, coptions->getBBmax()._v,coptions->getDepth(), cloud);
    }
    else{
      coptions->getOctree()->queryBBIncludes_subsample(coptions->getBBmin()._v, coptions->getBBmax()._v,coptions->getDepth(), coptions->getSampling(), cloud);
    }

    if (cloud->width*cloud->height == 0 ) return new osg::Node;
    coptions->getFactory()->setInputCloud(cloud);

    if(coptions->isRoot()){
     coptions->setRoot(false);
     lod->addChild(coptions->getFactory()->buildNode(), radius, FLT_MAX);
     }
    else{
      if (coptions->getDepth() == coptions->getMaxDepth()){
        lod->addChild(coptions->getFactory()->buildNode(), 0 , radius*3);
      }
      else  lod->addChild(coptions->getFactory()->buildNode(), radius , radius*3);
    }
    coptions->getFactory()->clearInput();

    std::vector<osg::Vec3d > minbbs;
    minbbs += bbmin, bbmin+ osg::Vec3d(sh[0],0,0), bbmin+ osg::Vec3d(sh[0],sh[1],0),
        bbmin+ osg::Vec3d(0,sh[1],0), bbmin+ osg::Vec3d(0, sh[1], sh[2]),
        bbmin+ osg::Vec3d(sh[0], sh[1], sh[2]),  bbmin+osg::Vec3d(sh[0], 0 , sh[2]),
        bbmin+osg::Vec3d(0, 0, sh[2]);

    float child_rad = sh.length();
     int cdepth = coptions->getDepth()+1;
    if (cdepth >= coptions->getMaxDepth()) return lod.get();

    for(int i=0; i<8; i++){
      //todo add some way to check the number of points within a bounding box without actually retrieving them
      OutOfCoreOptions* child_opts = new OutOfCoreOptions(*coptions, osg::CopyOp::DEEP_COPY_ALL);
      child_opts->setBoundingBox( minbbs[i],  minbbs[i]+sh);
      child_opts->setDepth(cdepth, coptions->getMaxDepth());
      osgDB::Options* opts = new osgDB::Options(*options, osg::CopyOp::DEEP_COPY_ALL);
      opts->setUserData(child_opts);

      lod->addChild( 0.0f,child_rad*3.0f, fileName,opts);
    }
    coptions = dynamic_cast< OutOfCoreOptions*>( const_cast<osg::Referenced*>(options->getUserData() ));


    return lod.get();
  }

  OutofCoreOctreeReader::OutOfCoreOptions::OutOfCoreOptions (float sample) : sample_ (sample),
      isRoot_(true),depth_(0), max_depth_(0), factory_(new osgPCL::PointCloudCRangeFactory<>), depth_set_(false),
      bbmin_(0,0,0),bbmax_(0,0,0)
  {
  }

  OutofCoreOctreeReader::OutOfCoreOptions::OutOfCoreOptions (
      osgPCL::PointCloudFactory*  factory, float sample) : sample_ (sample),
          isRoot_(true),depth_(0), max_depth_(0), factory_(factory), depth_set_(false),
          bbmin_(0,0,0),bbmax_(0,0,0)
  {
  }

  OutofCoreOctreeReader::OutOfCoreOptions::OutOfCoreOptions (
      const OutOfCoreOctree::Ptr& _octree, osgPCL::PointCloudFactory* _factory) : sample_ (1.0f),
          isRoot_(true),depth_(0), max_depth_(0), depth_set_(false), factory_(_factory),
          bbmin_(0,0,0),bbmax_(0,0,0)
  {
    this->init(octree_ );
  }

  bool OutofCoreOctreeReader::OutOfCoreOptions::init (const OutOfCoreOctree::Ptr& _octree )
  {
    if (!depth_set_){
      depth_ =0;
      max_depth_ = _octree->getTreeDepth();
      depth_set_ =true;
    }
    this->octree_ = _octree;
    if (bbmax_ == bbmin_){
      this->octree_->getBoundingBox(bbmin_._v, bbmax_._v);
    }
  }

  void OutofCoreOctreeReader::OutOfCoreOptions::setDepth (boost::uint64_t depth,
      boost::uint64_t max_depth)
  {
    depth_set_ = true;
    depth_ =depth;
    max_depth_ = max_depth;
  }

  bool OutofCoreOctreeReader::OutOfCoreOptions::depthIsSet ()
  {
    return depth_set_;
  }

  boost::uint64_t OutofCoreOctreeReader::OutOfCoreOptions::getDepth ()
  {
    return depth_;
  }

  boost::uint64_t OutofCoreOctreeReader::OutOfCoreOptions::getMaxDepth ()
  {
    return max_depth_;
  }

  bool OutofCoreOctreeReader::OutOfCoreOptions::isRoot ()
  {
    return isRoot_;
  }

  void OutofCoreOctreeReader::OutOfCoreOptions::setRoot (bool enable)
  {
    isRoot_ = enable;
  }

  float OutofCoreOctreeReader::OutOfCoreOptions::getSampling ()
  {
    return sample_;
  }

  void OutofCoreOctreeReader::OutOfCoreOptions::setSampling (float sample)
  {
    sample_ =sample;
  }

  void OutofCoreOctreeReader::OutOfCoreOptions::setBoundingBox (
      const osg::Vec3d& bbmin, const osg::Vec3d& bbmax)
  {
    bbmin_ = bbmin;
    bbmax_ = bbmax;
  }

  OutofCoreOctreeReader::OutOfCoreOptions::OutOfCoreOptions (
      const OutOfCoreOptions& options, const osg::CopyOp& copyop){
    this->bbmax_ = options.bbmax_;
    this->bbmin_ = options.bbmin_;
    this->isRoot_ = options.isRoot_;
    this->depth_ = options.depth_;
    this->max_depth_ = options.max_depth_;
    this->octree_ = options.octree_;
    this->factory_ = options.factory_;
    this->sample_ = options.sample_;
  }

  void OutofCoreOctreeReader::OutOfCoreOptions::getBoundingBox (
      osg::Vec3d& bbmin, osg::Vec3d& bbmax)
  {
    bbmin = bbmin_;
    bbmax = bbmax_;
  }

}


/* namespace osgPCL */
