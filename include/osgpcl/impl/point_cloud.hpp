/*
 * point_cloud.hpp
 *
 *  Created on: Jul 27, 2012
 *      Author: Adam Stambler
 */

#ifndef POINT_CLOUD_HPP_
#define POINT_CLOUD_HPP_


#include <osgpcl/point_cloud.h>
#include <pcl/console/print.h>
#include <osg/Point>

#include <pcl/point_traits.h>
#include <pcl/common/concatenate.h>
#include <pcl/ros/conversions.h>
#include <pcl/common/common.h>

#include <boost/type_traits.hpp>


  template<typename PointT>
  inline typename pcl::PointCloud<PointT>::ConstPtr osgPCL::PointCloudFactory::getInputCloud () const
  {

    std::string key;    // Get the fields list
    {
      pcl::PointCloud<PointT> cloud;
      key = pcl::getFieldsList(cloud);
    }

    std::map<std::string, boost::any>::const_iterator iter= input_clouds_.find(key);

    if (iter == input_clouds_.end()){
      pcl::console::print_error("PointCloudFactory trying to retrieve input %s that does not exist\n", key.c_str());
      return typename pcl::PointCloud<PointT>::ConstPtr();
    }
    try{
      return boost::any_cast< typename pcl::PointCloud<PointT>::ConstPtr>(iter->second);
    }
    catch(boost::bad_any_cast& e){
      pcl::console::print_error("PointCloudFactory Exception: %s\n", e.what());
      return typename pcl::PointCloud<PointT>::ConstPtr();
    }

  }

  template<typename PointT>
  inline void osgPCL::PointCloudFactory::addXYZToVertexBuffer (osg::Geometry& geom,
      const pcl::PointCloud<pcl::PointXYZ>& cloud) const
  {
    osg::Vec3Array* pts = new osg::Vec3Array;
    pts->reserve(cloud.points.size());
    for(int i=0; i<cloud.points.size(); i++){
      const pcl::PointXYZ& pt = cloud.points[i];
      pts->push_back(osg::Vec3(pt.x, pt.y, pt.z));
    }
    geom.setVertexArray( pts );
    geom.addPrimitiveSet( new osg::DrawArrays( GL_POINTS, 0, pts->size() ) );
  }


  // *****************************  PointCloudColorFactory ******************

  template<typename PointT>
  inline osgPCL::PointCloudColoredFactory<PointT>::PointCloudColoredFactory ()
  {
    const char* vertSource = {
         "#version 120\n"
         "void main(void)\n"
         "{\n"
         "    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;\n"
         "}\n"
         };
         const char* fragSource = {
         "#version 120\n"
         "uniform vec4 color;\n"
         "void main(void)\n"
         "{\n"
         "    gl_FragColor = color;\n"
         "}\n"
         };

         osg::Program* pgm = new osg::Program;
         pgm->setName( "UniformColor" );

         pgm->addShader( new osg::Shader( osg::Shader::VERTEX,   vertSource ) );
         pgm->addShader( new osg::Shader( osg::Shader::FRAGMENT, fragSource ) );
         stateset_ = new osg::StateSet;
         stateset_->setAttribute(pgm);

         osg::Point* p = new osg::Point;
         p->setSize(4);

         stateset_->setAttribute(p);

         osg::Vec4 color;
         color[0]=color[1]=color[2]=color[3]=1;

         osg::Uniform* ucolor( new osg::Uniform( "color", color ) );
         stateset_->addUniform( ucolor );

         stateset_->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
  }

  template<typename PointT> osgPCL::PointCloudGeometry*
   osgPCL::PointCloudColoredFactory<PointT>::buildGeometry (bool unique_stateset) const
  {
      typename pcl::PointCloud<PointT>::ConstPtr cloud = getInputCloud<PointT>();
      if (cloud ==NULL) return NULL;

     PointCloudGeometry * geom = new PointCloudGeometry;
     this->addXYZToVertexBuffer<PointT>(*geom, *cloud);

     osg::ref_ptr<osg::StateSet> ss;
     if (unique_stateset){
       ss = new osg::StateSet(*stateset_);
     }
     else{
       ss= stateset_;
     }
     geom->setStateSet(stateset_);

     return geom;
  }

template<typename PointT> void
osgPCL::PointCloudColoredFactory<PointT>::setInputCloud (
    const sensor_msgs::PointCloud2::ConstPtr& cloud)
{
  typename pcl::PointCloud<PointT>::Ptr xyz(new pcl::PointCloud<PointT>);
  pcl::fromROSMsg(*cloud,*xyz);
  PointCloudFactory::setInputCloud<PointT>(xyz);
}

  template<typename PointT> void
  osgPCL::PointCloudColoredFactory<PointT>::setColor (float r,float g,float b,
                                                            float alpha)
  {
    osg::Vec4 color;
    color[0]=r; color[1]=g; color[2]=b; color[3]=alpha;
    stateset_->getUniform("color")->set(color);
  }


// *************************************** PointCloudRGBFactory *************************
  template<typename PointTXYZ, typename RGBT> osgPCL::PointCloudGeometry*
  osgPCL::PointCloudRGBFactory<PointTXYZ, RGBT>::buildGeometry (
      bool unique_stateset) const
  {
    osgPCL::PointCloudGeometry* geom(new PointCloudGeometry);


    return geom;
  }


  template<typename PointTXYZ , typename RGBT>
  inline void osgPCL::PointCloudRGBFactory<PointTXYZ, RGBT>::setInputCloud (
      const sensor_msgs::PointCloud2::ConstPtr& cloud)
  {
    typename pcl::PointCloud<PointTXYZ>::Ptr xyz(new pcl::PointCloud<PointTXYZ>);
    pcl::fromROSMsg(*cloud,*xyz);
    PointCloudFactory::setInputCloud<PointTXYZ>(xyz);

    if ( !boost::is_same<PointTXYZ, RGBT>::value){
      typename pcl::PointCloud<RGBT>::Ptr rgb(new pcl::PointCloud<RGBT>);
      pcl::fromROSMsg(*cloud,*rgb);
      PointCloudFactory::setInputCloud<RGBT>(rgb);
    }
  }


  // ******************************* PointCloudCRange *************************


template<typename PointTXYZ , typename PointTF > inline
osgPCL::PointCloudCRangeFactory<PointTXYZ, PointTF>::PointCloudCRangeFactory () :
  max_range_(-1), min_range_(-1)
{
  color_table_.push_back(osg::Vec4(1,1,1,1));
  color_table_.push_back(osg::Vec4(1,0,0,1));
  stateset_ = new osg::StateSet;
  setPointSize(4);
  stateset_->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
}

template<typename PointTXYZ, typename PointTF> void
osgPCL::PointCloudCRangeFactory<PointTXYZ, PointTF>::setField (
    std::string field)
{
  field_name_ =field;
}

template<typename PointTXYZ , typename PointTF>
inline void osgPCL::PointCloudCRangeFactory<PointTXYZ, PointTF>::setRangle (
    double min, double max)
{
  min_range_ =min;
  max_range_ = max;
}

template<typename PointTXYZ, typename PointTF >
inline void osgPCL::PointCloudCRangeFactory<PointTXYZ, PointTF>::setColorTable (
    const std::vector<osg::Vec4>& table)
{
  color_table_ =table;
}

template<typename PointTXYZ , typename PointTF >
inline void osgPCL::PointCloudCRangeFactory<PointTXYZ, PointTF>::useJETColorTable ()
{
  //TODO
}

template<typename PointTXYZ , typename PointTF>
inline void osgPCL::PointCloudCRangeFactory<PointTXYZ, PointTF>::useGreyColorTable ()
{
  //TODO
}

template<typename PointTXYZ, typename PointTF> osgPCL::PointCloudGeometry*
osgPCL::PointCloudCRangeFactory<PointTXYZ, PointTF>::buildGeometry (
      bool unique_stateset) const
{

  typename pcl::PointCloud<PointTXYZ>::ConstPtr xyz = getInputCloud<PointTXYZ>();
   typename pcl::PointCloud<PointTF>::ConstPtr fcloud = getInputCloud<PointTF>();
  if  ( (fcloud == NULL) || (xyz==NULL) ){
    return NULL;
  }
  double minr, maxr;

  std::vector<sensor_msgs::PointField> flist;
  pcl::getFields<PointTF>(*fcloud, flist);

  int idx=-1;

  if (field_name_.empty()){
    idx=0;
  }
  else {
    for(int i=0; i< flist.size(); i++){
      if (flist[i].name==field_name_)  { idx=i; break;}
    }
  }

 if (idx <0 ){
   pcl::console::print_debug("[PointCloudCRangefactory] Pointfield ( %s )does not exist\n", field_name_.c_str());
   return NULL;
 }
 int offset =flist[idx].offset;

 if ( fabs(min_range_-max_range_) <0.001){
    minr = std::numeric_limits<double>::infinity();
    maxr = -std::numeric_limits<double>::infinity();
    for(int i=0; i< fcloud->points.size(); i++){
      double val = *( (float*) (  ( (uint8_t* ) &fcloud->points[i] )  + offset ) );
      if (val < minr) minr = val;
      if (val > maxr) maxr = val;
    }
  }
  else{
    minr = min_range_;
    maxr = max_range_;
  }
  double scale = (color_table_.size()-1)/(maxr-minr);
  int maxidx= color_table_.size()-1;

  osg::Vec4Array* colors = new osg::Vec4Array;
  colors->resize(fcloud->points.size());
  for(int i=0; i< fcloud->points.size(); i++){
   double val = *( (float*) (  ( (uint8_t* ) &fcloud->points[i] )  + offset ) );
   double idx = (val-minr)*scale;
   if (idx <0) idx=0;
   if (idx> maxidx)  idx =maxidx;
   double wl = idx-std::floor(idx);
   double wu = 1-wl;
   const osg::Vec4f& lpt = color_table_[std::floor(idx)];
   const osg::Vec4f& upt = color_table_[std::ceil(idx)];
   for(int j=0; j<4; j++) (*colors)[i][j] = lpt[j]*wl+ upt[j]*wu;
  }

  PointCloudGeometry* geom = new PointCloudGeometry;

  this->addXYZToVertexBuffer<PointTXYZ>(*geom, *xyz);

  geom->setColorArray(colors);
  geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

  if (unique_stateset){
    geom->setStateSet(new osg::StateSet(*stateset_));
  }
  else{
    geom->setStateSet(stateset_);
  }

  return geom;
}

template<typename PointTXYZ , typename PointTF > void
osgPCL::PointCloudCRangeFactory<PointTXYZ, PointTF>::setPointSize (
      int size)
  {
    osg::Point* p = new osg::Point();
    p->setSize(4);
    stateset_->setAttribute(p);
  }

template<typename PointTXYZ , typename PointTF >
inline void osgPCL::PointCloudCRangeFactory<PointTXYZ, PointTF>::setInputCloud (
    const sensor_msgs::PointCloud2::ConstPtr& cloud)
{
  typename  pcl::PointCloud<PointTXYZ>::Ptr xyz(new pcl::PointCloud<PointTXYZ>);
  pcl::fromROSMsg(*cloud,*xyz);
  PointCloudFactory::setInputCloud<PointTXYZ>(xyz);

  for(int i=0; i<cloud->fields.size(); i++) std::cout << i << "  " << cloud->fields[i].name << "\n";
  if ( !boost::is_same<PointTXYZ, PointTF>::value){
    typename pcl::PointCloud<PointTF>::Ptr fcloud(new pcl::PointCloud<PointTF>);
    pcl::fromROSMsg(*cloud,*fcloud);
    PointCloudFactory::setInputCloud<PointTF>(fcloud);
  }
}

  //**************************************** Intensity Point Cloud *******************

template<typename PointTXYZ, typename IntensityT>  osgPCL::PointCloudGeometry*
osgPCL::PointCloudIFactory<PointTXYZ, IntensityT>::buildGeometry () const
  {
  }


template<typename PointTXYZ, typename IntensityT>
inline void osgPCL::PointCloudIFactory<PointTXYZ, IntensityT>::setInputCloud (
    const sensor_msgs::PointCloud2::ConstPtr& cloud)
{
  typename pcl::PointCloud<PointTXYZ>::Ptr xyz(new pcl::PointCloud<PointTXYZ>);
  pcl::fromROSMsg(*cloud,*xyz);
  PointCloudFactory::setInputCloud<PointTXYZ>(xyz);

  if ( !boost::is_same<PointTXYZ, IntensityT>::value){
    typename  pcl::PointCloud<IntensityT>::Ptr icloud(new pcl::PointCloud<IntensityT>);
    pcl::fromROSMsg(*cloud,*icloud);
    PointCloudFactory::setInputCloud<IntensityT>(icloud);
  }
}


#endif /* POINT_CLOUD_HPP_ */
