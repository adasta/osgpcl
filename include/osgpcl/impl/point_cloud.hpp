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


  template<typename PointT>
  inline typename pcl::PointCloud<PointT>::ConstPtr osgPCL::PointCloudFactory::getInputCloud () const
  {
    typename pcl::PointCloud<PointT>::ConstPtr cloud;
    std::string key =pcl::getFieldsList(*cloud);
    std::map<std::string, boost::any>::const_iterator iter= input_clouds_.find(key);

    if (iter == input_clouds_.end()){
      return typename pcl::PointCloud<PointT>::ConstPtr();
    }
    try{
      cloud = boost::any_cast< typename pcl::PointCloud<PointT>::ConstPtr>(*iter);
      return cloud;
    }
    catch(boost::exception& e){
      pcl::console::print_error("PointCloudFactory trying to retrieve input %s that does not exist", key.c_str());
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

  }


  // ******************************* PointCloudCRange *************************


  template<typename PointTXYZ, typename PointTF>
  inline void osgPCL::PointCloudCRangeFactory<PointTXYZ, PointTF>::setField (
      std::string field)
  {
  }

  template<typename PointTXYZ , typename PointTF>
  inline void osgPCL::PointCloudCRangeFactory<PointTXYZ, PointTF>::setRangle (
      double min, double max)
  {
  }

  template<typename PointTXYZ, typename PointTF >
  inline void osgPCL::PointCloudCRangeFactory<PointTXYZ, PointTF>::setColorTable (
      const std::vector<osg::Vec4>& table)
  {
  }

  template<typename PointTXYZ , typename PointTF >
  inline void osgPCL::PointCloudCRangeFactory<PointTXYZ, PointTF>::useJETColorTable ()
  {
  }

  template<typename PointTXYZ , typename PointTF>
  inline void osgPCL::PointCloudCRangeFactory<PointTXYZ, PointTF>::useGreyColorTable ()
  {
  }

  template<typename PointTXYZ, typename PointTF>
  inline osgPCL::PointCloudGeometry* osgPCL::PointCloudCRangeFactory<PointTXYZ, PointTF>::buildGeometry (
      bool unique_stateset) const
  {
  }

  template<typename PointTXYZ , typename PointTF >
  inline void osgPCL::PointCloudCRangeFactory<PointTXYZ, PointTF>::setPointSize (
      int size)
  {
  }

  template<typename PointTXYZ, typename IntensityT>
  inline osgPCL::PointCloudGeometry* osgPCL::PointCloudIFactory<PointTXYZ, IntensityT>::buildGeometry () const
  {
  }


#endif /* POINT_CLOUD_HPP_ */
