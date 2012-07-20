/*
 * point_cloud.cpp
 *
 *  Created on: Jun 23, 2012
 *      Author: asher
 */

#include <osgpcl/point_cloud.h>
#include <osg/Point>

std::map<std::string, osg::Program*> osgPCL::PointCloud::shader_programs_;


namespace osgPCL
{

  PointCloud::PointCloud ()
  {
    uniform_color_.set(1,1,1,1);

    if (!shader_programs_.count("UniformColor") ){
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
      shader_programs_["UniformColor"] = pgm;
    }

  }

  PointCloud::~PointCloud ()
  {
  }

  void
  osgPCL::PointCloud::setPointColor(double r, double g, double b){
    uniform_color_[0]=r; uniform_color_[1]=g; uniform_color_[2]=b;
    osg::StateSet* sset = getOrCreateStateSet();
    if ( sset->getUniform("color") != NULL ){
      sset->setAttribute(shader_programs_["UniformColor"]);
      osg::Uniform* ucolor = sset->getUniform("color");
      ucolor->set(uniform_color_);
      ucolor->dirty();
    }
  }

  void
 osgPCL::PointCloud::setInputCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud){
   osg::Vec3Array* pts = new osg::Vec3Array;
   pts->reserve(cloud.points.size());
   for(int i=0; i<cloud.points.size(); i++){
     const pcl::PointXYZ& pt = cloud.points[i];
     pts->push_back(osg::Vec3(pt.x, pt.y, pt.z));
   }
   setVertexArray( pts );
   this->addPrimitiveSet( new osg::DrawArrays( GL_POINTS, 0, pts->size() ) );

   osg::StateSet* sset = getOrCreateStateSet();
   sset->setMode( GL_LIGHTING, osg::StateAttribute::OFF );

   // if things go wrong, fall back to big points
   osg::Point* p = new osg::Point;
   p->setSize(6);
   sset->setAttribute( p );
   sset->setAttribute(shader_programs_["UniformColor"]);
   osg::Uniform* ucolor( new osg::Uniform( "color", uniform_color_ ) );
   sset->addUniform( ucolor );
 }

} /* namespace osgPCL */
