#include <pcl/io/pcd_io.h>

#include <iostream>
#include <boost/program_options.hpp>

#include <osgpcl/point_cloud.h>

#include <osgViewer/Viewer>
#include <osgGA/TrackballManipulator>
#include <osg/ShapeDrawable>
#include <osg/PositionAttitudeTransform>
#include <osg/Geode>
#include <osg/Node>

namespace po=boost::program_options;

int main(int argc, char** argv){
  po::options_description desc("./osgpcl ");
  std::string infile;

  desc.add_options()
    ("help", "produce help message")
    ("input,i",po::value<std::string>(&infile)->required(), "input point cloud ")
    ;

  po::positional_options_description p;
  p.add("input",1);

  po::variables_map vm;
 try{
  po::store(po::command_line_parser(argc, argv).
  options(desc).positional(p).run(), vm);
  po::notify(vm);
 }
 catch( const std::exception& e)
 {
     std::cerr << e.what() << std::endl;
     std::cout << desc << std::endl;
     return 1;
 }

 pcl::PointCloud<pcl::PointXYZ> cloud;

 if ( pcl::io::loadPCDFile(infile, cloud) <0 )return -1;


 osgViewer::Viewer viewer;
 viewer.setUpViewInWindow(0,0,500,500,0);

 osg::Geode* geode = new osg::Geode;

 osgPCL::PointCloud* cren = new osgPCL::PointCloud;

 cren->setInputCloud(cloud);

 geode->addDrawable(cren);
 viewer.setSceneData( geode);

viewer.getCamera()->setClearColor( osg::Vec4(0,0,0,1));
return viewer.run();


return 0;
}
