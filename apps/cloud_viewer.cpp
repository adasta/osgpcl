#include <osgpcl/common.h>

#include <osgpcl/point_cloud_reader.h>
#include <osgpcl/outofcore_octree_reader.h>

#include <osgViewer/Viewer>
#include <osgDB/ReadFile>

#include <iostream>
#include <pcl/console/print.h>

#include <boost/program_options.hpp>
namespace po=boost::program_options;


int main(int argc, char** argv){
  po::options_description desc("./cloud_viewer [options] input ... ");

  std::vector<std::string> infiles;

  desc.add_options()
    ("help", "produce help message")
    ("input,i",po::value<std::vector<std::string> >(&infiles)->multitoken()->required(), "input point cloud ")
    ("sampling_rate,s", po::value<float>(), "randomly subsample the input cloud")
    ;

  po::positional_options_description p;
  p.add("input",-1);

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

 osg::ref_ptr< osgpcl::CloudReaderOptions>  options = new osgpcl::CloudReaderOptions;

 if( vm.count("sampling_rate")){
   options->setSamplingRate(vm["sampling_rate"].as<float>() );
 }

osgViewer::Viewer viewer;
viewer.setUpViewInWindow(0,0,500,500,0);

osgDB::Registry::instance()->addReaderWriter(new osgpcl::PointCloudReader);
osgDB::Registry::instance()->addReaderWriter(new osgpcl::OutofCoreOctreeReader);


for(int i=0; i< infiles.size(); i++) {
  pcl::console::print_info("Loading :  %s \n", infiles[i].c_str());
  pcl::PCDReader reader;
  sensor_msgs::PointCloud2 header;
  if ( reader.readHeader(infiles[i], header) >= 0)
    options->setFactory( osgpcl::chooseDefaultRepresentation( header.fields));
  viewer.setSceneData( osgDB::readNodeFile(infiles[i], options));
}
viewer.getCamera()->setClearColor( osg::Vec4(0,0,0,1));
return viewer.run();

return 0;
}
