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
    ("color,C",  "Render point cloud using RGB field")
    ("range,r", po::value<std::string>()->default_value("z"), "Render point cloud using field range.  specify the field")
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
 if (vm.count("color")){
   options->setFactory( new osgpcl::PointCloudRGBFactory<pcl::PointXYZ, pcl::RGB>());
   pcl::console::print_info("Using RGB Field for Rendering...\n");
 }
 if (vm.count("range")){
	 std::string field = vm["range"].as<std::string>();

   options->setFactory( new osgpcl::PointCloudCRangeFactory<pcl::PointXYZ, pcl::PointXYZ>(field));
   pcl::console::print_info("Using %s for Rendering...\n", field.c_str());
 }


osgViewer::Viewer viewer;
viewer.setUpViewInWindow(0,0,500,500,0);

osgDB::Registry::instance()->addReaderWriter(new osgpcl::PointCloudReader);
osgDB::Registry::instance()->addReaderWriter(new osgpcl::OutofCoreOctreeReader);

osg::Group * group = new osg::Group;
for(int i=0; i< infiles.size(); i++) {
  pcl::console::print_info("Loading :  %s \n", infiles[i].c_str());
  group->addChild(osgDB::readNodeFile(infiles[i], options));
}
viewer.setSceneData(group);
viewer.getCamera()->setClearColor( osg::Vec4(0,0,0,1));
return viewer.run();

return 0;
}
