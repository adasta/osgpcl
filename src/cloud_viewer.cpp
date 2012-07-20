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

#include <osg/Array>
#include <osgUtil/PolytopeIntersector>
#include <osgGA/GUIEventHandler>
#include <osgGA/GUIActionAdapter>

namespace po=boost::program_options;


osg::Camera* createHUDCamera( double left, double right, double bottom, double top )
{
    osg::ref_ptr<osg::Camera> camera = new osg::Camera;
    camera->setReferenceFrame( osg::Transform::ABSOLUTE_RF );
    camera->setClearMask( GL_DEPTH_BUFFER_BIT );
    camera->setRenderOrder( osg::Camera::POST_RENDER );
    camera->setAllowEventFocus( false );
    camera->setProjectionMatrix( osg::Matrix::ortho2D(left, right, bottom, top) );
    camera->getOrCreateStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
    return camera.release();
}

class Outline2D {
  public :

    Outline2D();

    void addpoint( const osg::Vec2f& pt );
    void changeLastPoint(const osg::Vec2f& pt);
    void clear();


  private:
    osg::ref_ptr<osg::Geometry> geom_;
  public:
    osg::Geometry* getGeometry(){return geom_.get();}

    void setColor(double r, double g, double b);



    osg::Vec3Array* getPoints(){
      return dynamic_cast<osg::Vec3Array*> (geom_->getVertexArray()->asArray());
    }

    void update(){
      osg::Vec3Array* pts = getPoints();
      if (geom_->getNumPrimitiveSets() ) geom_->removePrimitiveSet(0);
     if (pts->size()) geom_->addPrimitiveSet(new osg::DrawArrays( GL_LINE_STRIP, 0, pts->size() ));
    }

};

Outline2D::Outline2D(){
  geom_ = new osg::Geometry;
  geom_->setDataVariance(osg::Node::DYNAMIC);
  osg::Vec3Array* pts = new osg::Vec3Array;
  geom_->setVertexArray(pts);
  geom_->addPrimitiveSet(new osg::DrawArrays( GL_LINE_STRIP, 0, pts->size() ));

  osg::Vec3Array* color = new osg::Vec3Array;
  color->push_back(osg::Vec3f(1,1,1));

  geom_->setColorBinding( osg::Geometry::BIND_OVERALL);
  geom_->setColorArray(color);
}

void
Outline2D::addpoint(const osg::Vec2f& pt){
  osg::Vec3Array* pts = getPoints();
  pts->push_back(osg::Vec3f(pt[0],pt[1], 0));
  std::cout << " Adding " << pt[0] <<  "  " << pt[1] << " pts\n";

  std::cout << "There are " << pts->size() << " pts\n";
  pts->dirty();
}

void
Outline2D::clear(){
  osg::Vec3Array* pts =getPoints();
  pts->clear();
  pts->dirty();
}

void
Outline2D::changeLastPoint(const osg::Vec2f& pt){
  osg::Vec3Array* pts =getPoints();
  if (pts ==NULL) return;
  pts->pop_back();
  pts->push_back(osg::Vec3f(pt[0],pt[1], 0));
  pts->dirty();
}


// class to handle events with a pick
class PickHandler : public osgGA::GUIEventHandler {
public:
    PickHandler( );

    ~PickHandler() {}

    bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa);

    void handleMouseMove(osgGA::GUIEventAdapter& ea);

    void select(osg::View* view, osgViewer::ViewerBase* viewer  );

protected:
    bool enable_selection_;
    Outline2D outline_;
    osg::ref_ptr<osg::Group> rubber_band_;
};

PickHandler::PickHandler(): enable_selection_(false){
  rubber_band_ = new osg::Group;

  osg::Geode* rubber_g = new osg::Geode;
  rubber_g->setDataVariance(osg::Node::DYNAMIC);
  osg::Camera* hud = createHUDCamera(-1,1,-1,1);
  rubber_band_->addChild(hud);
  hud->addChild(rubber_g);
  rubber_g->addDrawable(outline_.getGeometry());
}


bool
PickHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa){
  switch(ea.getEventType())
  {
      case(osgGA::GUIEventAdapter::PUSH):
      {
          if (!enable_selection_) return false;

          osgViewer::View* view = dynamic_cast<osgViewer::View*>(&aa);


          outline_.addpoint( osg::Vec2f(ea.getXnormalized(), ea.getYnormalized()));
          outline_.update();

         // if (view) pick(view,ea);
          return true;
      }
      case  osgGA::GUIEventAdapter::MOVE:
      {

        if (!enable_selection_) return false;
        if ( outline_.getPoints()->size() ==0) return false;
        if (outline_.getPoints()->size() ==1 ){
          outline_.addpoint( osg::Vec2f(ea.getXnormalized(), ea.getYnormalized()));
        }else{
          outline_.changeLastPoint( osg::Vec2f(ea.getXnormalized(), ea.getYnormalized()));
        }
        outline_.update();
        return true;
      }
      case(osgGA::GUIEventAdapter::RELEASE):
        {
        if (!enable_selection_) return false;

        break;
      }
      case(osgGA::GUIEventAdapter::KEYDOWN):
      {
          if (ea.getKey()=='s')
          {
            osgViewer::View* view = dynamic_cast<osgViewer::View*>(&aa);
            enable_selection_ = !enable_selection_;
            std::clog << "Selection : " << enable_selection_ << " \n";

            if (!enable_selection_){
              osgViewer::ViewerBase* viewer =
                      dynamic_cast<osgViewer::ViewerBase*>( &aa );
              select(view, viewer);
              outline_.clear();
              outline_.update();
              osg::Group* root = dynamic_cast<osg::Group*>(view->getSceneData());
              root->removeChild(rubber_band_);
            }
            else{
              osg::Group* root = dynamic_cast<osg::Group*>(view->getSceneData());
              root->addChild(rubber_band_);
            }
              return true;
          }
      }
      default:
          if(enable_selection_) return true;
          return false;
  }
}

void
PickHandler::select(osg::View* view, osgViewer::ViewerBase* viewer){

  osg::Polytope polytope;

  osg::Polytope::VertexList vertices;

  osg::Vec3Array* pts = outline_.getPoints();

  if (pts->size() ==0){
    std::cout << "There are no points \n";
    return;
  }
  for(int i=0; i<pts->size(); i++){
      vertices.push_back((*pts)[i]);
  }

  osg::Polytope::PlaneList pl;
  pl.pu

  polytope.setReferenceVertexList(vertices);

  osgUtil::PolytopeIntersector* picker =
          new osgUtil::PolytopeIntersector(
              osgUtil::Intersector::WINDOW, polytope );
  picker->setDimensionMask(osgUtil::PolytopeIntersector::DimZero);
  osgUtil::IntersectionVisitor iv( picker );
  view->getCamera()->accept( iv );



  if (picker->containsIntersections()){
    std::cout << "There are interesctions   "  << picker->getIntersections().size() << " \n";
    for(osgUtil::PolytopeIntersector::Intersections::iterator hitr = picker->getIntersections().begin();
          hitr != picker->getIntersections().end();
          ++hitr)
      {
          if (!hitr->nodePath.empty() && !(hitr->nodePath.back()->getName().empty()))
          {
              // the geodes are identified by name.
              std::cout <<"Object \""<<hitr->nodePath.back()->getName()<<"\""<<std::endl;
          }
          else if (hitr->drawable.valid())
          {
              std::cout <<"Object \""<<hitr->drawable->className()<<"\""<<std::endl;
          }
          }
      }
}


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
 cren->setPointColor(0,1,0.25);

 cren->setInputCloud(cloud);
 geode->addDrawable(cren);
 geode->setDataVariance(osg::Node::STATIC);

 osg::Group* root = new osg::Group;
 root->addChild(geode);
 viewer.setSceneData( root);
 viewer.addEventHandler(new PickHandler);

viewer.getCamera()->setClearColor( osg::Vec4(0,0,0,1));
return viewer.run();


return 0;
}
