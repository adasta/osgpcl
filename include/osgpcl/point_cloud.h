/*
 * point_cloud.h
 *
 *  Created on: Jun 23, 2012
 *      Author: asher
 */

#ifndef POINT_CLOUD_H_
#define POINT_CLOUD_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <osg/Geometry>
#include <osg/Uniform>

namespace osgPCL
{

  /*
   * The point cloud visualization must be initialized with how it should be rendered
   * and then the point cloud should be set.
   */

  class PointCloud : public osg::Geometry
  {
  public:
    PointCloud ();
    virtual
    ~PointCloud ();



    //Color
    void setInputCloud( const pcl::PointCloud<pcl::PointXYZ>& cloud);


  private:
    static std::map<std::string, osg::Program*> shader_programs_;

    osg::Vec4 uniform_color_;
public:
    //Enables the solid color shader and sets the uniform color as
    //this RGB Value
    void setPointColor(double r, double g, double b);

  };




} /* namespace osgPCL */
#endif /* POINT_CLOUD_H_ */
