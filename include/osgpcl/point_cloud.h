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

#include <boost/any.hpp>
#include <string>
#include <map>
#include <pcl/common/io.h>

#include <sensor_msgs/PointCloud2.h>

namespace osgpcl
{

  /*
   * The point cloud visualization must be initialized with how it should be rendered
   * and then the point cloud should be set.
   */


  typedef osg::Geometry PointCloudGeometry;

  class PointCloudFactory :  public osg::Referenced {
    public:
      PointCloudFactory();
      virtual ~PointCloudFactory(){}


      virtual PointCloudGeometry* buildGeometry(bool unique_stateset=false) const =0;

    //  void setInputIndices(const pcl::IndicesConstPtr& indices);

      virtual osg::Node * buildNode();

    private:
      std::map<std::string, boost::any> input_clouds_;

    public:
      template<typename PointT>
      void setInputCloud(const typename pcl::PointCloud<PointT>::ConstPtr& cloud ){
        input_clouds_[pcl::getFieldsList(*cloud)] = cloud;
      }

      virtual void setInputCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud)=0;
      void clearInput(){input_clouds_.clear();}

      void setIndices( const pcl::IndicesConstPtr& indices){
    	  indices_ = indices;
      }

    protected:
      osg::ref_ptr<osg::StateSet> stateset_;
      pcl::IndicesConstPtr indices_;

      template<typename PointT>
      typename pcl::PointCloud<PointT>::ConstPtr getInputCloud() const ;

      template<typename PointT>
      void addXYZToVertexBuffer( osg::Geometry&, const pcl::PointCloud<pcl::PointXYZ>& cloud) const;
  };


  template<typename PointT=pcl::PointXYZ>
  class PointCloudColoredFactory : public PointCloudFactory {

    public:

    PointCloudColoredFactory();

    virtual PointCloudGeometry* buildGeometry(bool unique_stateset=false) const;

    using PointCloudFactory::setInputCloud;
    virtual void setInputCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud);

      void setColor(float r, float g, float b, float alpha =1);

    private:
  };


  template<typename PointTXYZ=pcl::PointXYZ, typename PointTF=pcl::PointXYZ>
  class PointCloudCRangeFactory : public PointCloudFactory {

    public:

    PointCloudCRangeFactory(std::string field="");


    typedef boost::shared_ptr<PointCloudCRangeFactory<PointTXYZ, PointTF> > Ptr;

     typedef boost::shared_ptr<typename pcl::PointCloud<PointTXYZ>::ConstPtr > CloudConstPtr;

    void setField(std::string field);
    void setRange(double min, double max);
    void setColorTable( const std::vector<osg::Vec4>& table);

    //void useJETColorTable();
   // void useGreyColorTable();

    virtual PointCloudGeometry* buildGeometry(bool unique_stateset=false) const;
    void setPointSize(int size);
    virtual void setInputCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud);
    using PointCloudFactory::setInputCloud;

    protected:
      std::string field_name_;
      double min_range_, max_range_;
      std::vector<osg::Vec4> color_table_;
  };

  template<typename PointTXYZ=pcl::PointXYZ, typename RGBT=pcl::RGB>
  class PointCloudRGBFactory : public PointCloudFactory {
    public:
    virtual PointCloudGeometry* buildGeometry(bool unique_stateset=false) const;
    virtual void setInputCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud);
    using PointCloudFactory::setInputCloud;

  };

  template<typename PointTXYZ, typename IntensityT>
  class PointCloudIFactory : public PointCloudFactory {
    public:
      virtual PointCloudGeometry* buildGeometry(bool unique_stateset=false) const;
      virtual void setInputCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud);
      using PointCloudFactory::setInputCloud;

  };


  template<typename PointTXYZ, typename LabelT>
  class PointCloudLabelFactory : public PointCloudFactory{

    public:
    PointCloudLabelFactory();

    virtual PointCloudGeometry* buildGeometry(bool unique_stateset=false) const;

    using PointCloudFactory::setInputCloud;
    virtual void setInputCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud);


    typedef  std::map<uint32_t, osg::Vec4f> ColorMap;
    void setColorMap( const ColorMap& color_map);

    /*
     * If the color map does not have a color assigned to the label,
     * the factory generates a random set of colors
     */
    void enableRandomColoring(bool enable);

    private:
      std::map<uint32_t, osg::Vec4f> color_map_;
      bool random_coloring_;
  };


  template<typename PointTXYZ, typename NormalT>
  class PointCloudNormalFactory: public PointCloudFactory{

    public:
	  PointCloudNormalFactory();

    virtual PointCloudGeometry* buildGeometry(bool unique_stateset=false) const;

    using PointCloudFactory::setInputCloud;
    virtual void setInputCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud);

    void setColor(osg::Vec4f color);

    void setNormalLength(float length){ nlength = length;};

    private:
      osg::Vec4f color_;
      float nlength;
  };

}


/* namespace osgPCL */
#endif /* POINT_CLOUD_H_ */
