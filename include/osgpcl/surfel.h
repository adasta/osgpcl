/*
 * surfel.h
 *
 *  Created on: Sep 4, 2012
 *      Author: asher
 */

#ifndef SURFEL_H_
#define SURFEL_H_

#include <osgpcl/point_cloud.h>

namespace osgpcl{
	template<typename PointT, typename  NormalT>
	class SurfelFactory : public  PointCloudFactory{
	public:
		SurfelFactory(float radius =0.1);
		~SurfelFactory(){}
		using PointCloudFactory::setInputCloud;

		virtual void setInputCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud);

		virtual PointCloudGeometry* buildGeometry(bool unique_state =false)const ;
		void setRadius(float radius);
	};
}

#endif /* SURFEL_H_ */
