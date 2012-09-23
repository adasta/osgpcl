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
template<typename PointT, typename  NormalT, typename RadiusT>
	class SurfelFactoryFF : public  PointCloudFactory{
	public:
		SurfelFactoryFF();
		~SurfelFactoryFF(){}
		using PointCloudFactory::setInputCloud;

		virtual void setInputCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud);

		virtual PointCloudGeometry* buildGeometry(bool unique_state =false)const ;
	private:
		Eigen::MatrixXf circle_cache;
	};

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

	template<typename PointT, typename  NormalT, typename IntensityT>
		class SurfelFactoryI : public  PointCloudFactory{
		public:
			SurfelFactoryI(float radius =0.01);
			~SurfelFactoryI(){}
			using PointCloudFactory::setInputCloud;

			virtual void setInputCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud);

			virtual PointCloudGeometry* buildGeometry(bool unique_state =false)const ;
			void setRadius(float radius);
		};

	template<typename PointT, typename  NormalT, typename IntensityT>
		class SurfelFactoryFFI : public  PointCloudFactory{
		public:
			SurfelFactoryFFI(float radius =0.01);
			~SurfelFactoryFFI(){}
			using PointCloudFactory::setInputCloud;

			virtual void setInputCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud);

			virtual PointCloudGeometry* buildGeometry(bool unique_state =false)const ;
			void setRadius(float radius);
		private:
			float radius_;
			Eigen::MatrixXf circle_cache;
		};

}

#endif /* SURFEL_H_ */
