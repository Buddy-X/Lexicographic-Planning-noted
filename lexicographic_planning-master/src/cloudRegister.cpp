#include "utility.h"

#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

#include <Eigen/Core>
#include <Eigen/Dense>

class CloudRegister : public ParamServer
{

public:

    ros::Publisher pubLaserCloud;
    ros::Subscriber subLaserCloud;

    tf::TransformListener listener;
    tf::StampedTransform transform;
    tf::StampedTransform transform_to_map;


    bool newCloudFlag;
    std_msgs::Header cloudHeader;

    pcl::PointCloud<PointType>::Ptr laserCloudIn;
    

    std::shared_ptr<pcl::VoxelGrid<PointType>> downSizeFilter;

    CloudRegister()
    {
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(_pointCloudTopic, 1, &CloudRegister::cloudHandler, this);
        pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("planning/registered_cloud", 1);

        newCloudFlag = false;
        laserCloudIn.reset(new pcl::PointCloud<PointType>());
        downSizeFilter = std::make_shared<pcl::VoxelGrid<PointType>>();
        downSizeFilter->setLeafSize(_mapResolution, _mapResolution, _mapResolution);
    }

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        if (pubLaserCloud.getNumSubscribers() == 0)
            return;

        try{
            listener.waitForTransform("map", "base_link", laserCloudMsg->header.stamp, ros::Duration(0.5));
            listener.lookupTransform("map", "base_link", laserCloudMsg->header.stamp, transform);

            listener.lookupTransform("map", ros::Time(0), "base_link", laserCloudMsg->header.stamp, "map", transform_to_map);
            std::cout << transform_to_map.getOrigin().x() << std::endl;
            std::cout << transform_to_map.getOrigin().y() << std::endl;
        } 
        catch (tf::TransformException ex){
            return;
        }

        cloudHeader = laserCloudMsg->header;

        pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);
    
        newCloudFlag = true;
    }

    void run()
    {
        if (newCloudFlag == false)
        {
            // publish empty cloud to keep the program running
            pcl::PointCloud<PointType>::Ptr emptyCloud(new pcl::PointCloud<PointType>());
            publishCloud(&pubLaserCloud, emptyCloud, cloudHeader.stamp, "map");
            return;
        }
        // get robot position
        PointType robotPoint;
        robotPoint.x = transform.getOrigin().x();
        robotPoint.y = transform.getOrigin().y();
        robotPoint.z = transform.getOrigin().z();

        // // transform cloud
        laserCloudIn->header.frame_id = "base_link";
        laserCloudIn->header.stamp = 0;
        pcl::PointCloud<PointType>::Ptr laserCloudInGlobal(new pcl::PointCloud<PointType>());
        Eigen::Quaterniond q(transform_to_map.getRotation().w(), transform_to_map.getRotation().x(), transform_to_map.getRotation().y(), transform_to_map.getRotation().z());
        Eigen::Vector3d t(transform_to_map.getOrigin().x(), transform_to_map.getOrigin().y(), transform_to_map.getOrigin().z());
        Eigen::Isometry3d trans_mat = Eigen::Isometry3d::Identity();
        trans_mat.translate(t);
        trans_mat.rotate(q);
        // std::cout << "trans_mat:  \n" << trans_mat.matrix() << std::endl;
        pcl::transformPointCloud(*laserCloudIn, *laserCloudInGlobal, trans_mat.cast<float>());
        // pcl_ros::transformPointCloud("map", *laserCloudIn, *laserCloudInGlobal, listener);
        // down-sample cloud
        pcl::PointCloud<PointType>::Ptr laserCloudInDS(new pcl::PointCloud<PointType>());
        downSizeFilter->setInputCloud(laserCloudInGlobal);

        downSizeFilter->filter(*laserCloudInDS);
        // filter cloud
        pcl::PointCloud<PointType>::Ptr laserCloudFiltered(new pcl::PointCloud<PointType>());
        for (int i = 0; i < laserCloudInDS->size(); ++i)
        {
            PointType p;
            p.x = laserCloudInDS->points[i].x;
            p.y = laserCloudInDS->points[i].y;
            p.z = laserCloudInDS->points[i].z;
            p.intensity = laserCloudInDS->points[i].intensity;

            if (p.z > _sensorHeightLimitUpper + robotPoint.z || p.z < _sensorHeightLimitDown + robotPoint.z)
                continue;

            float range = pointDistance(p, robotPoint);
            if (range < _sensorRangeLimitMin || range > _sensorRangeLimitMax)
                continue;

            laserCloudFiltered->push_back(p);
        }

        // publish filtered cloud
        publishCloud(&pubLaserCloud, laserCloudFiltered, cloudHeader.stamp, "map");

        newCloudFlag = false;
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "lexicographic_planning");
    
    ROS_INFO("\033[1;32m----> lexicographic_planning: Cloud Register Started.\033[0m");

    CloudRegister cr;

    ros::Rate r(5);
    while (ros::ok())
    {
        ros::spinOnce();

        cr.run();

        r.sleep();
    }

    return 0;
}