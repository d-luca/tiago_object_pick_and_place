#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

class PointCloudNode{
private:
    ros::NodeHandle node_;
    //projector used to transform laser scans into points in the map
    laser_geometry::LaserProjection projector_;
    tf::TransformListener tfListener_;

    ros::Publisher publisher_;
    ros::Subscriber subscriber_;
public:
    PointCloudNode(){
        //subscribe to the /scan topic for LaserScan messages
        //get the subscriber to shut down the subscription
        subscriber_ = node_.subscribe<sensor_msgs::LaserScan> ("/scan", 100, &PointCloudNode::scanCallback, this);
        //advertise the new topic /obstacles_cloud
        //get the publisher to publish PointCloud2 messages or shut down the topic
        publisher_ = node_.advertise<sensor_msgs::PointCloud2> ("/intro_navigation/scan_cloud", 100, false);
    }
    // when a laser scan is published in /scan this is called
    void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
    {
        if(!tfListener_.waitForTransform(scan_in->header.frame_id,
                                       "/map",
                                       scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),ros::Duration(1.0)))
        {return;}
        sensor_msgs::PointCloud2 cloud;
        projector_.transformLaserScanToPointCloud("/map", *scan_in, cloud, tfListener_);
        //publish the map
        publisher_.publish(cloud);
    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "scan_cloud");
    //creates the node for the pointCloud and keep it working
    PointCloudNode node;
    ROS_INFO("Node started -- converting from /scan into CloudPoint2 /intro_navigation/scan_cloud w.r.t. map frame");
    ros::spin();
}