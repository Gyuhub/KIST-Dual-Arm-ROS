#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <pthread.h>

using namespace std;

static tf::TransformBroadcaster *br_ptr_;

void ObjectPositionCallBack(const std_msgs::String::ConstPtr& msgs)
{
    cout << "Receive!\n";
    ROS_INFO_STREAM(msgs->data);
}

void *TfRoutine(void *nh)
{
    ros::NodeHandle nh_ = *(ros::NodeHandle *)nh;
    ros::Subscriber sub_ = nh_.subscribe("/object_position", 1, ObjectPositionCallBack);
    // ros::Rate rate(0);
    while (ros::ok())
    {
        static tf::TransformBroadcaster br_;
        br_ptr_ = &br_;
        tf::Transform transform_;
        transform_.setOrigin(tf::Vector3(0.0, 2.0, 0.0));
        tf::Quaternion q_;
        q_.setRPY(10.0 * (3.14 / 180.0), 10.0  * (3.14 / 180.0), 20.0  * (3.14 / 180.0));
        transform_.setRotation(q_);
        br_ptr_->sendTransform(tf::StampedTransform(transform_, ros::Time::now(), "base_link", "LWrR_Link"));
        // ROS_INFO("Send! x:[%lf],y:[%lf],z:[%lf],qx:[%lf],qy:[%lf],qz:[%lf],qw:[%lf]\n", transform_.getOrigin().x(), transform_.getOrigin().y(), transform_.getOrigin().z(),
        //                     transform_.getRotation().x(), transform_.getRotation().y(), transform_.getRotation().z(), transform_.getRotation().w());
        // rate.sleep();
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "tf_test");
    ros::NodeHandle nh_;
    pthread_t pth_;

    int iret = 0;
    pthread_create(&pth_, NULL, &TfRoutine, (void *)&nh_);
    pthread_join(pth_, 0);
    ros::spin();
}