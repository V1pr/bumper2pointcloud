/**
 * @file /src/bumper2pointcloud.cpp
 *
 * @brief Bumper to pointcloud node class implementation.
 *
 * @author Tamas DAJKA, work based on Jorge Santos @ Yujin Robot's work
 *
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <bumper2pointcloud/bumper2pointcloud.hpp>

#include <pluginlib/class_list_macros.h>

namespace bumper2pointcloud
{
    
  Bumper2PointcloudNode::Bumper2PointcloudNode() : P_INF_X(+100*sin(0.34906585)),
        P_INF_Y(+100*cos(0.34906585)),
        N_INF_Y(-100*cos(0.34906585)),
        ZERO(0), prev_rbump(0), prev_lbump(0)
  {
    ros::NodeHandle nh;

    // Bumper pointcloud distance to base frame; should be something like the robot radius plus
    // costmap resolution plus an extra to cope with robot inertia. This is a bit tricky parameter: if
    // it's too low, costmap will ignore this pointcloud (the robot footprint runs over the hit obstacle),
    // but if it's too big, hit obstacles will be mapped too far from the robot and the navigation around
    // them will probably fail.

//    std::string base_link_frame;
    double r, h, angle;
    nh.param("pointcloud_radius", r, 0.25); pc_radius_ = r;
    nh.param("pointcloud_height", h, 0.04); pc_height_ = h;
    nh.param("side_point_angle", angle, 0.34906585);  pc_angle_ = angle;
    nh.param<std::string>("bumper_left_frame", bumper_left_frame_, "bumper_left");
    nh.param<std::string>("bumper_right_frame", bumper_right_frame_, "bumper_right");
    nh.param<std::string>("base_frame", base_frame_, "base_link");
    //nh.param<std::string>("base_link_frame", base_link_frame, "base_link");

    // Lateral points x/y coordinates; we need to store float values to memcopy later
    p_side_x_ = + pc_radius_*sin(pc_angle_); // angle degrees from vertical
    p_side_y_ = + pc_radius_*cos(pc_angle_); // angle degrees from vertical
    n_side_y_ = - pc_radius_*cos(pc_angle_); // angle degrees from vertical

    // Prepare constant parts of the pointcloud message to be  published
    pointcloud_.header.frame_id = base_frame_;
    pointcloud_.width  = 2;
    pointcloud_.height = 1;
    pointcloud_.fields.resize(3);

    // Set x/y/z as the only fields
    pointcloud_.fields[0].name = "x";
    pointcloud_.fields[1].name = "y";
    pointcloud_.fields[2].name = "z";

    int offset = 0;
    // All offsets are *4, as all field data types are float32
    for (size_t d = 0; d < pointcloud_.fields.size(); ++d, offset += 4)
    {
      pointcloud_.fields[d].count    = 1;
      pointcloud_.fields[d].offset   = offset;
      pointcloud_.fields[d].datatype = sensor_msgs::PointField::FLOAT32;
    }

    pointcloud_.point_step = offset;
    pointcloud_.row_step   = pointcloud_.point_step * pointcloud_.width;

    // width * point_step
    pointcloud_.data.resize(2 * pointcloud_.point_step);
    pointcloud_.is_bigendian = false;
    pointcloud_.is_dense     = true;

    // Bumper/cliff "points" fix coordinates (the others depend on sensor activation/deactivation)

    // y: always 0 for central bumper
    //memcpy(&pointcloud_.data[1 * pointcloud_.point_step + pointcloud_.fields[1].offset], &ZERO, sizeof(float));

    // Bumper "points" fix coordinates (the others depend on sensor activation/deactivation)
    // z: constant elevation from base frame
    memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[2].offset], &pc_height_, sizeof(float));
    memcpy(&pointcloud_.data[1 * pointcloud_.point_step + pointcloud_.fields[2].offset], &pc_height_, sizeof(float));
    //memcpy(&pointcloud_.data[2 * pointcloud_.point_step + pointcloud_.fields[2].offset], &pc_height_, sizeof(float));

    pointcloud_pub_  = nh.advertise <sensor_msgs::PointCloud2> ("/sensors/bumper_pointcloud", 10);
    // the bumper info is coming from as emergency
    emergency_sub_ = nh.subscribe("/ll/emergency", 10, &Bumper2PointcloudNode::BumperCB, this);

    // get tf between base_frame and bummper_frames
    get_tf_bumper();
    ROS_INFO("Bumper pointcloud configured at distance %f and height %f from base frame", pc_radius_, pc_height_);
  }

  // callback
  void Bumper2PointcloudNode::BumperCB(const mower_msgs::Emergency::ConstPtr& msg)
  {
    if (pointcloud_pub_.getNumSubscribers() == 0)
      return;

    // We publish just one "no events" pc (with all three points far away) and stop spamming when bumper/cliff conditions disappear
    if (! msg->lbump && ! prev_lbump && ! msg->rbump && ! prev_rbump )
      return;

    prev_lbump = msg->lbump;
    prev_rbump  = msg->rbump;

    // We replicate the sensors order of bumper/cliff event messages: LEFT = 0, CENTER = 1 and RIGHT = 2
    // For any of {left/center/right} with no bumper/cliff event, we publish a faraway point that won't get used 
    if ( msg->lbump )
    {
      distance_x_ = p_side_x_ + bumper_frame_left_x;
      distance_y_ = bumper_frame_left_y + p_side_y_;
      memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[0].offset], &distance_x_, sizeof(float));
      memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[1].offset], &distance_y_, sizeof(float));
    }
    else
    {
      memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[0].offset], &P_INF_X, sizeof(float));
      memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[1].offset], &P_INF_Y, sizeof(float));
    }

    /*
    we don't fake central bumber now
    if ( msg->lbump && msg->rbump )
    {
      memcpy(&pointcloud_.data[1 * pointcloud_.point_step + pointcloud_.fields[0].offset], &pc_radius_, sizeof(float));
    }
    else
    {
      memcpy(&pointcloud_.data[1 * pointcloud_.point_step + pointcloud_.fields[0].offset], &P_INF_X, sizeof(float));
    }
    */

    if ( msg->rbump )
    {
      distance_x_ = p_side_x_ + bumper_frame_right_x;
      distance_y_ = bumper_frame_right_y + p_side_y_;
      memcpy(&pointcloud_.data[1 * pointcloud_.point_step + pointcloud_.fields[0].offset], &distance_x_, sizeof(float));
      memcpy(&pointcloud_.data[1 * pointcloud_.point_step + pointcloud_.fields[1].offset], &distance_y_, sizeof(float));
    }
    else
    {
      memcpy(&pointcloud_.data[2 * pointcloud_.point_step + pointcloud_.fields[0].offset], &P_INF_X, sizeof(float));
      memcpy(&pointcloud_.data[2 * pointcloud_.point_step + pointcloud_.fields[1].offset], &N_INF_Y, sizeof(float));
    }

    pointcloud_.header.stamp = msg->stamp;
    pointcloud_pub_.publish(pointcloud_);
  }
  
  // Get transformation between base_link and bumper frames to calculate point location
    void Bumper2PointcloudNode::get_tf_bumper()
    {
        tf2_ros::TransformListener tfListener(tfBuffer);
        geometry_msgs::TransformStamped transformStamped;
        try
        {
            transformStamped = tfBuffer.lookupTransform(base_frame_, bumper_left_frame_, ros::Time(0), ros::Duration(3.0));
            bumper_frame_left_x = transformStamped.transform.translation.x;
            bumper_frame_left_y = transformStamped.transform.translation.y;
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
        }

        try
        {
            transformStamped = tfBuffer.lookupTransform(base_frame_, bumper_right_frame_, ros::Time(0), ros::Duration(3.0));
            bumper_frame_right_x = transformStamped.transform.translation.x;
            bumper_frame_right_y = transformStamped.transform.translation.y;
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
        } 
       
    }

}

// Main function
int main(int argc, char** argv) {
    ros::init(argc, argv, "bumper2pointcloud");
    bumper2pointcloud::Bumper2PointcloudNode node;
    ros::spin(); // Keep it continous / Hoia node pidevalt töös
    return 0;
}