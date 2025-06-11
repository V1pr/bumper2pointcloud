/*
 * Copyright (c) 2025, Tamas Dajka.
 * All rights reserved.
 *
 * Code based on/inspired by the work of 2013, Yujin Robot kobuki_bumper2pointcloud
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Yujin Robot nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file /include/bumper2pointcloud/bumper2pointcloud.hpp
 *
 * @brief Bumper/cliff to pointcloud node class declaration.
 *
 * Publish bumpers and cliff sensors events as points in a pointcloud, so navistack can use them
 * for poor-man navigation. Implemented as a node intended to run together with kobuki_node.
 *
 * @author Jorge Santos, Yujin Robot
 *
 **/

#ifndef _BUMPER2POINTCLOUD_HPP_
#define _BUMPER2POINTCLOUD_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <mower_msgs/Emergency.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

/*****************************************************************************
 ** Namespace
 *****************************************************************************/

/**
 * @brief bumper2pointcloudNode class declaration
 */
 
namespace bumper2pointcloud
{

  class Bumper2PointcloudNode
  {
  public:
    Bumper2PointcloudNode();
    
    ~Bumper2PointcloudNode() { };

  private:
    const float P_INF_X;  // somewhere out of reach from the robot (positive x)
    const float P_INF_Y;  // somewhere out of reach from the robot (positive y)
    const float N_INF_Y;  // somewhere out of reach from the robot (negative y)
    const float ZERO;

    bool prev_rbump;
    bool prev_lbump;
    
    // additional bumper attributes
    std::string bumper_left_frame_;
    std::string bumper_right_frame_;
    std::string base_frame_;

    float pc_radius_;
    float pc_height_;
    float p_side_x_;
    float p_side_y_;
    float n_side_y_;
    float distance_x_;
    float distance_y_;

    // tf transformation between base_frame and bumper frames
    float bumper_frame_left_x = 0.0;
    float bumper_frame_left_y = 0.0;
    float bumper_frame_right_x = 0.0;
    float bumper_frame_right_y = 0.0;
    tf2_ros::Buffer tfBuffer;   

    ros::Publisher  pointcloud_pub_;
    ros::Subscriber emergency_sub_;

    sensor_msgs::PointCloud2 pointcloud_;

    /**
     * @brief Core sensors state structure callback
     * @param msg incoming topic message
     */
    void BumperCB(const mower_msgs::Emergency::ConstPtr& msg);

    /**
    * @brief transformation betweeb base link and bumper frame
    *
    */
    void get_tf_bumper();
    
    // ToDo: dynamic reconfiguration support
  };
}; // end namespace

#endif // _BUMPER2POINTCLOUD_HPP_
