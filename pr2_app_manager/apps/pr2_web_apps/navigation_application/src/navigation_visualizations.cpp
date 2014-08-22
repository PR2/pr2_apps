/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include "message_filters/time_synchronizer.h"
#include "nav_msgs/GridCells.h"
#include "nav_msgs/OccupancyGrid.h"

namespace navigation_application {
class NavigationVisualization {
  public:
    NavigationVisualization(): tf_(ros::Duration(10.0)), laser_sub_(NULL), tf_laser_filter_(NULL),
    tf_obs_(NULL), tf_inf_(NULL),
    obstacle_sub_(NULL), inflated_obstacle_sub_(NULL), obs_sync_(NULL){
      ros::NodeHandle n;
      ros::NodeHandle nav_nh("nav_app_visualizations");

      //we'll get the size of the grid from the parameters used to construct the local costmap, same defaults as in the costmap
      //wish that this info was available over ROS, but it isn't so oh well
      ros::NodeHandle local_map_nh("move_base_node/local_costmap");
      local_map_nh.param("resolution", resolution_, 0.05); 
      local_map_nh.param("width", width_, 10.0); 
      local_map_nh.param("height", height_, 10.0); 

      laser_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(n, "base_scan_throttled", 10);
      tf_laser_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*laser_sub_, tf_, "/map", 10);

      tf_laser_filter_->registerCallback(boost::bind(&NavigationVisualization::laserScanCallback, this, _1));
      laser_point_pub_ = nav_nh.advertise<sensor_msgs::PointCloud>("base_scan", 1);

      obs_pub_ = nav_nh.advertise<nav_msgs::OccupancyGrid>("costmap", 1);

      obstacle_sub_ = new message_filters::Subscriber<nav_msgs::GridCells>(n, "move_base_node/local_costmap/obstacles", 1);
      inflated_obstacle_sub_ = new message_filters::Subscriber<nav_msgs::GridCells>(n, "move_base_node/local_costmap/inflated_obstacles", 1);

      tf_obs_ = new tf::MessageFilter<nav_msgs::GridCells>(*obstacle_sub_, tf_, "/map", 10);
      tf_inf_ = new tf::MessageFilter<nav_msgs::GridCells>(*inflated_obstacle_sub_, tf_, "/map", 10);

      std::vector<std::string> frames;
      frames.push_back("/map");
      frames.push_back("base_link");
      tf_obs_->setTargetFrames(frames);
      tf_inf_->setTargetFrames(frames);

      obs_sync_ = new message_filters::TimeSynchronizer<nav_msgs::GridCells, nav_msgs::GridCells>(*tf_obs_, *tf_inf_, 10);
      obs_sync_->registerCallback(boost::bind(&NavigationVisualization::obstaclesCallback, this, _1, _2));

    }

    ~NavigationVisualization(){
      delete tf_laser_filter_;
      delete laser_sub_;
      delete obs_sync_;
      delete tf_obs_;
      delete tf_inf_;
      delete obstacle_sub_;
      delete inflated_obstacle_sub_;
    }

    inline int getIndex(double wx, double wy, const nav_msgs::OccupancyGrid& grid) {
      //make sure to bounds check
      double upper_bound_x = grid.info.origin.position.x + (grid.info.width * grid.info.resolution);
      double upper_bound_y = grid.info.origin.position.y + (grid.info.height * grid.info.resolution);
      if(wx < grid.info.origin.position.x || wx >= upper_bound_x || wy < grid.info.origin.position.y || wy >= upper_bound_y)
        return -1;

      unsigned int mx = int((wx - grid.info.origin.position.x) / grid.info.resolution);
      unsigned int my = int((wy - grid.info.origin.position.y) / grid.info.resolution);

      return my * grid.info.width + mx;
    }

    void obstaclesCallback(const nav_msgs::GridCells::ConstPtr& obstacles, const nav_msgs::GridCells::ConstPtr& inflated_obstacles){
      //we need a grid to send out
      nav_msgs::OccupancyGrid grid;

      //we'll build the grid in the odometric frame and transform it later
      grid.header.frame_id = obstacles->header.frame_id;
      grid.header.stamp = obstacles->header.stamp;

      grid.info.resolution = resolution_;
      grid.info.width = int(width_ / resolution_);
      grid.info.height = int(height_ / resolution_);
      grid.data.resize(grid.info.width * grid.info.height);

      //first, we'll get the transform we want from tf
      tf::StampedTransform  robot_transform;
      try{
        tf_.lookupTransform(obstacles->header.frame_id, "base_link", obstacles->header.stamp, robot_transform);
      }
      catch(tf::TransformException &ex){
        ROS_ERROR("This should never happen: %s", ex.what());
        return;
      }

      //get the position of the robot in the map frame
      tf::Point robot_position = robot_transform * tf::Point(0.0, 0.0, 0.0);

      //we assume that the robot is centered in the costmap and can compute the origin from there
      grid.info.origin.position.x = robot_position.getX() - width_ / 2.0;
      grid.info.origin.position.y = robot_position.getY() - height_ / 2.0;
      grid.info.origin.position.z = 0.0;

      //put the obstacles into the grid
      for(unsigned int i = 0; i < obstacles->cells.size(); ++i){
        int index = getIndex(obstacles->cells[i].x, obstacles->cells[i].y, grid);
        if(index >= 0)
          grid.data[index] = 254;
      }

      //put the inflated obstacles into the grid
      for(unsigned int i = 0; i < inflated_obstacles->cells.size(); ++i){
        int index = getIndex(inflated_obstacles->cells[i].x, inflated_obstacles->cells[i].y, grid);
        if(index >= 0)
          grid.data[index] = 253;
      }

      //we actually want to publish the grid in the map frame, so we'll change the origin here
      geometry_msgs::PoseStamped orig_origin, map_origin;
      orig_origin.header = obstacles->header;
      orig_origin.pose.position = grid.info.origin.position;
      orig_origin.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

      try{
        tf_.transformPose("/map", orig_origin, map_origin);
      }
      catch(tf::TransformException &ex){
        ROS_ERROR("This should never happen: %s", ex.what());
        return;
      }

      //now... the grid is in the map frame, we'll update the header
      grid.header = map_origin.header;
      grid.info.origin = map_origin.pose;

      ROS_DEBUG("Publishing a grid with x, y, th: (%.2f, %.2f, %.2f)", grid.info.origin.position.x, grid.info.origin.position.y, tf::getYaw(grid.info.origin.orientation));
      obs_pub_.publish(grid);
    }

    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
      sensor_msgs::PointCloud base_scan_cloud;
      base_scan_cloud.header = scan->header;

      try{
        projector_.transformLaserScanToPointCloud(scan->header.frame_id, *scan, base_scan_cloud, tf_);

        tf_.transformPointCloud("map", base_scan_cloud, base_scan_cloud);
        base_scan_cloud.channels.resize(0);
        laser_point_pub_.publish(base_scan_cloud);
      }
      catch(tf::TransformException &ex){
        ROS_ERROR("This should not happen");
      }
    }

    tf::TransformListener tf_;
    laser_geometry::LaserProjection projector_;
    ros::Publisher laser_point_pub_;
    ros::Publisher obs_pub_;
    message_filters::Subscriber<sensor_msgs::LaserScan>* laser_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan>* tf_laser_filter_;
    tf::MessageFilter<nav_msgs::GridCells>* tf_obs_, *tf_inf_;

    message_filters::Subscriber<nav_msgs::GridCells>* obstacle_sub_, *inflated_obstacle_sub_;
    message_filters::TimeSynchronizer<nav_msgs::GridCells, nav_msgs::GridCells>* obs_sync_;

    double width_, height_, resolution_;
};

};


int main(int argc, char **argv){
  ros::init(argc, argv, "nav_app_visualizations");
  navigation_application::NavigationVisualization visualizer;

  ros::spin();

  return(0);
}
