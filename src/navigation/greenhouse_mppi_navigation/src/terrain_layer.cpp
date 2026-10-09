// SPDX-License-Identifier: Apache-2.0
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <chrono>
#include <mutex>
#include <cmath>

namespace greenhouse_mppi_navigation {
class TerrainLayer final : public nav2_costmap_2d::Layer {
 public:
  void onInitialize() override {
    auto node=node_.lock(); if(!node) throw std::runtime_error("costmap node expired");
    declareParameter("topic",rclcpp::ParameterValue("/perception/terrain_grid"));
    declareParameter("timeout",rclcpp::ParameterValue(0.7));
    std::string topic; node->get_parameter(name_+".topic",topic); node->get_parameter(name_+".timeout",timeout_);
    if(!std::isfinite(timeout_)||timeout_<=0) throw std::invalid_argument("terrain timeout");
    enabled_=true; current_=false;
    rclcpp::SubscriptionOptions options; options.callback_group=callback_group_;
    subscription_=node->create_subscription<nav_msgs::msg::OccupancyGrid>(topic,
      rclcpp::QoS(1).reliable().transient_local(),[this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m) {
        if(m->header.frame_id!=layered_costmap_->getGlobalFrameID() || m->info.width==0 || m->info.height==0 ||
           m->info.width>1000||m->info.height>1000||!std::isfinite(m->info.resolution)||m->info.resolution<=0||
           m->data.size()!=std::size_t(m->info.width)*m->info.height||
           !std::isfinite(m->info.origin.position.x)||!std::isfinite(m->info.origin.position.y)||
           !std::isfinite(m->info.origin.orientation.w)||!std::isfinite(m->info.origin.orientation.x)||
           !std::isfinite(m->info.origin.orientation.y)||!std::isfinite(m->info.origin.orientation.z)||
           std::fabs(m->info.origin.orientation.w-1)>1e-6||
           std::fabs(m->info.origin.orientation.x)+std::fabs(m->info.origin.orientation.y)+
           std::fabs(m->info.origin.orientation.z)>1e-6) return;
        const double age=(clock_->now()-rclcpp::Time(m->header.stamp)).seconds();
        if(age < -0.1 || age > timeout_) return;
        std::lock_guard<std::mutex> lock(mutex_); grid_=m; receipt_=std::chrono::steady_clock::now();
      },options);
  }
  void reset() override { std::lock_guard<std::mutex> lock(mutex_); grid_.reset(); current_=false; }
  bool isClearable() override { return false; }
  void updateBounds(double,double,double,double* min_x,double* min_y,double* max_x,double* max_y) override {
    auto* grid=layered_costmap_->getCostmap();
    *min_x=std::min(*min_x,grid->getOriginX()); *min_y=std::min(*min_y,grid->getOriginY());
    *max_x=std::max(*max_x,grid->getOriginX()+grid->getSizeInMetersX());
    *max_y=std::max(*max_y,grid->getOriginY()+grid->getSizeInMetersY());
  }
  void updateCosts(nav2_costmap_2d::Costmap2D& master,int min_i,int min_j,int max_i,int max_j) override {
    std::lock_guard<std::mutex> lock(mutex_);
    const double source_age=grid_?(clock_->now()-rclcpp::Time(grid_->header.stamp)).seconds():INFINITY;
    current_=grid_ && source_age>=-0.1 && source_age<=timeout_ &&
      std::chrono::duration<double>(std::chrono::steady_clock::now()-receipt_).count()<=timeout_;
    for(int y=min_j;y<max_j;++y) for(int x=min_i;x<max_i;++x) {
      unsigned char cost=nav2_costmap_2d::LETHAL_OBSTACLE;
      if(current_) {
        double wx,wy; master.mapToWorld(x,y,wx,wy);
        const double fx=(wx-grid_->info.origin.position.x)/grid_->info.resolution;
        const double fy=(wy-grid_->info.origin.position.y)/grid_->info.resolution;
        // Unknown cells remain -1 in the published grid; control treats them as occupied.
        cost=nav2_costmap_2d::LETHAL_OBSTACLE;
        if(fx>=0&&fy>=0&&fx<grid_->info.width&&fy<grid_->info.height) {
          const int gx=std::floor(fx),gy=std::floor(fy);
          const int value=grid_->data[gy*grid_->info.width+gx];
          if(value>=0) cost=value>=50?nav2_costmap_2d::LETHAL_OBSTACLE:nav2_costmap_2d::FREE_SPACE;
        }
      }
      // This is the first layer. Overwrite clears obstacles absent from the new observation.
      master.setCost(x,y,cost);
    }
  }
 private:
  double timeout_=0.7; std::mutex mutex_;
  std::chrono::steady_clock::time_point receipt_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
};
}
PLUGINLIB_EXPORT_CLASS(greenhouse_mppi_navigation::TerrainLayer,nav2_costmap_2d::Layer)
