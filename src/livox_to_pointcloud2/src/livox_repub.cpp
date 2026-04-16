#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include "livox_ros_driver2/CustomMsg.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

typedef pcl::PointXYZINormal PointType;

ros::Publisher pub_pcl_out;
uint64_t TO_MERGE_CNT = 1; 
constexpr bool b_dbg_line = false;
std::vector<livox_ros_driver2::CustomMsgConstPtr> livox_data;

std::unique_ptr<Eigen::Affine3d> sensor2novatel;
std::unique_ptr<Eigen::Affine3d> novatel2world;
std::unique_ptr<Eigen::Affine3d> sensor2world;

bool b_tf = false;

void LivoxMsgCbk1(const livox_ros_driver2::CustomMsgConstPtr& livox_msg_in) {

  if((sensor2novatel == nullptr)&&(novatel2world == nullptr)&&(sensor2world == nullptr)&&(b_tf == false))
  {
      return;
  }

  livox_data.push_back(livox_msg_in);
  if (livox_data.size() < TO_MERGE_CNT) return;

  pcl::PointCloud<PointType> handle_cloud;
  pcl::PointCloud<PointType> out_cloud;
  for (size_t j = 0; j < livox_data.size(); j++) {
    auto& livox_msg = livox_data[j];
    auto time_end = livox_msg->points.back().offset_time;
    for (unsigned int i = 0; i < livox_msg->point_num; ++i) {
      PointType pt;
      pt.x = livox_msg->points[i].x;
      pt.y = livox_msg->points[i].y;
      pt.z = livox_msg->points[i].z;
      float s = livox_msg->points[i].offset_time / (float)time_end;
      pt.intensity = livox_msg->points[i].line + s*0.1; // The integer part is line number and the decimal part is timestamp
      pt.curvature = livox_msg->points[i].reflectivity * 0.1;
      handle_cloud.push_back(pt);
    }
  }
 
    //ROS_INFO("in__cloud points size: %ld",handle_cloud.points.size());
    
    if(handle_cloud.points.size() == 0)
    {
        return;
    }

    for (size_t i = 0; i < handle_cloud.points.size(); ++i) {
        //if(handle_cloud.points[i].x > 50) continue;
        //if(handle_cloud.points[i].y > 50) continue;
        Eigen::Vector3d trans_point(-handle_cloud.points[i].y, handle_cloud.points[i].x, handle_cloud.points[i].z);
        trans_point = (*sensor2novatel) * trans_point;
        PointType new_point;
        
        new_point.x = trans_point(0);
        new_point.y = trans_point(1);
        new_point.z = trans_point(2);
        new_point.intensity = handle_cloud.points[i].intensity;
        // new_point.intensity = 1;
        out_cloud.points.push_back(new_point);
    }
    ROS_INFO("out_cloud points size: %ld",out_cloud.points.size());
    if(out_cloud.points.size() == 0)
    {
        return;
    }
    
    sensor_msgs::PointCloud2 output_points;
    pcl::toROSMsg(out_cloud,output_points);
    output_points.header.frame_id = "livox_frame";
    output_points.header.stamp = ros::Time::now();
    pub_pcl_out.publish(output_points);
    livox_data.clear();
    b_tf = false;
}

void imuCallback(sensor_msgs::Imu imu)
{
    double x_position = imu.linear_acceleration_covariance[1];
    double y_position = imu.linear_acceleration_covariance[2];
    double z_position = imu.linear_acceleration_covariance[3];

    double x_euler_angles = imu.angular_velocity_covariance[0];
    double y_euler_angles = imu.angular_velocity_covariance[1];
    double z_heading = imu.angular_velocity_covariance[2]-1.5707963;

    double x_trans_value = 0.0;
    double y_trans_value = 6.4;
    double z_trans_value = 2.8;

    double x_rotate_value = 0;
    double y_rotate_value = -0.21;
    double z_rotate_value = -0.014;

    double coeff = static_cast<double>(0.5);
    double r = x_euler_angles * coeff;
    double p = y_euler_angles * coeff;
    double y = z_heading * coeff;

    double sr = std::sin(r);
    double sp = std::sin(p);
    double sy = std::sin(y);

    double cr = std::cos(r);
    double cp = std::cos(p);
    double cy = std::cos(y);

    double qw = cr * cp * cy - sr * sp * sy;
    double qx = cr * sp * cy - sr * cp * sy;
    double qy = cr * sp * sy + sr * cp * cy;
    double qz = cr * cp * sy + sr * sp * cy;
    
    double x_orientation,y_orientation,z_orientation,w_orientation;

    if (qw < 0.0) {
        x_orientation = -qx;
        y_orientation = -qy;
        z_orientation = -qz;
        w_orientation = -qw;
    }
    else
    {
        x_orientation = qx;
        y_orientation = qy;
        z_orientation = qz;
        w_orientation = qw;
    }

    r = x_rotate_value * coeff;
    p = y_rotate_value * coeff;
    y = z_rotate_value * coeff;

    sr = std::sin(r);
    sp = std::sin(p);
    sy = std::sin(y);

    cr = std::cos(r);
    cp = std::cos(p);
    cy = std::cos(y);

    qw = cr * cp * cy - sr * sp * sy;
    qx = cr * sp * cy - sr * cp * sy;
    qy = cr * sp * sy + sr * cp * cy;
    qz = cr * cp * sy + sr * sp * cy;
    
    double qx_orientation,qy_orientation,qz_orientation,qw_orientation;

    if (qw < 0.0) {
        qx_orientation = -qx;
        qy_orientation = -qy;
        qz_orientation = -qz;
        qw_orientation = -qw;
    }
    else
    {
        qx_orientation = qx;
        qy_orientation = qy;
        qz_orientation = qz;
        qw_orientation = qw;
    }

    
    //生成雷达坐标系到车坐标系仿射变换矩阵
    Eigen::Translation3d sensor2novatel_translation;
    Eigen::Quaterniond sensor2novatel_rotation;  

    sensor2novatel_translation = Eigen::Translation3d(x_trans_value,y_trans_value,z_trans_value);                            
    sensor2novatel_rotation = Eigen::Quaterniond(qw_orientation,qx_orientation,qy_orientation,qz_orientation);
    
    sensor2novatel.reset(new Eigen::Affine3d);
    *sensor2novatel = sensor2novatel_translation * sensor2novatel_rotation;

    //生成车坐标系到世界坐标系仿射变换矩阵
    Eigen::Translation3d novatel2world_translation;
    Eigen::Quaterniond novatel2world_rotation;

    novatel2world_translation = Eigen::Translation3d(x_position,y_position,z_position);                            
    novatel2world_rotation = Eigen::Quaterniond(w_orientation,x_orientation,y_orientation,z_orientation);

    novatel2world.reset(new Eigen::Affine3d);
    *novatel2world = novatel2world_translation * novatel2world_rotation;
    
    //生成雷达坐标系到世界坐标系仿射变换矩阵  
    sensor2world.reset(new Eigen::Affine3d);
    *sensor2world = *novatel2world * (*sensor2novatel);       

    b_tf = true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "livox_repub_node");
  ros::NodeHandle nh;

  ROS_INFO("start livox_repub");
    
  pub_pcl_out = nh.advertise<sensor_msgs::PointCloud2>("/livox/lidar_pc2", 1);

  ros::Subscriber sub_livox_msg1 = nh.subscribe<livox_ros_driver2::CustomMsg>(
      "/livox/lidar", 1, LivoxMsgCbk1);
    
   ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/livox/imu", 1, imuCallback);
    
  ros::Rate loop_rate(10);

  while(ros::ok())
  {
      ros::spinOnce();
      loop_rate.sleep();
  }
}
