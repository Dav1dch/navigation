#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

ros::Publisher filtered_points_pub;

std::string POINTS_INPUT;
std::string POINTS_OUTPUT;

double X_MIN = 0;
double X_MAX = 100;
double Y_MIN = 0;
double Y_MAX = 100;
double Z_MIN = 0;
double Z_MAX = 100;

// 
static pcl::PointCloud<pcl::PointXYZI> removePointsByRange(pcl::PointCloud<pcl::PointXYZI> scan)
{
  pcl::PointCloud<pcl::PointXYZI> narrowed_scan;
  narrowed_scan.header = scan.header;

  for(pcl::PointCloud<pcl::PointXYZI>::const_iterator iter = scan.begin(); iter != scan.end(); ++iter)
  {
    const pcl::PointXYZI &p = *iter;
   
    if(p.x > X_MIN && p.x < X_MAX && 
       p.y > Y_MIN && p.y < Y_MAX &&
       p.z > Z_MIN && p.z < Z_MAX){
      narrowed_scan.points.push_back(p);
    }
  }

  return narrowed_scan;
}

static void scan_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  pcl::PointCloud<pcl::PointXYZI> scan;
  pcl::fromROSMsg(*input, scan);
  scan = removePointsByRange(scan);

  pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());

  sensor_msgs::PointCloud2 filtered_msg;

  pcl::toROSMsg(*scan_ptr, filtered_msg);
  
  filtered_msg.header = input->header;
  filtered_points_pub.publish(filtered_msg);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "points_filter");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  private_nh.getParam("points_input", POINTS_INPUT);
  private_nh.getParam("points_output", POINTS_OUTPUT);
  private_nh.getParam("x_min", X_MIN);
  private_nh.getParam("x_max", X_MAX);
  private_nh.getParam("y_min", Y_MIN);
  private_nh.getParam("y_max", Y_MAX);
  private_nh.getParam("z_min", Z_MIN);
  private_nh.getParam("z_max", Z_MAX);

  ROS_INFO("x_min is: %.2f  ", X_MIN);
  ROS_INFO("x_max is: %.2f  ", X_MAX);
  ROS_INFO("y_min is: %.2f  ", Y_MIN);
  ROS_INFO("y_max is: %.2f  ", Y_MAX);
  ROS_INFO("z_min is: %.2f  ", Z_MIN);
  ROS_INFO("z_max is: %.2f  ", Z_MAX);
  ROS_INFO("points_input is: %s ", POINTS_INPUT.c_str());
  ROS_INFO("points_output is: %s ", POINTS_OUTPUT.c_str());

  // Subscribers
  ros::Subscriber scan_sub = nh.subscribe(POINTS_INPUT, 10, scan_callback);

  // Publishers
  filtered_points_pub = nh.advertise<sensor_msgs::PointCloud2>(POINTS_OUTPUT, 10);


  ros::spin();

  return 0;
}
