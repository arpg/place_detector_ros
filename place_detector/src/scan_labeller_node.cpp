#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "fstream"

using namespace std;
	
ros::NodeHandle* nh_;
double mostRecentLabelTime_ = 0.0;
string mostRecentLabel_ = "";

ofstream dataFile_;

void ros_info(const string& s);
void label_cb(const std_msgs::String& labelMsg);
void scan_cb(const sensor_msgs::LaserScan& scanMsg);
void ros_warn(const string& s);

int main(int argc, char** argv)
{
	ros::init(argc, argv, "scan_labeller");
	nh_ = new ros::NodeHandle(ros::this_node::getName());

  ros_info("Waiting for params to load ...");

  string folderPath;
  while(!nh_->getParam("folder_path", folderPath));

	dataFile_.open(folderPath+"data_file.csv");

	ros::Subscriber scanSub = nh_->subscribe("scan_in", 1, scan_cb);
	ros::Subscriber labelSub = nh_->subscribe("label_in", 1, label_cb);
	
	ros::spin();

	dataFile_.close();
	return 0;
}
// **********************************************************************************
void label_cb(const std_msgs::String& labelMsg)
{
	mostRecentLabel_ = labelMsg.data;
	mostRecentLabelTime_ = ros:Time::now().toSec();
}

// **********************************************************************************
void scan_cb(const sensor_msgs::LaserScan& scanMsg)
{
	if(mostRecentLabel_ == "")
		return;

	if( ros::Time::now().toSec() - mostRecentLabelTime_ > 0.2 )
	{
		ros_warn("Most recent label is stale");
		return;
	}

	dataFile_ << mostRecentLabel_;

	for(int i=0; i<scanMsg.ranges.size(); i++)
		dataFile_ << ", " << to_string(scanMsg.ranges[i]); 
		
	dataFile_ << endl;
}

// **********************************************************************************
void ros_info(const string& s)
{
	ROS_INFO("s: " + s, nh_->getNamespace().c_str());	
}

// **********************************************************************************
void ros_warn(const string& s)
{
	ROS_WARN("s: " + s, nh_->getNamespace().c_str());	
}

// **********************************************************************************

