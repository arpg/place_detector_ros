#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"

#include <fstream>
#include <numeric>

using namespace std;

class place_detector
{
private:
  ros::NodeHandle* nh_;

  ros::Subscriber scanSub_;
  ros::Subscriber labelSub_;
  ros::Publisher labelPub_;

  bool dataLabelMode_ = false;

  ofstream dataFile_;
  //map<string, int> labelToIndx_;

  double scanAngleMin_ = 0;
  double scanAngleMax_ = 0;
  double scanAngleInc_ = 0;

  vector<float> scanR_; // ranges
  vector<pair<double,double>> scanP_; // polygon, cartesian

  vector<double> featureVecA_;
  vector<double> featureVecB_;

  string mostRecentLabel_ = "";
  double mostRecentLabelTime_ = 0.0;

  string filePath_ = ""; // file path to write feature set to

  double pi_ = atan(1)*4;
public:
  place_detector(ros::NodeHandle* nh);
  ~place_detector();
  void load_params();
  void label_cb(const std_msgs::String& labelMsg);
  void scan_cb(const sensor_msgs::LaserScan& scanMsg);
  void scan_cb_data_label_mode(const sensor_msgs::LaserScan& scanMsg);
  void update_feature_vec_b();
  double compactness(const double& area, const double& perimeter);
  double eccentricity(const double& area, const vector<double>& secondOrderCentralMoments);
  double roundness(const double& area, const double& convexPerimeter);
  double convex_perimeter(const vector<double>& convHullInds);
  double dist(const pair<double, double>& pt1, const pair<double, double>& pt2);
  vector<double> convex_hull_indices(const int& longestRangeIndx);
  int orientation(const pair<double,double>& p, const pair<double,double>& q, const pair<double,double>& r);
  double form_factor(const double& area, const double& circumCircleArea);
  double circumscribed_circle_area(const pair<double,double>& cog);
  pair<double, double> cog();
  vector<double> seven_invariants(const pair<double,double>& cog, vector<double>& secondOrderCentralMoments);
  pair<double,double> area_perimeter_polygon(double& longestRangeIndx);
  void update_feature_vec_a();
  int n_gaps(const double& thresh);
  pair<double, double> mean_sdev_range_diff(const float& thresh);
  void test_function();
  void ros_info(const string& s);
  void ros_warn(const string& s);

};

ostream& operator<<(ostream& os, const vector<double>& vecIn);