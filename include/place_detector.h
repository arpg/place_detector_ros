#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"

#include <fstream>
#include <numeric>

using namespace std;

class place_detector
{
private:
  enum MODE {FEATURE_EXTRACTION, REALTIME_PREDICTION, SVM_TRAINING, TEST};

  ros::NodeHandle* nh_;

  ros::Subscriber scanSub_;
  ros::Subscriber labelSub_;
  ros::Publisher labelPub_;

  MODE mode_;

  ofstream dataFile_;
  //map<string, int> labelToIndx_;

  double scanAngleMin_ = 0;
  double scanAngleMax_ = 0;
  double scanAngleInc_ = 0;

  vector<float> scanR_; // ranges
  vector<pair<double,double>> scanP_; // polygon, cartesian

  vector<double> featureVecA_;
  vector<double> featureVecB_;

  double featureVecAComputeTime_ = 0.0;
  double featureVecBComputeTime_ = 0.0;

  string mostRecentLabel_ = "";
  ros::Time mostRecentLabelTime_;

  string filePath_ = ""; // file path to write feature set to
  int nFeatureVecsWritten_ = 0;

  double pi_ = atan(1)*4;

public:
  place_detector(ros::NodeHandle* nh);
  ~place_detector();
  void load_params();
  bool is_valid(const MODE& mode);
  void label_cb(const std_msgs::String& labelMsg);
  void scan_cb(const sensor_msgs::LaserScan& scanMsg);
  void scan_cb_feature_extraction_mode(const sensor_msgs::LaserScan& scanMsg);
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
  double p_q_th_order_central_moment(const int& p, const int& q, const pair<double,double>& cog);
  pair<double,double> area_perimeter_polygon(double& longestRangeIndx);
  void update_feature_vec_a();
  int n_gaps(const double& thresh);
  pair<double, double> mean_sdev_range_diff(const float& thresh);
  void test_function();
  void write_feature_vecs_to_file();
  void ros_info(const string& s, double throttle = -1);
  void ros_warn(const string& s, double throttle = -1);

};

template <typename T> ostream& operator<<(ostream& os, const vector<T>& vecIn);
template <typename T1, typename T2> ostream& operator<<(ostream& os, const vector<pair<T1,T2>>& vecIn);
template <typename T1, typename T2> ostream& operator<<(ostream& os, const pair<T1,T2>& pairIn);