#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/ColorRGBA.h"
#include "sensor_msgs/LaserScan.h"

#include <fstream>
#include <numeric>
#include <cmath>
#include <algorithm>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/ml.hpp>
#include <stack>

using namespace std;

// **********************************************************************************
struct compare
{
private:
  pair<double,double> p0;
public:
  compare(const pair<double,double>& p0Val) 
  { 
    p0 = p0Val; 
  }

  bool operator()(const pair<double,double>& p1, const pair<double,double>& p2)
  {
    int o = orientation(p0, p1, p2);
    if (o == 0)
      return (dist_sq(p0, p2) >= dist_sq(p0, p1))? true : false;
  
    return (o == 2)? true: false;

    cout << "SOMETHINGS WRONG!!!!!!!!!!!!!!!!!!!!" << endl;
    return false;
  }

  double dist_sq(const pair<double,double>& p1, const pair<double,double>& p2)
  {
    return (p1.first - p2.first)*(p1.first - p2.first) + 
            (p1.second - p2.second)*(p1.second - p2.second);
  }

  int orientation(const pair<double,double>& p, const pair<double,double>& q, const pair<double,double>& r)
  {
    double val = (q.second - p.second) * (r.first - q.first) - 
              (q.first - p.first) * (r.second - q.second);
  
    if (val == 0) return 0;  // collinear
    return (val > 0)? 1: 2; // clock or counterclock wise
  }
};

// **********************************************************************************
class place_detector
{
private:
  enum MODE {RECORD_SCANS, REALTIME_PREDICTION, FEATURE_EXTRACTION, LABEL_SCANS, SVM_TRAINING, TEST, NONE};

  ros::NodeHandle* nh_;

  ros::Subscriber scanSub_;
  ros::Subscriber labelSub_;
  ros::Publisher labelPub_;
  ros::Publisher convHullPub_;
  ros::Publisher scanPub_;

  MODE mode_ = MODE::NONE;

  ofstream dataFile_;
  map<string, int> labelToIndx_;
  map<int, string> indxToLabel_;

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

  string scanFrameId_ = "";

  string filePath_ = ""; // file path to write feature set to

  int nFeatureVecsWritten_ = 0;

  vector<vector<string>> rawScansIn_; // used for label scans mode
  vector<vector<string>> labelledScansOut_; // used for label scans mode
  vector<vector<string>> featuresOut_; // used for label scans mode
  int scanLabelItr_ = -1; // iterator of the scan curently being shown
  stack<string> labelActions_; // used for label scans mode

  const double pi_ = atan(1)*4;

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
  double convex_perimeter(const vector<pair<double,double>>& convHullPts);
  double dist(const pair<double, double>& pt1, const pair<double, double>& pt2);
  vector<pair<double,double>> convex_hull_points(const int& bottomPtIndx);
  double form_factor(const double& area, const double& circumCircleArea);
  double circumscribed_circle_area(const pair<double,double>& cog);
  pair<double, double> cog();
  vector<double> seven_invariants(const pair<double,double>& cog, vector<double>& secondOrderCentralMoments);
  double p_q_th_order_central_moment(const int& p, const int& q, const pair<double,double>& cog);
  pair<double,double> area_perimeter_polygon(int& bottomPtIndx);
  void update_feature_vec_a();
  int n_gaps(const double& thresh);
  pair<double, double> mean_sdev_range_diff(const float& thresh);
  void test_function();
  void write_feature_vecs_to_file();
  void ros_info(const string& s, double throttle = -1);
  void ros_warn(const string& s, double throttle = -1);
  void publish_convex_hull(const vector<pair<double,double>>& convHullPts, const int& bottomPtIndx);
  bool fill_gaps_in_scan();
  void interp_scan(const int& indx1, const int& indx2, const vector<int>& midInds);
  void swap(pair<double,double>& p1, pair<double,double>& p2);
  void train_svm();
  void print_label_counts_svm(const cv::Mat& reponsesMat);

  void label_scans();
  void publish_raw_scan(const int& row);
  vector<vector<string>> read_from_file();
  void write_to_file(const vector<vector<string>>& contentIn);
};

template <typename T> ostream& operator<<(ostream& os, const vector<T>& vecIn);
template <typename T1, typename T2> ostream& operator<<(ostream& os, const vector<pair<T1,T2>>& vecIn);
template <typename T1, typename T2> ostream& operator<<(ostream& os, const pair<T1,T2>& pairIn);