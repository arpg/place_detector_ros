#include "place_detector.h"

// **********************************************************************************
place_detector::place_detector(ros::NodeHandle* nh)
{
  nh_ = nh;
  load_params();

  if(mode_ == MODE::TEST)
  {
    test_function();
    return;
  }

  dataFile_.open(filePath_);

  scanSub_ = nh_->subscribe("scan_in", 1, &place_detector::scan_cb, this);
  labelSub_ = nh_->subscribe("label_in", 1, &place_detector::label_cb, this);

  labelPub_ = nh_->advertise<std_msgs::String>("label_out", 1);

  return;
}

// **********************************************************************************
place_detector::~place_detector()
{
  dataFile_.close();
}
// **********************************************************************************
void place_detector::load_params()
{
  ros_info("Waiting for params to load ...");

  while( !is_valid(mode_) )
  {
    string mode = "";
    nh_->getParam("mode", mode);

    if( mode == "feature_extraction" )
      mode_ = MODE::FEATURE_EXTRACTION;
    else if( mode == "svm_training" )
      mode_ = MODE::SVM_TRAINING;
    else if( mode == "realtime_prediction" )
      mode_ = MODE::REALTIME_PREDICTION;
    else if( mode == "test" )
      mode_ = MODE::TEST;
  }

  while(!nh_->getParam("file_path", filePath_));

  ros_info("Params loaded");
}

// **********************************************************************************
bool place_detector::is_valid(const MODE& mode)
{
  return (mode_ == MODE::FEATURE_EXTRACTION || mode_ == MODE::SVM_TRAINING || mode_ == MODE::REALTIME_PREDICTION || mode_ == MODE::TEST);
}

// **********************************************************************************
void place_detector::test_function()
{
  ros_warn("TEST FUNCTION CALLED, COMMENT IT OUT IF PROCESSING REALTIME DATA");
  scanR_.resize(0);
  scanR_.push_back(20.5);
  scanR_.push_back(20.0);
  scanR_.push_back(20.5);
  scanR_.push_back(20.5);
  scanR_.push_back(20.5);
  scanR_.push_back(20.5);
  scanR_.push_back(5.5);
  scanR_.push_back(20.5);

  scanAngleMin_ = 0;
  scanAngleMax_ = 0 + pi_/4*7;
  scanAngleInc_ = pi_/4;

  update_feature_vec_a();
  update_feature_vec_b();

  mostRecentLabel_ = "corridor";
  write_feature_vecs_to_file();
  //cout << "done" << endl;
  //cout << scanP_ << endl;
  //update_feature_vec_b();
  //cout << featureVecB_ << endl;
}

// **********************************************************************************
void place_detector::label_cb(const std_msgs::String& labelMsg)
{
	mostRecentLabel_ = labelMsg.data;
	mostRecentLabelTime_ = ros::Time::now();
}

// **********************************************************************************
void place_detector::scan_cb(const sensor_msgs::LaserScan& scanMsg)
{
  if(mode_ == MODE::FEATURE_EXTRACTION)
    scan_cb_feature_extraction_mode(scanMsg);
  //else ...
}

// **********************************************************************************
void place_detector::scan_cb_feature_extraction_mode(const sensor_msgs::LaserScan& scanMsg)
{
	if(mostRecentLabel_ == "")
		return;

	if( (ros::Time::now() - mostRecentLabelTime_).toSec() > 0.1 )
	{
		ros_warn("Most recent label is stale", 5);
		return;
	}

  scanAngleMin_ = scanMsg.angle_min;
  scanAngleMax_ = scanMsg.angle_max;
  scanAngleInc_ = scanMsg.angle_increment;
  scanR_ = scanMsg.ranges;

  update_feature_vec_a();
  update_feature_vec_b();
	write_feature_vecs_to_file();

  mostRecentLabel_ = "";
}

// **********************************************************************************
void place_detector::update_feature_vec_b()
{
  ros::Time startTime = ros::Time::now();

  featureVecB_.resize(0);

  double longestRangeIndx;
  pair<double,double> area_perimeter = area_perimeter_polygon(longestRangeIndx); // updates scanP_ to be used by later functions
  featureVecB_.push_back(area_perimeter.first);
  featureVecB_.push_back(area_perimeter.second);

  if( abs(area_perimeter.second) < 1e-6 )
    ros_warn("Scan perimeter is zero");

  featureVecB_.push_back( area_perimeter.first / area_perimeter.second );

  pair<double,double> cogVal = cog();
  //cout << "cog: " << cogVal << endl;

  vector<double> secondOrderCentralMoments(3,0);
  vector<double> sevenInvariants = seven_invariants(cogVal, secondOrderCentralMoments);
  featureVecB_.insert( featureVecB_.end(), sevenInvariants.begin(), sevenInvariants.end() );

  featureVecB_.push_back( compactness(area_perimeter.first, area_perimeter.second) );
  featureVecB_.push_back( eccentricity(area_perimeter.first, secondOrderCentralMoments) );

  double circumCircleArea = circumscribed_circle_area(cogVal);
  //cout << "circumcircle area: " << circumCircleArea << endl; 
  featureVecB_.push_back( form_factor(area_perimeter.first, circumCircleArea) );

  vector<double> convexHullInds = convex_hull_indices(longestRangeIndx);
  //cout << "convex hull indices: " << convexHullInds << endl;
  double convexPerimeter = convex_perimeter(convexHullInds);
  //cout << "convex perimeter: " << convexPerimeter << endl;
  featureVecB_.push_back( roundness(area_perimeter.first, convexPerimeter) );

  featureVecBComputeTime_ = ( ros::Time::now() - startTime ).toSec();
}

// **********************************************************************************
// the ratio of the area of an object to the area of a circle with the same perimeter
double place_detector::compactness(const double& area, const double& perimeter)
{
  return (4*pi_*area) / (perimeter*perimeter);
}

// **********************************************************************************
// https://docs.baslerweb.com/visualapplets/files/manuals/content/examples%20imagemoments.html
double place_detector::eccentricity(const double& area, const vector<double>& secondOrderCentralMoments)
{
  double mu_0_2 = secondOrderCentralMoments[0];
  double mu_2_0 = secondOrderCentralMoments[1];
  double mu_1_1 = secondOrderCentralMoments[2];

  if( abs(mu_2_0 + mu_0_2) < 1e-6 )
    ros_warn("Eccentricity denominator zero");
  double ecc = pow(mu_2_0 - mu_0_2, 2) + 4*mu_1_1*mu_1_1;
  return ecc / pow(mu_2_0 + mu_0_2, 2); 
  //return ( pow(secondOrderCentralMoments[0] - secondOrderCentralMoments[1], 2) + 4*secondOrderCentralMoments[2] ) / area;
}

// **********************************************************************************
// the ratio of the area of an object to the area of a circle with the same convex perimeter
double place_detector::roundness(const double& area, const double& convexPerimeter)
{
  if( abs(convexPerimeter) < 1e-6 )
    ros_warn("Convex perimeter zero");
  return (4*pi_*area) / (convexPerimeter*convexPerimeter);
}

// **********************************************************************************
// perimeter of the convex hull that encloses the object
double place_detector::convex_perimeter(const vector<double>& convHullInds)
{
  double sum = 0;
  for(int i=0; i<convHullInds.size()-1; i++)
  {
    int hullIndx = convHullInds[i];
    int hullIndxNxt = convHullInds[i+1];
    sum += dist( scanP_[hullIndx], scanP_[hullIndxNxt] );
  }

  int hullIndx = convHullInds.back();
  int hullIndxNxt = convHullInds[0];
  sum += dist( scanP_[hullIndx], scanP_[hullIndxNxt] );

  return sum;
}

// **********************************************************************************
double place_detector::dist(const pair<double, double>& pt1, const pair<double, double>& pt2)
{
  return sqrt( pow(pt2.first-pt1.first,2) + pow(pt2.second-pt1.second,2) );
}

// **********************************************************************************
vector<double> place_detector::convex_hull_indices(const int& longestRangeIndx)
{
  vector<double> convHullInds;
  if(scanP_.size() == 0)
    return vector<double>(0,0);
  if(scanP_.size() == 1)
  {
    convHullInds.push_back(longestRangeIndx);
    return convHullInds;
  }
  if(scanP_.size() == 2)
  {
    convHullInds.push_back(longestRangeIndx);
    convHullInds.push_back( (longestRangeIndx+1) % scanP_.size() );
    return convHullInds;
  }

  convHullInds.push_back( longestRangeIndx );
  convHullInds.push_back( (longestRangeIndx+1) % scanP_.size() );
  convHullInds.push_back( (longestRangeIndx+2) % scanP_.size() );

  for(int i = 3; i<scanP_.size(); i++)
  {
    int indx = (longestRangeIndx+i) % scanP_.size();
    int lastIndx = convHullInds.back();
    int secondToLastIndx = convHullInds[convHullInds.size()-2];

    while (convHullInds.size()>1 && orientation( scanP_[convHullInds[convHullInds.size()-2]] , scanP_[convHullInds.back()], scanP_[indx]) != 2)
      convHullInds.pop_back();
    convHullInds.push_back(indx);
  }

  while (convHullInds.size()>1 && orientation( scanP_[convHullInds[convHullInds.size()-2]] , scanP_[convHullInds.back()], scanP_[longestRangeIndx]) != 2)
    convHullInds.pop_back();
  
  return convHullInds;
}

// **********************************************************************************
// https://www.geeksforgeeks.org/convex-hull-set-2-graham-scan/
int place_detector::orientation(const pair<double,double>& p, const pair<double,double>& q, const pair<double,double>& r)
{
  int val = (q.second - p.second) * (r.first - q.first) - (q.first - p.first) * (r.second - q.second);
 
  if (val == 0) return 0;  // collinear
    return (val > 0)? 1: 2; // clock or counterclock wise
}

// **********************************************************************************
// the ratio between the area of the block and the area of the circumscribed circle
double place_detector::form_factor(const double& area, const double& circumCircleArea)
{
  if( abs(circumCircleArea) < 1e-6)
    ros_warn("Circum circle area is zero");
  return area / circumCircleArea;
}

// **********************************************************************************
double place_detector::circumscribed_circle_area(const pair<double,double>& cog)
{
  double maxDist = DBL_MIN;

  for(int i=0; i<scanP_.size(); i++)
  {
    double distance = dist( cog, scanP_[i] );

    if(distance > maxDist)
      maxDist = distance;
  }

  return pi_*maxDist*maxDist;
}

// **********************************************************************************
pair<double, double> place_detector::cog()
{
  double cogX = 0, cogY = 0;

  for(int i=0; i<scanP_.size(); i++)
  {
    cogX += scanP_.size() * scanP_[i].first;
    cogY += scanP_[i].second;
  }

  if( scanP_.size() < 1 )
    ros_warn("Scan size is zero");

  cogX = cogX / ( scanP_.size() * scanP_.size() ) ;
  cogY = cogY / scanP_.size();

  return make_pair(cogX, cogY);
}

// **********************************************************************************
// Correct One => https://www.mathworks.com/matlabcentral/fileexchange/33975-the-seven-invariant-moments
// https://towardsdatascience.com/introduction-to-the-invariant-moment-and-its-application-to-the-feature-extraction-ee991f39ec
vector<double> place_detector::seven_invariants(const pair<double,double>& cogVal, vector<double>& secondOrderCentralMoments)
{
  const double mu_0_0 = p_q_th_order_central_moment(0,0, cogVal);//p_q_th_order_central_moment(0,0, cog);

  double mu_2_0 = p_q_th_order_central_moment(2,0, cogVal);
  double mu_0_2 = p_q_th_order_central_moment(0,2, cogVal);
  double mu_1_1 = p_q_th_order_central_moment(1,1, cogVal);
  double mu_1_2 = p_q_th_order_central_moment(1,2, cogVal); 
  double mu_2_1 = p_q_th_order_central_moment(2,1, cogVal);
  double mu_0_3 = p_q_th_order_central_moment(0,3, cogVal);
  double mu_3_0 = p_q_th_order_central_moment(3,0, cogVal);

  double lambda_2_0 = 2, lambda_0_2 = 2;
  double lambda_1_1 = 2;
  double lambda_1_2 = 2.5, lambda_2_1 = 2.5;
  double lambda_0_3 = 2.5, lambda_3_0 = 2.5; 

  if( abs(mu_0_0) < 1e-6 )
    ros_warn("Mu_0_0 is zero");

  double eta_2_0 = mu_2_0/pow(mu_0_0,lambda_2_0);
  double eta_0_2 = mu_0_2/pow(mu_0_0,lambda_0_2);
  double eta_1_1 = mu_1_1/pow(mu_0_0,lambda_1_1);
  double eta_1_2 = mu_1_2/pow(mu_0_0,lambda_1_2);
  double eta_2_1 = mu_2_1/pow(mu_0_0,lambda_2_1);
  double eta_0_3 = mu_0_3/pow(mu_0_0,lambda_0_3);
  double eta_3_0 = mu_3_0/pow(mu_0_0,lambda_3_0); 

  vector<double> moments(7, 0);

  double tA = eta_2_0 + eta_0_2;
  double tAA = eta_2_0 - eta_0_2;
  double tB = eta_3_0 - 3*eta_1_2;
  double tC = 3*eta_2_1 - eta_0_3;
  double tD = eta_3_0 + eta_1_2;
  double tE = eta_2_1 + eta_0_3;
  double tF = eta_3_0 - 3*eta_2_1;
  double tG = eta_3_0 + eta_1_2;
  double tH = eta_3_0 + 3*eta_1_2;

  moments[0] = tA;
  moments[1] = pow(tAA,2) + 4*pow(eta_1_1,2);
  moments[2] = pow(tB,2) + pow(tC,2);
  moments[3] = pow(tD,2 ) + pow(tE,2);
  moments[4] = tF*tG*( pow(tG,2) - 3*pow(tE,2) ) + tC*tE*( 3*pow(tG,2) - pow(tE,2) );
  moments[5] = tAA*( pow(tG,2)-pow(tE,2) ) + 4*eta_1_1*tG*tE;

  moments[6] = tC*tG*( pow(tG,2)-3*pow(tE,2) ) - tH*tE*( 3*pow(tG,2)-pow(tE,2) );

  secondOrderCentralMoments[0] = mu_0_2;
  secondOrderCentralMoments[1] = mu_2_0;
  secondOrderCentralMoments[2] = mu_1_1;

  return moments;  
}

// **********************************************************************************
double place_detector::p_q_th_order_central_moment(const int& p, const int& q, const pair<double,double>& cog)
{
  double sum = 0.0;
  for( int i=0; i<scanP_.size(); i++ )
  {
      //if(p==3 && q == 0)
      //  cout << scanP_[i].first - cog.first << ", " << scanP_[i].second - cog.second << endl;
      sum += ( pow(scanP_[i].first - cog.first,p) * pow(scanP_[i].second - cog.second,q) );
  }
  //cout << endl;
  return sum;
}

// **********************************************************************************
pair<double,double> place_detector::area_perimeter_polygon(double& longestRangeIndx)
{
  pair<double,double> result;
  scanP_.resize(0);

  double sumA = 0.0, sumB = 0.0, perimeter = 0;

  double theta, thetaNxt, xCoord, yCoord, xCoordNxt, yCoordNxt;

  thetaNxt = scanAngleMin_;
  xCoordNxt = scanR_[0]*cos(thetaNxt);
  yCoordNxt = scanR_[0]*sin(thetaNxt);

  scanP_.push_back( make_pair(xCoordNxt, yCoordNxt) );
  longestRangeIndx = scanR_.size() - 1;

  for(int i=0; i<scanR_.size()-1; i++)
  {
    theta = thetaNxt;
    xCoord = xCoordNxt;
    yCoord = yCoordNxt;

    thetaNxt = scanAngleMin_ + scanAngleInc_ * (i+1);
    xCoordNxt = scanR_[i+1]*cos(thetaNxt);
    yCoordNxt = scanR_[i+1]*sin(thetaNxt);

    scanP_.push_back( make_pair(xCoordNxt, yCoordNxt) );

    sumA += ( xCoord * yCoordNxt );
    sumB += ( yCoord * xCoordNxt );
    perimeter +=  dist( make_pair(xCoord, yCoord), make_pair(xCoordNxt, yCoordNxt) );

    if( scanR_[longestRangeIndx] < scanR_[i])
      longestRangeIndx = i;
  }

  theta = thetaNxt;
  xCoord = xCoordNxt;
  yCoord = yCoordNxt;

  thetaNxt = scanAngleMin_;
  xCoordNxt = scanR_[0]*cos(thetaNxt);
  yCoordNxt = scanR_[0]*sin(thetaNxt);

  sumA += ( xCoord * yCoordNxt );
  sumB += ( yCoord * xCoordNxt );
  perimeter += dist( make_pair(xCoord, yCoord), make_pair(xCoordNxt, yCoordNxt) );

  result.first = (sumA - sumB) / 2.0;
  result.second = perimeter;

  return result;
}

// **********************************************************************************
void place_detector::update_feature_vec_a()
{
  ros::Time startTime = ros::Time::now();

  featureVecA_.resize(0);

  pair<double, double> meanSdev = mean_sdev_range_diff(DBL_MAX);

  featureVecA_.push_back(meanSdev.first);
  featureVecA_.push_back(meanSdev.second);

  const double delRange = 5;
  const double maxRange = 50;

  for(double i=delRange; i<=maxRange; i+=delRange)
  {
    pair<double, double> meanSdev = mean_sdev_range_diff(i);

    featureVecA_.push_back(meanSdev.first);
    featureVecA_.push_back(meanSdev.second);
  }

  if(scanR_.size() < 1)
    ros_warn("Scan size is zero");

  featureVecA_.push_back( accumulate(scanR_.begin(), scanR_.end(), 0.0) / scanR_.size() );
  double sqSum = inner_product(scanR_.begin(), scanR_.end(), scanR_.begin(), 0.0);
  double sdev = sqrt(sqSum / scanR_.size() - featureVecA_.back() * featureVecA_.back());
  featureVecA_.push_back(sdev);
  
  const double delGap1 = 1.0;
  const double maxGap1 = 20.0;

  for(double i=delGap1; i<=maxGap1; i+=delGap1)
  {
    int nGaps = n_gaps(i);
    featureVecA_.push_back(nGaps);
  }

  const double delGap2 = 5.0;
  const double maxGap2 = 50.0;
  for(double i=maxGap1+delGap2; i<=maxGap2; i+=delGap2)
  {
    int nGaps = n_gaps(i);
    featureVecA_.push_back(nGaps);
  }

  featureVecAComputeTime_ = ( ros::Time::now() - startTime ).toSec();
}

// **********************************************************************************
int place_detector::n_gaps(const double& thresh)
{
  int nGaps = 0;
  if( abs(scanR_[0] - scanR_.back()) > thresh )
    nGaps++;

  for(int i=0; i<scanR_.size()-1; i++)
    nGaps += (abs( scanR_[i+1] - scanR_[i] ) > thresh);

  return nGaps;
}

// **********************************************************************************
pair<double, double> place_detector::mean_sdev_range_diff(const float& thresh)
{
  vector<double> lenDiff;
  lenDiff.push_back( abs( min(scanR_[0], thresh) - min(scanR_.back(), thresh) ) );

  for(int i=0; i<scanR_.size()-1; i++)
    lenDiff.push_back( abs( min(scanR_[i+1], thresh) - min(scanR_[i], thresh) ) );

  if(lenDiff.size() < 1)
    ros_warn("LenDiff size is zero");

  double mean = accumulate(lenDiff.begin(), lenDiff.end(), 0.0) / lenDiff.size();
  double sqSum = inner_product(lenDiff.begin(), lenDiff.end(), lenDiff.begin(), 0.0);
  double sdev = sqrt(sqSum / lenDiff.size() - mean * mean);

  return make_pair(mean, sdev);
}
// **********************************************************************************
void place_detector::write_feature_vecs_to_file()
{
	dataFile_ << mostRecentLabel_;

	for(int i=0; i<featureVecA_.size(); i++)
		dataFile_ << ", " << to_string(featureVecA_[i]); 

  for(int i=0; i<featureVecB_.size(); i++)
		dataFile_ << ", " << to_string(featureVecB_[i]); 
		
	dataFile_ << endl;

  nFeatureVecsWritten_++;
  ros_info( to_string(nFeatureVecsWritten_) + ": Written feature vector of size " + 
  to_string(featureVecA_.size()) + " + " + to_string(featureVecB_.size()) + " = " + to_string(featureVecA_.size() + featureVecB_.size()) + 
  " in " + to_string(featureVecAComputeTime_*1e3 + featureVecBComputeTime_*1e3) + " ms" );
}

// **********************************************************************************
/*
void update_training_data()
{
  string line, word;
  while( getline(dataFile_, line) )
	{
		stringstream str(line);
 
    getline(str, word, ',');

    if( labelToIndx_.find(word) == labelToIndx_.end() ) // new label found
      labelToIndx_.insert( make_pair(word, labelToIndx_.size()) );

    int labelIndx = labelToIndx_[word];

    vector<double> scanMeas;
		while(getline(str, word, ','))
			scanMeas.push_back( stod(word) );

		trainingData_[labelIndx].push_back(scanMeas);
	}
} */


// **********************************************************************************
template <typename T>
ostream& operator<<(ostream& os, const vector<T>& vecIn)
{
  if(vecIn.size() < 1)
    return os;

  for(int i=0; i<vecIn.size()-1; i++)
    os << vecIn[i] << ", ";

  os << vecIn.back();
  return os;
}

// **********************************************************************************
template <typename T1, typename T2>
ostream& operator<<(ostream& os, const vector<pair<T1,T2>>& vecIn)
{
  if(vecIn.size() < 1)
    return os;

  for(int i=0; i<vecIn.size()-1; i++)
    os << "(" << vecIn[i].first << ", " << vecIn[i].second << "), ";

    os << "(" << vecIn.back().first << ", " << vecIn.back().second << ")";
  return os;
}

// **********************************************************************************
template <typename T1, typename T2>
ostream& operator<<(ostream& os, const pair<T1,T2>& pairIn)
{
  os << "(" << pairIn.first << ", " << pairIn.second << ")";
  return os;
}

// **********************************************************************************
void place_detector::ros_info(const string& s, double throttle_s)
{
  if(throttle_s < 0.0)
	  ROS_INFO("%s: %s", nh_->getNamespace().c_str(), s.c_str());	
  else
    ROS_INFO_THROTTLE(throttle_s, "%s: %s", nh_->getNamespace().c_str(), s.c_str());	
}

// **********************************************************************************
void place_detector::ros_warn(const string& s, double throttle_s)
{
  if(throttle_s < 0.0)
	  ROS_WARN("%s: %s", nh_->getNamespace().c_str(), s.c_str());		
  else
    ROS_WARN_THROTTLE(throttle_s, "%s: %s", nh_->getNamespace().c_str(), s.c_str());
}

// **********************************************************************************