#include "place_detector.h"

// **********************************************************************************
place_detector::place_detector(ros::NodeHandle* nh)
{
  nh_ = nh;
  test_function();
  return;

  load_params();

  if(dataLabelMode_)
    dataFile_.open(filePath_);

  scanSub_ = nh_->subscribe("scan_in", 1, &place_detector::scan_cb, this);
  labelSub_ = nh_->subscribe("label_in", 1, &place_detector::label_cb, this);

  labelPub_ = nh_->advertise<std_msgs::String>("label_out", 1);
}

// **********************************************************************************
place_detector::~place_detector()
{
  if(dataLabelMode_)
    dataFile_.close();
}
// **********************************************************************************
void place_detector::load_params()
{
  ros_info("Waiting for params to load ...");

  while(!nh_->getParam("data_collect_mode", dataLabelMode_));

  if(dataLabelMode_)
    while(!nh_->getParam("folder_path", filePath_));
  else
    nh_->getParam("folder_path", filePath_);
}
// **********************************************************************************
void place_detector::test_function()
{
  scanR_.resize(0);
  scanR_.push_back(20.5);
  scanR_.push_back(20.5);
  scanR_.push_back(20.5);
  scanR_.push_back(20.5);

  double scanAngleMin_ = pi_/4;
  double scanAngleMax_ = pi_/4 + pi_/2 + pi_/2 + pi_/2;
  double scanAngleInc_ = pi_/2;

  update_feature_vec_a();
  cout << featureVecA_;
  update_feature_vec_b();
  cout << featureVecB_;
}

// **********************************************************************************
void place_detector::label_cb(const std_msgs::String& labelMsg)
{
	mostRecentLabel_ = labelMsg.data;
	mostRecentLabelTime_ = ros::Time::now().toSec();
}

// **********************************************************************************
void place_detector::scan_cb(const sensor_msgs::LaserScan& scanMsg)
{
  if(dataLabelMode_)
    scan_cb_data_label_mode(scanMsg);
  //else ...
}

// **********************************************************************************
void place_detector::scan_cb_data_label_mode(const sensor_msgs::LaserScan& scanMsg)
{
	if(mostRecentLabel_ == "")
		return;

	if( ros::Time::now().toSec() - mostRecentLabelTime_ > 0.2 )
	{
		ros_warn("Most recent label is stale");
		return;
	}

  scanAngleMin_ = scanMsg.angle_min;
  scanAngleMax_ = scanMsg.angle_max;
  scanAngleInc_ = scanMsg.angle_increment;
  scanR_ = scanMsg.ranges;

	dataFile_ << mostRecentLabel_;

  update_feature_vec_a();
  update_feature_vec_b();

	for(int i=0; i<featureVecA_.size(); i++)
		dataFile_ << ", " << to_string(featureVecA_[i]); 

  for(int i=0; i<featureVecB_.size(); i++)
		dataFile_ << ", " << to_string(featureVecB_[i]); 
		
	dataFile_ << endl;
}

// **********************************************************************************
void place_detector::update_feature_vec_b()
{
  featureVecB_.resize(0);

  double longestRangeIndx;
  pair<double,double> area_perimeter = area_perimeter_polygon(longestRangeIndx); // updates scanP_ to be used by later functions
  featureVecB_.push_back(area_perimeter.first);
  featureVecB_.push_back(area_perimeter.second);
  featureVecB_.push_back( area_perimeter.first / area_perimeter.second );

  pair<double,double> cogVal = cog();
  vector<double> secondOrderCentralMoments(3,0);
  vector<double> sevenInvariants = seven_invariants(cogVal, secondOrderCentralMoments);
  featureVecB_.insert( featureVecB_.end(), sevenInvariants.begin(), sevenInvariants.end() );

  featureVecB_.push_back( compactness(area_perimeter.first, area_perimeter.second) );
  featureVecB_.push_back( eccentricity(area_perimeter.first, secondOrderCentralMoments) );

  double circumCircleArea = circumscribed_circle_area(cogVal);
  featureVecB_.push_back( form_factor(area_perimeter.first, circumCircleArea) );

  vector<double> convexHullInds = convex_hull_indices(longestRangeIndx);
  double convexPerimeter = convex_perimeter(convexHullInds);
  featureVecB_.push_back( roundness(area_perimeter.first, convexPerimeter) );
}

// **********************************************************************************
// the ratio of the area of an object to the area of a circle with the same perimeter
double place_detector::compactness(const double& area, const double& perimeter)
{
  return (4*pi_*area) / (perimeter*perimeter);
}

// **********************************************************************************
double place_detector::eccentricity(const double& area, const vector<double>& secondOrderCentralMoments)
{
  return ( pow(secondOrderCentralMoments[0] - secondOrderCentralMoments[1], 2) + 4*secondOrderCentralMoments[2] ) / area;
}

// **********************************************************************************
// the ratio of the area of an object to the area of a circle with the same convex perimeter
double place_detector::roundness(const double& area, const double& convexPerimeter)
{
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

  sum += dist( scanP_.back(), scanP_[0] );

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
    convHullInds.push_back(0);
    return convHullInds;
  }
  if(scanP_.size() == 2)
  {
    convHullInds.push_back(0);
    convHullInds.push_back(1);
    return convHullInds;
  }

  convHullInds.push_back(0);
  convHullInds.push_back(1);
  convHullInds.push_back(2);

  for(int i=3; i<scanP_.size(); i++)
  {
    while (convHullInds.size()>1 && orientation( scanP_[scanP_.size()-2] , scanP_.back(), scanP_[i]) != 2)
         convHullInds.pop_back();
    convHullInds.push_back(i);
  }
  
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
// https://towardsdatascience.com/introduction-to-the-invariant-moment-and-its-application-to-the-feature-extraction-ee991f39ec
pair<double, double> place_detector::cog()
{
  double cogX = 0, cogY = 0;

  for(int i=0; i<scanP_.size(); i++)
  {
    cogX += scanP_.size() * scanP_[i].first;
    cogY += scanP_[i].second;
  }

  cogX = cogX / ( scanP_.size() * scanP_.size() ) ;
  cogY = cogY / scanP_.size();

  return make_pair(cogX, cogY);
}

// **********************************************************************************
vector<double> place_detector::seven_invariants(const pair<double,double>& cog, vector<double>& secondOrderCentralMoments)
{
  const double mu_0_0 = scanP_.size() * scanP_.size();

  double mu_2_0_y = 0, mu_0_2_y = 0;
  double mu_1_1_y = 0;
  double mu_1_2_y = 0, mu_2_1_y = 0;
  double mu_0_3_y = 0, mu_3_0_y = 0; 

  for( int i=0; i<scanP_.size(); i++ )
  {
    int X = scanP_[i].first - cog.first;
    int Y = scanP_[i].second - cog.second;

    mu_2_0_y += pow( Y, 0 ); mu_0_2_y += pow( Y, 2 );
    mu_1_1_y += pow( Y, 1 );
    mu_1_2_y += pow( Y, 2 ); mu_2_1_y += pow( Y, 1 );
    mu_3_0_y += pow( Y, 0 ); mu_0_3_y += pow( Y, 3 );
  }

  double mu_2_0_x = 0, mu_0_2_x = 0;
  double mu_1_1_x = 0;
  double mu_1_2_x = 0, mu_2_1_x = 0;
  double mu_0_3_x = 0, mu_3_0_x = 0; 

  for( int i=0; i<scanP_.size(); i++ )
  {
    int X = scanP_[i].first - cog.first;
    int Y = scanP_[i].second - cog.second;

    mu_2_0_x += pow( X, 2 ); mu_0_2_x += pow( X, 0 );
    mu_1_1_x += pow( X, 1 );
    mu_1_2_x += pow( X, 1 ); mu_2_1_x += pow( X, 2 );
    mu_3_0_x += pow( X, 3 ); mu_0_3_x += pow( X, 0 );
  }

  double mu_2_0 = mu_2_0_x*mu_2_0_y, mu_0_2 = mu_0_2_x*mu_0_2_y;
  double mu_1_1 = mu_1_1_x*mu_1_1_y;
  double mu_1_2 = mu_1_2_x*mu_1_2_y, mu_2_1 = mu_2_1_x*mu_2_1_y;
  double mu_0_3 = mu_0_3_x*mu_0_3_y, mu_3_0 = mu_3_0_x*mu_3_0_y;

  double lambda_2_0 = 2, lambda_0_2 = 2;
  double lambda_1_1 = 2;
  double lambda_1_2 = 2.5, lambda_2_1 = 2.5;
  double lambda_0_3 = 2.5, lambda_3_0 = 2.5; 

  double eta_2_0 = mu_2_0/pow(mu_0_0,lambda_2_0);
  double eta_0_2 = mu_0_2/pow(mu_0_0,lambda_0_2);
  double eta_1_1 = mu_1_1/pow(mu_0_0,lambda_1_1);
  double eta_1_2 = mu_1_2/pow(mu_0_0,lambda_1_2);
  double eta_2_1 = mu_2_1/pow(mu_0_0,lambda_2_1);
  double eta_0_3 = mu_0_3/pow(mu_0_0,lambda_0_3);
  double eta_3_0 = mu_3_0/pow(mu_0_0,lambda_3_0); 

  vector<double> moments(7, 0);

  double tA = eta_2_0 + eta_0_2;
  double tB = eta_3_0 - 3*eta_1_2;
  double tC = 3*eta_2_1 - eta_0_3;
  double tD = eta_3_0 + eta_1_2;
  double tE = eta_2_1 + eta_0_3;

  moments[0] = tA;
  moments[1] = pow(tA,2) + 4*pow(eta_1_1,2);
  moments[2] = pow(tB,2) + pow(tC,2);
  moments[3] = pow(tD,2 ) + pow(tE,2);
  moments[4] = tC*tE*( pow(tD,2)-pow(tE,2) ) + tB*tD*( pow(tD,2) - 3*pow(tE,2) );
  moments[5] = tC*( pow(tD,2)-pow(tE,2) ) + 4*eta_1_1*tD*tE;
  moments[6] = tC*tD*( pow(tD,2)-3*pow(tE,2) ) - tB*tE*( 3*pow(tD,2)-pow(tE,2) );

  secondOrderCentralMoments[0] = mu_0_2;
  secondOrderCentralMoments[1] = mu_2_0;
  secondOrderCentralMoments[2] = mu_1_1;

  return moments;  
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
  longestRangeIndx = scanR_.back();

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
    perimeter += sqrt ( pow(xCoordNxt - xCoord, 2) + pow(yCoordNxt - yCoord, 2) );

    if(longestRangeIndx < scanR_[i])
      longestRangeIndx = scanR_[i];
  }

  theta = thetaNxt;
  xCoord = xCoordNxt;
  yCoord = yCoordNxt;

  thetaNxt = scanAngleMin_;
  xCoordNxt = scanR_[0]*cos(theta);
  yCoordNxt = scanR_[0]*sin(theta);

  sumA += ( xCoord * yCoordNxt );
  sumB += ( yCoord * xCoordNxt );
  perimeter += sqrt ( pow(xCoordNxt - xCoord, 2) + pow(yCoordNxt - yCoord, 2) );

  result.first = (sumA - sumB) / 2.0;
  result.second = perimeter;

  return result;
}

// **********************************************************************************
void place_detector::update_feature_vec_a()
{
  featureVecA_.resize(0);;

  pair<double, double> meanSdev = mean_sdev_range_diff(DBL_MAX);

  featureVecA_.push_back(meanSdev.first);
  featureVecA_.push_back(meanSdev.second);

  const double delRange = 5;
  const double maxRange = 50;

  for(double i=0; i<maxRange; i+=delRange)
  {
    pair<double, double> meanSdev = mean_sdev_range_diff(i);

    featureVecA_.push_back(meanSdev.first);
    featureVecA_.push_back(meanSdev.second);
  }

  featureVecA_.push_back( accumulate(scanR_.begin(), scanR_.end(), 0.0) / scanR_.size() );
  double sqSum = inner_product(scanR_.begin(), scanR_.end(), scanR_.begin(), 0.0);
  double sdev = sqrt(sqSum / scanR_.size() - featureVecA_.back() * featureVecA_.back());
  featureVecA_.push_back(sdev);

  const double delGap1 = 0.5;
  const double maxGap1 = 20;

  for(double i=0; i<maxGap1; i+=delGap1)
  {
    int nGaps = n_gaps(i);
    featureVecA_.push_back(nGaps);
  }

  const double delGap2 = 5;
  const double maxGap2 = 50;
  for(double i=maxGap1; i<maxGap2; i+=delGap2)
  {
    int nGaps = n_gaps(i);
    featureVecA_.push_back(nGaps);
  }
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
  lenDiff.push_back( abs(scanR_[0] - scanR_.back()) );

  for(int i=0; i<scanR_.size()-1; i++)
    lenDiff.push_back( abs( min(scanR_[i+1], thresh) - min(scanR_[i], thresh) ) );

  double mean = accumulate(lenDiff.begin(), lenDiff.end(), 0.0) / lenDiff.size();
  double sqSum = inner_product(lenDiff.begin(), lenDiff.end(), lenDiff.begin(), 0.0);
  double sdev = sqrt(sqSum / lenDiff.size() - mean * mean);

  return make_pair(mean, sdev);
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
}
// **********************************************************************************
void write_feature_set_to_file()
{
	dataFile_.open(folderPath+"data_file.csv");
	dataFile_.close();
}
*/

// **********************************************************************************
ostream& operator<<(ostream& os, const vector<double>& vecIn)
{
  if(vecIn.size() < 1)
    return os;

  for(int i=0; i<vecIn.size()-1; i++)
    os << vecIn[i] << ", ";

  os << vecIn.back();
  return os;
}

// **********************************************************************************
void place_detector::ros_info(const string& s)
{
	ROS_INFO("%s: %s", nh_->getNamespace().c_str(), s.c_str());	
}

// **********************************************************************************
void place_detector::ros_warn(const string& s)
{
	ROS_WARN("%s: %s", nh_->getNamespace().c_str(), s.c_str());		
}

// **********************************************************************************