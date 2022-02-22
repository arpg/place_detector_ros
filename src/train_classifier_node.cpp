#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "fstream"

using namespace std;
	
ros::NodeHandle* nh_;

void ros_info(const string& s);
void ros_warn(const string& s);

ifstream dataFile_;
map<string, int> labelToIndx_;

double scanAngleMin_ = 0;
double scanAngleMax_ = 0;
double scanAngleInc_ = 0;

vector<vector<vector<double>>> trainingData_;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "scan_labeller");
	nh_ = new ros::NodeHandle(ros::this_node::getName());

  ros_info("Waiting for params to load ...");

  string folderPath;
  while(!nh_->getParam("folder_path", folderPath));

	dataFile_.open(folderPath+"data_file.csv");
	
	

	dataFile_.close();
	return 0;
}
// **********************************************************************************
vector<double> feature_set_b(const vector<double>& scanMeas)
{
  vector<double> featureSet;

  pair<double,double> area_perimeter = area_perimeter_polygon(scanMeas);
  featureSet.push_back(area_perimeter.first);
  featureSet.push_back(area_perimeter.second);

  featureSet.push_back( area_perimeter.first / area_perimeter.second );


}

// **********************************************************************************
// https://towardsdatascience.com/introduction-to-the-invariant-moment-and-its-application-to-the-feature-extraction-ee991f39ec
pair<double, double> cog(const vector<pair<double,double>>& scanMeasCoords)
{
  double cogX = 0, cogY = 0;

  for(int i=0; i<scanMeasCoords; i++)
  {
    cogX += scanMeasCoords.size() * scanMeasCoords[i].first;
    cogY += scanMeasCoords[i].second;
  }

  cogX = cogX / ( scanMeasCoords.size() * scanMeasCoords.size() ) ;
  cogY = cogY / scanMeasCoords.size();

  return make_pair(cogX, cogY);
}

// **********************************************************************************
vector<double> normalized_central_moments(const pair<double,double>& cog, const vector<pair<double,double>>& scanMeasCoords)
{
  const double mu_0_0 = scanMeasCoords.size() * scanMeasCoords.size();

  double mu_2_0_y = 0, mu_0_2_y = 0;
  double mu_1_1_y = 0;
  double mu_1_2_y = 0, mu_2_1_y = 0;
  double mu_0_3_y = 0, mu_3_0_y = 0; 

  for( int i=0; i<scanMeasCoords.size(); i++ )
  {
    int X = scanMeasCoords[i].first - cog.first;
    int Y = scanMeasCoords[i].second - cog.second;

    mu_2_0_y += pow( Y, 0 ); mu_0_2_y += pow( Y, 2 );
    mu_1_1_y += pow( Y, 1 );
    mu_1_2_y += pow( Y, 2 ); mu_2_1_y += pow( Y, 1 );
    mu_3_0_y += pow( Y, 0 ); mu_0_3_y += pow( Y, 3 );
  }

  double mu_2_0_x = 0, mu_0_2_x = 0;
  double mu_1_1_x = 0;
  double mu_1_2_x = 0, mu_2_1_x = 0;
  double mu_0_3_x = 0, mu_3_0_x = 0; 

  for( int i=0; i<scanMeasCoords.size(); i++ )
  {
    int X = scanMeasCoords[i].first - cog.first;
    int Y = scanMeasCoords[i].second - cog.second;

    mu_2_0_x += pow( X, 2 ); mu_0_2_x += pow( X, 0 );
    mu_1_1_x += pow( X, 1 );
    mu_1_2_x += pow( X, 1 ); mu_2_1_x += pow( X, 2 );
    mu_3_0_x += pow( X, 3 ); mu_0_3_x += pow( X, 0 );
  }

  double lamda_2_0 = 2, lamda_0_2 = 2;
  double lamda_1_1 = 2;
  double lamda_1_2 = 2.5, lamda_2_1 = 2.5;
  double lamda_0_3 = 2.5, lamda_3_0 = 2.5; 

  double eta_2_0 = mu_2_0/pow(mu_0_0,lambda_2_0), eta_0_2 = mu_0_2/pow(mu_0_0,lambda_0_2);
  double eta_1_1 = mu_1_1/pow(mu_0_0,lambda_1_1);
  double eta_1_2 = mu_1_2/pow(mu_0_0,lambda_1_2), eta_2_1 = mu_2_1/pow(mu_0_0,lambda_2_1);
  double eta_0_3 = mu_0_3/pow(mu_0_0,lambda_0_3), eta_3_0 = mu_3_0/pow(mu_0_0,lambda_3_0); 

  vector<double> moments;

  double tA = eta_2_0 + eta_0_2;
  double tB = eta_3_0 - 3*eta_1_2;
  double tC = 3*eta_2_1 - eta_0_3;
  double tD = eta_3_0 + eta_1_2;
  double tE = eta_2_1 + eta_0_3;

  moments.push_back( tA );
  moments.push_back( pow(tA,2) + 4*pow(eta_1_1,2) );
  moments.push_back( pow(tB,2) + pow(tC,2) );
  moments.push_back( pow(tD,2 ) + pow(tE,2) );
  moments.push_back( tC*tE*( pow(tD,2)-pow(tE,2) ) + tB*tD*( pow(tD,2) - 3*pow(tE,2) ) );
  moments.push_back( tC*( pow(tD,2)-pow(tE,2) ) + 4*eta_1_1*tD*tE );
  moments.push_back( tC*tD*( pow(tD,2)-3*pow(tE,2) ) - tB*tE*( 3*pow(tD,2)-pow(tE,2) ) );

  return moments;  
}

// **********************************************************************************
pair<double,double> area_perimeter_polygon(const vector<double>& scanMeas, vector<pair<double,double>>& scanMeasCoords )
{
  pair<double,double> result;

  double sumA = 0.0, sumB = 0.0, perimeter = 0;

  double theta, xCoord, yCoord, xCoordNxt, yCoordNxt;

  thetaNxt = scanAngleMin_;
  xCoordNxt = scanMeas[0]*cos(theta);
  yCoordNxt = scanMeas[0]*sin(theta);

  scanMeasCoords.push_back( make_pair(xCoordNxt. yCoordNxt) );

  for(int i=0; i<scanMeas.size()-1; i++)
  {
    theta = thetaNxt;
    xCoord = xCoordNxt;
    yCoord = yCoordNxt;

    thetaNxt = scanAngleMin_ + scanAngleInc_ * (i+1);
    xCoordNxt = scanMeas[i+1]*cos(theta);
    yCoordNxt = scanMeas[i+1]*sin(theta);

    scanMeasCoords.push_back( make_pair(xCoordNxt. yCoordNxt) );

    sumA += ( xCoord * yCoordNxt );
    sumB += ( yCoord * xCoordNxt );
    perimeter += sqrt ( pow(xCoordNxt - xCoord, 2) + pow(yCoordNxt - yCoord, 2) );
  }

  theta = thetaNxt;
  xCoord = xCoordNxt;
  yCoord = yCoordNxt;

  thetaNxt = scanAngleMin_;
  xCoordNxt = scanMeas[0]*cos(theta);
  yCoordNxt = scanMeas[0]*sin(theta);

  sumA += ( xCoord * yCoordNxt );
  sumB += ( yCoord * xCoordNxt );
  perimeter += sqrt ( pow(xCoordNxt - xCoord, 2) + pow(yCoordNxt - yCoord, 2) );

  result.first = (sumA - sumB) / 2.0;
  result.second = perimeter;

  return result;
}

// **********************************************************************************
vector<double> feature_set_a(const vector<double>& scanMeas)
{
  vector<double> featureVec;

  pair<double, double> meanSdev = mean_sdev_range_diff(scanMeas, DBL_MAX);

  featureVec.push_back(meanSdev.first);
  featureVec.push_back(meanSdev.second);

  const double delRange = 5;
  const double maxRange = 50;

  for(double i=0; i<maxRange; i+=delRange)
  {
    pair<double, double> meanSdev = mean_sdev_range_diff(scanMeas, i);

    featureVec.push_back(meanSdev.first);
    featureVec.push_back(meanSdev.second);
  }

  featureVec.push_back( accumulate(scanMeas.begin(), scanMeas.end(), 0.0) / scanMeas.size() );
  double sqSum = inner_product(scanMeas.begin(), scanMeas.end(), scanMeas.begin(), 0.0);
  double sdev = sqrt(sqSum / scanMeas.size() - mean * mean);
  featureVec.push_back(sdev);

  const double delGap1 = 0.5;
  const double maxGap1 = 20;

  for(double i=0; i<maxGap1; i+=delGap1)
  {
    int nGaps = n_gaps(scanMeas, i);
    featureVec.push_back(nGaps);
  }

  const double delGap2 = 5;
  const double maxGap2 = 50;
  for(double i=maxGap1; i<maxGap2; i+=delGap2)
  {
    int nGaps = n_gaps(scanMeas, i);
    featureVec.push_back(nGaps);
  }
}

// **********************************************************************************
int n_gaps(const vector<double>& scanMeas, const double& thresh)
{
  int nGaps = 0;
  if( abs(scanMeas[0] - scanMeas.back()) > thresh )
    nGaps++;

  for(int i=0; i<scanMeas.size()-1; i++)
    nGaps += (abs( scanMeas[i+1] - scanMeas[i] ) > thresh);

  return nGaps;
}

// **********************************************************************************
pair<double, double> mean_sdev_range_diff(const vector<double>& scanMeas, const double& thresh)
{
  vector<double> lenDiff;
  lenDiff.push_back( abs(scanMeas[0] - scanMeas.back()) );

  for(int i=0; i<scanMeas.size()-1; i++)
    lenDiff.push_back( abs( min(scanMeas[i+1], thresh) - min(scanMeas[i], thresh) ) );

  double mean = accumulate(lenDiff.begin(), lenDiff.end(), 0.0) / lenDiff.size();
  double sqSum = inner_product(lenDiff.begin(), lenDiff.end(), lenDiff.begin(), 0.0);
  double sdev = sqrt(sqSum / lenDiff.size() - mean * mean);

  return make_pair(mean, sdev);
}

// **********************************************************************************
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