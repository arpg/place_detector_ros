#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "fstream"

using namespace std;
	
ros::NodeHandle* nh_;

void ros_info(const string& s);
void ros_warn(const string& s);

ifstream dataFile_;
map<string, int> labelToIndx_;

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
vector<double> feature_set_a(const vector<double>& scanMeas)
{
  vector<double> featureVec;

  pair<double, double> meanSdev = mean_sdev_range_diff(scanMeas, DBL_MAX);

  featureVec.push_back(meanSdev.first);
  featureVec.push_back(meanSdev.second);

  const double delRange = 5;
  const double maxRange = 50;

  for(int i=0; i<maxRange; i+=delRange)
  {
    pair<double, double> meanSdev = mean_sdev_range_diff(scanMeas, i);

    featureVec.push_back(meanSdev.first);
    featureVec.push_back(meanSdev.second);
  }

  featureVec.push_back( accumulate(scanMeas.begin(), scanMeas.end(), 0.0) / scanMeas.size() );
  double sqSum = inner_product(scanMeas.begin(), scanMeas.end(), scanMeas.begin(), 0.0);
  double sdev = sqrt(sqSum / scanMeas.size() - mean * mean);
  featureVec.push_back(sdev);

  const int delGap = 1;
  const int maxGap = 20;

  for(int i=0; i<maxGap; i+=delGap)
  {
    int nGaps = n_gaps(scanMeas, i);
    featureVec.push_back(nGaps);
  }


}

// **********************************************************************************
int n_gaps(const vector<double>& scanMeas, const int& thresh)
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