#include "place_detector.h"

using namespace std;
	
int main(int argc, char** argv)
{
	ros::init(argc, argv, "place_detector");
	ros::NodeHandle nh(ros::this_node::getName());
	
	place_detector placeDetector(&nh);
	
	ros::spin();

	return 0;
}