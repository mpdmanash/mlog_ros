#include <ros/ros.h>
#include "vlog.h"
#include <iostream>
#include <string>
using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "localize");
  ros::NodeHandle nh;

  
  TopicNames tn;
  ConfigSettings cs;
  //cs.f = 374.6706070969281;
  // tn.inBottomImage = "/ardrone/bottom/image_raw";
  tn.inBottomImage = "/image";
  // tn.inFrontImage = "/ardrone/front/image_raw";
  // tn.inBottomInfo = "/ground_truth/state";
  // tn.inFrontInfo = "/ardrone/front/camera_info";

  cs.outFileName = argv[1];
  if (!strcmp (argv[2],"front")){
    cs.useFrontImage=true;
    cout << "Front Image\n";
  }
  else{
    cs.useFrontImage=false;
    cout << "Bottom Image\n";
  }
  if (!strcmp(argv[3],"debug")){
    cs.debug=true;
    cout << "Debug true\n";
  }
  else{
    cs.debug=false;
    cout << "Debug false\n";
  }

  vlog s2o(nh, tn, cs);
  ros::spin();

  return 0;
}