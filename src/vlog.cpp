#include "vlog.h"


#define PI 3.1428

using namespace std;
using namespace cv;
using namespace Eigen;

GRANSAC::VPFloat Slope(int x0, int y0, int x1, int y1)
{
    return (GRANSAC::VPFloat)(y1-y0)/(x1-x0);
}

double cross_product( cv::Point a, cv::Point b ){
   return a.x*b.y - a.y*b.x;
}

struct ExponentialResidual {
  ExponentialResidual(double x, double y, double m, double f)
      : x_(x), y_(y), m_(m),  f_(f){}
  template <typename T> bool operator()(const T* const phi,
                                        const T* const o,
                                        const T* const h,
                                        const T* const i,
                                        T* residual) const {
    T phip = phi[0]*T(3.14/180.0);
    T si = o[0]+T(m_)*(floor(i[0])+T(x_));
    T thetai = atan(si/h[0]);
    T y = (si*cos(thetai)*f_)/(cos(thetai-phip)*h[0]*cos(phip)*cos(phip));
    residual[0] = y_ - y;
    return true;
  }
 private:
  const double x_;
  const double y_;
  const double m_;
  const double f_;
};

double distance_to_line(cv::Point size, cv::Point begin, cv::Point end, cv::Point x ){
   //translate the begin to the origin
  GRANSAC::VPFloat slope = Slope(begin.x, begin.y, end.x, end.y);

  cv::Point p(0,0), q(size.x, size.y);

  p.y = -(begin.x - p.x) * slope + begin.y;
  q.y = -(end.x - q.x) * slope + end.y;

   q -= p;
   x -= p;

   //¿do you see the triangle?
   double area = cross_product(x, q);
   return area / norm(q);
}

void DrawFullLine(cv::Mat& img, cv::Point a, cv::Point b, cv::Scalar color, int LineWidth)
{
    GRANSAC::VPFloat slope = Slope(a.x, a.y, b.x, b.y);

    cv::Point p(0,0), q(img.cols, img.rows);

    p.y = -(a.x - p.x) * slope + a.y;
    q.y = -(b.x - q.x) * slope + b.y;

    cv::line(img, p, q, color, LineWidth, 8, 0);
}

vlog::vlog(ros::NodeHandle innh, TopicNames intn, ConfigSettings incs)
{
  s_tn = intn;
  s_nh = innh;
  s_cs = incs;
  num_pc_points = 0;

  h1 = 26; s1 = 20; v1 = 82;
  th = 50;
  lines = 60;
  if (s_cs.debug) namedWindow("sliders");
  if (s_cs.debug) namedWindow("graph");
  hough_rho = 1; hough_theta = 1; hough_threshold = 1;
  m_px = 0;
  m_py = 0;
  m_pox = -1;
  m_poy = -1;
  m_pvx = 0;
  m_pvy = 0;
  m_ofx = 0;
  m_ofy = 0;
  first_run = true;
  gtfr = false;

  myfile.open(s_cs.outFileName, ios::app);
  google::InitGoogleLogging("localizer");
  
  string imageTopicName = (s_cs.useFrontImage)?s_tn.inFrontImage:s_tn.inBottomImage;
  string infoTopicName = (s_cs.useFrontImage)?s_tn.inFrontInfo:s_tn.inBottomInfo;
  imagenew_sub = s_nh.subscribe(imageTopicName, 1, &vlog::callback, this);
  gt_sub = s_nh.subscribe("/albatross/ground_truth/odometry", 1, &vlog::gtcallback, this);

  // image_sub = new message_filters::Subscriber<sensor_msgs::Image>(s_nh, imageTopicName, 1);
  // info_sub = new message_filters::Subscriber<nav_msgs::Odometry>(s_nh, "/albatross/ground_truth/odometry", 1);

  // sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10),
  //                                                        *image_sub,
  //                                                        *info_sub);
  // sync->registerCallback(boost::bind(&vlog::callback, this, _1, _2));
}

vlog::~vlog(){
  //delete sync;
  //delete imagenew_sub;
  //delete info_sub;
}

void vlog::gtcallback(const nav_msgs::OdometryConstPtr& odom){
  gtfr=true;
  m_odom = *odom;
}

void vlog::callback(const sensor_msgs::ImageConstPtr& left_image){
  //cout << "Cb" << endl;
  cv_bridge::CvImagePtr left_cv_ptr;
  try{
    left_cv_ptr = cv_bridge::toCvCopy(left_image, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  // End Data Check

  Mat hsv_image, canny;
  cvtColor(left_cv_ptr->image, hsv_image, CV_BGR2HSV);
  inRange(hsv_image,Scalar(h1 - th, s1 - th, 0), Scalar(h1 + th, s1 + th, 255), hsv_image);
  blur( hsv_image, hsv_image, Size(3,3) );    
  Canny(hsv_image, canny, 100, 300);

  if(s_cs.debug){
    createTrackbar( "Rho:", "sliders", &hough_rho, 710);
    createTrackbar( "Theta:", "sliders", &hough_theta, 628);   
    createTrackbar( "Threshold:", "sliders", &hough_threshold, 710);
  }

  //imshow("image", canny);
  processImage(left_cv_ptr->image, canny);
}


void vlog::publisher(){
}

void vlog::processImage(Mat &display, Mat &canny){
  vector<Vec2f> houghLines, houghLines2, houghLinesf;
  vector<vector<Vec2f> > gridLines;
  Mat graph;
  float line_theta [2] = {0};
  if (s_cs.debug) graph = Mat(628, 710, CV_8UC1, Scalar(0));
  HoughLines(canny, houghLines, 1, (float)1/100, 145, 0, 0 );
  //filterLines(houghLines, houghLinesf);
  houghLinesf = houghLines;

  if (s_cs.debug) {
    //display lines
    for( size_t i = 0; i < houghLinesf.size(); i++ ){
        float rho = houghLinesf[i][0];
        float theta = houghLinesf[i][1];
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        Point pt1(cvRound(x0 + 1000*(-b)),
                cvRound(y0 + 1000*(a)));
        Point pt2(cvRound(x0 - 1000*(-b)),
                cvRound(y0 - 1000*(a)));
        line(display, pt1, pt2, Scalar(0,0,255), 3, 8 );
        if (rho < 0) {
          rho = - rho;
          theta = PI - theta;
        }
        graph.at<uchar>(theta * 100, rho) = 255;
    }
  }

  //RANSAC
  int Side = 710;
  cv::Mat Canvas;
  if (s_cs.debug) Canvas = cv::Mat(Side, Side, CV_8UC3);
  if (s_cs.debug) Canvas.setTo(255);
  std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> CandPoints;
  for( size_t i = 0; i < houghLinesf.size(); i++)
  {
    float rho = houghLinesf[i][0];
    float theta = houghLinesf[i][1];
    if (rho < 0) {
      rho = - rho;
      theta = PI - theta;
    }
    std::shared_ptr<GRANSAC::AbstractParameter> CandPt = std::make_shared<Point2D>(rho, (theta+PI/2)*100);
    CandPoints.push_back(CandPt);
    //circle(Canvas, Point(rho/5, (theta+PI/2)*100), 2, cv::Scalar(0), -1);
  }
  
  GRANSAC::RANSAC<Line2DModel, 2> Estimator;
  Estimator.Initialize(20, 200); // Threshold, iterations
  for (int i = 0; i < 2; i++){
    std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> newCandPoints;
    
    if (i == 1){
    for (int j = 0; j < CandPoints.size(); j++){
      auto RPt = std::dynamic_pointer_cast<Point2D>(CandPoints[j]);
      if (s_cs.debug) cv::circle(Canvas, cv::Point(RPt->m_Point2D[0], RPt->m_Point2D[1]), floor(Side / 200), cv::Scalar(0*200, 255, 0), -1);
    }}


    int start = cv::getTickCount();
    Estimator.Estimate(CandPoints);
    int end = cv::getTickCount();

    auto BestLine = Estimator.GetBestModel();
    if(BestLine)
    {
      auto BestLinePt1 = std::dynamic_pointer_cast<Point2D>(BestLine->GetModelParams()[0]);
      auto BestLinePt2 = std::dynamic_pointer_cast<Point2D>(BestLine->GetModelParams()[1]);
      if(BestLinePt1 && BestLinePt2)
      {
        cv::Point Pt1(BestLinePt1->m_Point2D[0], BestLinePt1->m_Point2D[1]);
        cv::Point Pt2(BestLinePt2->m_Point2D[0], BestLinePt2->m_Point2D[1]);
        line_theta[i] = float(Pt1.y+Pt2.y)/2;
        if (s_cs.debug) DrawFullLine(Canvas, Pt1, Pt2, cv::Scalar(i*200, 0, 255), 2);
        float distanceThreshold = 20;
        vector<Vec2f> tempLines;
        for (int j = 0; j < CandPoints.size(); j++){
          auto RPt = std::dynamic_pointer_cast<Point2D>(CandPoints[j]);
          float temp_dist = distance_to_line(cv::Point(Side, Side), Pt1, Pt2, cv::Point(RPt->m_Point2D[0], RPt->m_Point2D[1]));
          
          if (abs(temp_dist) < distanceThreshold){ // inliner
            tempLines.push_back(Vec2f(RPt->m_Point2D[0], RPt->m_Point2D[1]));
          }
          else { // outliner
            std::shared_ptr<GRANSAC::AbstractParameter> CandPt = std::make_shared<Point2D>(RPt->m_Point2D[0], RPt->m_Point2D[1]);
            newCandPoints.push_back(CandPt);
          }

        }
        gridLines.push_back(tempLines);
        CandPoints = newCandPoints;
      } 
    }
  }
  //Ransac Ends

  double phix, ox, hx, phiy, oy, hy, ix, iy;
  int idx = (abs(line_theta[0]-(PI*100)) < abs(line_theta[1]-(PI*100)))?1:0;
  int idy = (idx==1)?0:1;
  ceres::Solver::Summary summary;

  optimize(gridLines[idx], display.cols/2, phix, ox, hx, ix, summary);
  filter(ox, m_px, m_pox, m_pvx);
  optimize(gridLines[idy], display.rows/2, phiy, oy, hy, iy, summary);
  filter(oy, m_py, m_poy, m_pvy);

  if (s_cs.debug) cout << m_px << " " << m_py << endl;
  if(gtfr && first_run){m_ofx = m_odom.pose.pose.position.y-m_px; m_ofy = m_odom.pose.pose.position.x-m_py; first_run=false;}
  //m_px += m_ofx; m_py += m_ofy;}
  if(gtfr){
    //double x_error = m_odom.pose.pose.position.y-m_px-m_ofx; double y_error = m_odom.pose.pose.position.x-m_py-m_ofy;
    myfile << m_px+m_ofx << " " << m_py+m_ofy << " " << m_odom.pose.pose.position.y << " " << m_odom.pose.pose.position.x << " " << summary.iterations.size() << " " << hx << " " << hy << " " << m_odom.pose.pose.position.z << endl;
  }
  
  if (s_cs.debug){
    imshow("image", display);
    imshow("graph", graph);
    imshow("RANSAC", Canvas);
    waitKey(1);
  }
}

void vlog::filterLines(vector<Vec2f> inlines, vector<Vec2f> & outlines){
  //vector<Vec2f> outlines;
  int loop_no = inlines.size();
  float threshold = 7;
  for( size_t i = 0; i < loop_no; i++ )
  {
    float rho1 = inlines[i][0];
    float theta1 = inlines[i][1];
    bool flag_write = true; 

    for (int j = 0; j < outlines.size();j++)
    {
      float rho2 = outlines[j][0], theta2 = outlines[j][1];
      if(  norm(Point(rho1, theta1) - Point(rho2, theta2)) < threshold)
      {
        flag_write = false;
      }
    }         
    if(flag_write == true)
    {
      outlines.push_back(Vec2f(rho1,theta1));          
    }     
  }
}

void vlog::optimize(vector<Vec2f>& gridLines, int center_rho, double & out_phi, double & out_o, double & out_h, double & out_i, ceres::Solver::Summary & summary){
  //cout << "op" << endl;
  float ox = -1.0;
  vector<Vec2f> uniqueGridLines;
  std::vector<int> is;

  // Sort GridLines
  std::sort(gridLines.begin(), gridLines.end(), [](const cv::Vec2f &a, const cv::Vec2f &b) {
    return ( a[0] < b[0]);
  });

  // Start Clusturing + ID assignment
    //find out max and min diff
  float mean_rho = gridLines[0][0];
  float mean_theta = gridLines[0][1];
  int zero_pos = -1; int continous_counter = 0;
  int counter = 1;
  int start_diff = gridLines[0][0];
  //cout << gridLines[0][0] << " ";
  for (int i = 1; i < gridLines.size(); i++){
    //cout << gridLines[i][0] << " ";
    int diff = gridLines[i][0] - gridLines[i-1][0];
    if (diff > (gridLines[i-1][0]-start_diff)*2 && diff > 10){
      //break
      float new_rho = (mean_rho/counter)-center_rho;
      uniqueGridLines.push_back(Vec2f(new_rho, mean_theta/counter));
      counter = 1;
      mean_rho = gridLines[i][0];
      mean_theta = gridLines[i][1];
      start_diff = gridLines[i][0];
      if (new_rho>0 && zero_pos == -1) zero_pos = continous_counter;
      continous_counter++;
    }
    else{
      mean_rho += gridLines[i][0];
      mean_theta += gridLines[i][1];
      counter++;
    }
  }
  //cout << endl;

  float new_rho = (mean_rho/counter)-center_rho;
  uniqueGridLines.push_back(Vec2f(new_rho, mean_theta/counter));
  if (new_rho>0 && zero_pos == -1) zero_pos = continous_counter;
  for (int i=0; i<uniqueGridLines.size(); i++){
    is.push_back(i-zero_pos);
    //cout << uniqueGridLines[i][0] << "," << i-zero_pos << " ";
  }
  //cout << endl << endl;
  // End Clusturing + ID assignment

  /*
  // Naive Optimization
  float lb[4] = {-45.0,0.1,2.8,-3};
  float ub[4] = {45.0,0.9,4,3};
  float sb[4] = {2,0.1,0.1,1};
  int l1 = ((ub[0] - lb[0]) / sb[0]) + 1;
  int l2 = ((ub[1] - lb[1]) / sb[1]) + 1;
  int l3 = ((ub[2] - lb[2]) / sb[2]) + 1;
  int l4 = ((ub[3] - lb[3]) / sb[3]) + 1;
  double min_cost = 99999999;
  float phi1, o1, h1;
  int start_i;
  for (int iter_i = 0; iter_i<l1; iter_i++){
    for (int iter_j = 0; iter_j<l2; iter_j++){
      for (int iter_k = 0; iter_k<l3; iter_k++){
        for (int iter_l = 0; iter_l<l4; iter_l++){
          double temp_costs = getCost(lb[0]+iter_i*sb[0], lb[1]+iter_j*sb[1], lb[2]+iter_k*sb[2], lb[3]+iter_l*sb[3], uniqueGridLines, is);
          if (temp_costs < min_cost){
            min_cost = temp_costs;
            phi1 = lb[0]+iter_i*sb[0];
            o1 = lb[1]+iter_j*sb[1];
            h1 = lb[2]+iter_k*sb[2];
          }
        }
      }
    }
  }
  o = o1;
  phi = phi1;
  h = h1;*/

  // Ceres optimizer
  double phi = 0.0;
  double o = 0.5;
  double h = 1.0;
  double start_i = 0.0;
  ceres::Problem problem;
  for (int i = 0; i < uniqueGridLines.size(); ++i) {
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<ExponentialResidual, 1, 1, 1, 1, 1>(
            new ExponentialResidual(double(is[i]), double(uniqueGridLines[i][0]), s_cs.m, s_cs.f)),
        NULL,
        &phi, &o, &h, &start_i);
  }
  
  problem.SetParameterLowerBound(&phi, 0, -45.0);
  problem.SetParameterLowerBound(&o, 0, 0.0);
  problem.SetParameterLowerBound(&h, 0, 0.01);
  problem.SetParameterLowerBound(&start_i, 0, -15.0);
  problem.SetParameterUpperBound(&phi, 0, 45.0);
  problem.SetParameterUpperBound(&o, 0, 1);
  problem.SetParameterUpperBound(&start_i, 0, 15.0);

  ceres::Solver::Options options;
  options.max_num_iterations = 100;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.minimizer_progress_to_stdout = false;
  //ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  out_o = o;
  out_phi = phi;
  out_h = h;
  out_i = start_i;
  //std::cout << "Final " << phi << " " << o << " " << h << " " << start_i << "\n";
}

double vlog::getSpppi(double phi, double o, double m, int i, double h, double f){
  // f = 323.451202597659
  // h = 2
  // m = 1
  // o = 0.5 
  phi = phi*PI/180;
  double si = o+m*i;
  double thetai = atan(si/h);
  double Spppi = (si*cos(thetai)*f)/(cos(thetai-phi)*h*cos(phi)*cos(phi));
  return Spppi;
}

double vlog::getCost(double phi, double o, double h, int start_i,  vector<Vec2f> & uniqueGridLines, vector<int> & is){
  double cost = 0;
  int l = is.size();
  for (int index = 0; index < l; index++){
    double Yp = getSpppi(phi, o, s_cs.m, is[index+start_i], h, s_cs.f); 
    cost = cost + (abs(Yp - uniqueGridLines[index][0]));
  }
  cost = cost / (2*l);
  return cost;
}

float vlog::square(float input){
  return input*input;
}

void vlog::filter(float ox, float & px, float & pox, float & vpx){
  //cout << "fl" << endl;
  ox = 1-ox;
  if(pox != -1){
    float temp_px = (square(px-(px+ox-pox)) < square(px-(px+(ox-1-pox))))? px+ox-pox : px+(ox-1-pox);
    temp_px = (square(px-temp_px) < square(px-(px+ox+1-pox)))? temp_px : px+ox+1-pox;
    vpx = temp_px-px;
    //cout << vpx << endl;
    if (abs(vpx)>0.33) return;
    px = temp_px;
  }
  else {
    vpx = ox;
    px = ox;
  }

  pox = ox;
  //cout << px << endl;
}


