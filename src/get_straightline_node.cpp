#include <ros/ros.h>
#include <tiago_iaslab_simulation/GetStraightline.h>
#include <vector>
#include <random>

GetStraightline::GetStraightline(std::shared_ptr<ros::NodeHandle> nh_ptr)
: nh_ptr_(nh_ptr)
{
  start();
}

void GetStraightline::start(){
  coefficients_server_ = nh_ptr_->advertiseService("/straight_line_coefficients", &GetStraightline::coeffsService, this);
  ROS_INFO_STREAM("Service called!");
}

std::vector<float> GetStraightline::generate_coefficients()
{
    std::random_device rd;
    std::mt19937 gen(rd());

    // Distribuzione per il m tra 0.1 e 10 (float)
    std::uniform_real_distribution<float> d1(0.1f, 10.0f);
    // Distribuzione per il q tra -2.0 e 2.0 (float)
    std::uniform_real_distribution<float> d2(0.0f, 0.3f);

    // Genera i numeri casuali come float
    float m = d1(gen);
    float q = d2(gen);
    std::vector<float> coeffs;
    coeffs.push_back(m);
    coeffs.push_back(q);

    return coeffs;
} 

bool GetStraightline::coeffsService(tiago_iaslab_simulation::Coeffs::Request &req, tiago_iaslab_simulation::Coeffs::Response &res){
    if(!req.ready){
        ROS_ERROR_STREAM("Error! Ready is false. It must be TRUE");
        ros::shutdown();
        return false;
    }
    std::vector<float> coeffs_vec;
    coeffs_vec = GetStraightline::generate_coefficients();
    
    res.coeffs = coeffs_vec;
    return true;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "get_straightline_node");
  auto nh_ptr = std::make_shared<ros::NodeHandle>();

  GetStraightline coeffs(nh_ptr);

  ros::spin();

  return 0;
}


/*x: -0.0006502187255741645
            y: 0.08722621103882858
            z: 0.7083295381429472
          orientation: 
            x: 0.7718361606993515
            y: 0.41887594686395135
            z: -0.22812128634046264
            w: 0.4204432909371797
*/