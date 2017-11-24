/*
 * LocationApplication.h
 *
 *  Created on: Nov 7, 2017
 *      Author: root
 */

#ifndef _LOCATIONAPPLICATION_H_
#define _LOCATIONAPPLICATION_H_

#include <Application/Application.h>
#include <DataSet/DataType/DataBase.h>
#include <Transform/LinearMath/Transform.h>
#include <DataSet/DataType/LaserScan.h>
#include <DataSet/DataType/OccupancyGrid.h>
#include <Service/ServiceType/ServiceBase.h>
#include <DataSet/DataType/PoseWithCovarianceStamped.h>
#include <Time/Time.h>
#include <Time/Duration.h>
#include <Service/Service.h>
#include <Service/ServiceType/ServiceMap.h>
#include <Service/ServiceType/ServiceTransform.h>
#include <DataSet/DataType/Odometry.h>
#include <Service/Server.h>
#include <DataSet/Subscriber.h>
#include <Service/Client.h>
#include <Transform/DataTypes.h>

#include "Map/map.h"
#include "ParticleFilter/pf.h"
#include "Sensors/Odom.h"
#include "Sensors/Laser.h"

namespace NS_Location
{

  class LocationApplication : public Application
  {
  public:
    LocationApplication();
    virtual ~LocationApplication();

  private:
    void loadParameters();

    void laserDataCallback(NS_DataType::LaserScan& laser);

    bool getOdomPose(NS_Transform::Stamped<NS_Transform::Pose>& pose, double& x, double& y, double& yaw);

    map_t* convertMap(NS_DataType::OccupancyGrid& map_msg);

    void mapTransformService(NS_ServiceType::ServiceTransform& transform);

    bool getInitPose(NS_Transform::Stamped< NS_Transform::Pose >& global_pose);

    void freeMapDependentMemory();

    bool processMap();

    static pf_vector_t uniformPoseGenerator(void* arg);

  private:
    //parameters
    int max_beams_, min_particles_, max_particles_;
    double alpha1_, alpha2_, alpha3_, alpha4_, alpha5_;
    double alpha_slow_, alpha_fast_;
    double z_hit_, z_short_, z_max_, z_rand_, sigma_hit_, lambda_short_;
    bool do_beamskip_;
    double beam_skip_distance_, beam_skip_threshold_, beam_skip_error_threshold_;
    double laser_likelihood_max_dist_;
    odom_model_t odom_model_type_;
    laser_model_t laser_model_type_;
    double pf_err_, pf_z_;

    double laser_min_range_;
    double laser_max_range_;

    double d_thresh_, a_thresh_;

    int resample_interval_;

  private:
    AMCLOdom* amcl_odom;
    AMCLLaser* amcl_laser;

    map_t* cur_map;

    pf_t* pf;
    bool pf_initted;
    pf_vector_t pf_odom_pose;

    int resample_count;

    int scan_count;

    boost::mutex map_to_odom_lock;
    boost::mutex map_lock;

    NS_Service::Server< NS_ServiceType::ServiceTransform >* map_tf_srv;

    NS_Transform::Stamped<NS_Transform::Pose> latest_odom_pose;

    NS_DataType::PoseWithCovarianceStamped last_published_pose;

    NS_Transform::Transform map_odom_tf;

    NS_DataSet::Subscriber< NS_DataType::LaserScan >* laser_sub;
    NS_Service::Client< NS_ServiceType::ServiceTransform >* odom_tf_cli;
    NS_Service::Client< NS_ServiceType::ServiceTransform >* map_tf_cli;
    NS_Service::Client< NS_ServiceType::ServiceMap >* map_cli;

    static std::vector<std::pair<int,int> > free_space_indices;

  public:
    virtual void run();
    virtual void quit();
  };

} /* namespace NS_Location */

#endif /* LOCATIONAPPLICATION_H_ */
