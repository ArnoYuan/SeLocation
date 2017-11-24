/*
 * LocationApplication.cpp
 *
 *  Created on: Nov 7, 2017
 *      Author: root
 */

#include "LocationApplication.h"
#include <Parameter/Parameter.h>
#include <DataSet/DataType/Odometry.h>
#include <Service/ServiceType/ServiceOdometry.h>
#include <Console/Console.h>

namespace NS_Location
{

  typedef struct
  {
    // Total weight (weights sum to 1)
    double weight;

    // Mean of pose esimate
    pf_vector_t pf_pose_mean;

    // Covariance of pose estimate
    pf_matrix_t pf_pose_cov;

  }amcl_hyp_t;

  static double normalize(double z)
  {
    return atan2(sin(z), cos(z));
  }

  static double angle_diff(double a, double b)
  {
    double d1, d2;
    a = normalize(a);
    b = normalize(b);
    d1 = a - b;
    d2 = 2 * M_PI - fabs(d1);
    if(d1 > 0)
      d2 *= -1.0;
    if(fabs(d1) < fabs(d2))
      return(d1);
    else
      return(d2);
  }

  std::vector<std::pair<int,int> > LocationApplication::free_space_indices;

  LocationApplication::LocationApplication() :
      cur_map(NULL),
      pf(NULL),
      resample_count(0),
      amcl_odom(NULL),
      amcl_laser(NULL),
      pf_initted(false)
  {
    map_tf_srv = new NS_Service::Server< NS_ServiceType::ServiceTransform >(
        "ODOM_MAP_TF",
        boost::bind(&LocationApplication::mapTransformService, this, _1));

    laser_sub = new NS_DataSet::Subscriber< NS_DataType::LaserScan >(
        "LASER_SCAN",
        boost::bind(&LocationApplication::laserDataCallback, this, _1));

    odom_tf_cli = new NS_Service::Client< NS_ServiceType::ServiceTransform >(
        "BASE_ODOM_TF");

    map_tf_cli = new NS_Service::Client< NS_ServiceType::ServiceTransform >(
        "BASE_ODOM_TF");

    map_cli = new NS_Service::Client< NS_ServiceType::ServiceMap >(
        "ODOM_MAP_TF");
  }

  LocationApplication::~LocationApplication()
  {
    delete map_tf_srv;
    delete laser_sub;
    delete odom_tf_cli;
    delete map_tf_cli;
    delete map_cli;
  }

  void LocationApplication::loadParameters()
  {
    NS_NaviCommon::Parameter parameter;
    parameter.loadConfigurationFile("location.xml");

    laser_min_range_ = parameter.getParameter("laser_min_range", -1.0f);
    laser_max_range_ = parameter.getParameter("laser_max_range", -1.0f);
    max_beams_ = parameter.getParameter("laser_max_beams",30);
    min_particles_ = parameter.getParameter("min_particles", 100);
    max_particles_ = parameter.getParameter("max_particles", 5000);
    pf_err_ = parameter.getParameter("kld_err",0.01f);
    pf_z_ = parameter.getParameter("kld_z",0.99f);
    alpha1_ = parameter.getParameter("odom_alpha1", 0.2f);
    alpha2_ = parameter.getParameter("odom_alpha2", 0.2f);
    alpha3_ = parameter.getParameter("odom_alpha3", 0.2f);
    alpha4_ = parameter.getParameter("odom_alpha4", 0.2f);
    alpha5_ = parameter.getParameter("odom_alpha5", 0.2f);

    do_beamskip_ = parameter.getParameter("do_beamskip", false);
    beam_skip_distance_ = parameter.getParameter("beam_skip_distance", 0.5f);
    beam_skip_threshold_ = parameter.getParameter("beam_skip_threshold", 0.3f);
    beam_skip_error_threshold_ = parameter.getParameter("beam_skip_error_threshold", 0.9f);

    z_hit_ = parameter.getParameter("laser_z_hit", 0.95f);
    z_short_ = parameter.getParameter("laser_z_short", 0.1f);
    z_max_ = parameter.getParameter("laser_z_max", 0.05f);
    z_rand_ = parameter.getParameter("laser_z_rand", 0.05f);
    sigma_hit_ = parameter.getParameter("laser_sigma_hit", 0.2f);
    lambda_short_ = parameter.getParameter("laser_lambda_short", 0.1f);
    laser_likelihood_max_dist_ = parameter.getParameter("laser_likelihood_max_dist", 2.0f);

    std::string tmp_model_type = parameter.getParameter("laser_model_type", std::string("likelihood_field"));
    if(tmp_model_type == "beam")
      laser_model_type_ = LASER_MODEL_BEAM;
    else if(tmp_model_type == "likelihood_field")
      laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD;
    else if(tmp_model_type == "likelihood_field_prob"){
      laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD_PROB;
    }
    else
    {
      laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD;
    }

    tmp_model_type = parameter.getParameter("odom_model_type", std::string("diff"));
    if(tmp_model_type == "diff")
      odom_model_type_ = ODOM_MODEL_DIFF;
    else if(tmp_model_type == "omni")
      odom_model_type_ = ODOM_MODEL_OMNI;
    else if(tmp_model_type == "diff-corrected")
      odom_model_type_ = ODOM_MODEL_DIFF_CORRECTED;
    else if(tmp_model_type == "omni-corrected")
      odom_model_type_ = ODOM_MODEL_OMNI_CORRECTED;
    else
    {
      odom_model_type_ = ODOM_MODEL_DIFF;
    }

    d_thresh_ = parameter.getParameter("update_min_d", 0.2f);
    a_thresh_ = parameter.getParameter("update_min_a", 0.5f);
    resample_interval_ = parameter.getParameter("resample_interval", 2);

    alpha_slow_ = parameter.getParameter("recovery_alpha_slow", 0.001f);
    alpha_fast_ = parameter.getParameter("recovery_alpha_fast", 0.1f);
  }

  bool LocationApplication::getOdomPose(NS_Transform::Stamped<NS_Transform::Pose>& pose, double& x, double& y, double& yaw)
  {
    NS_ServiceType::ServiceTransform odom_tf_rep;
    NS_Transform::Transform odom_tf;
    NS_Service::Client< NS_ServiceType::ServiceTransform > odom_tf_cli("BASE_ODOM_TF");

    if(!odom_tf_cli.call(odom_tf_rep))
    {
      return false;
    }

    if(!odom_tf_rep.result)
    {
      return false;
    }

    NS_Transform::transformMsgToTF(odom_tf_rep.transform, odom_tf);

    x = odom_tf.getOrigin().x();
    y = odom_tf.getOrigin().y();
    double pitch,roll;
    odom_tf.getBasis().getEulerYPR(yaw, pitch, roll);

    pose.setData(odom_tf);

    return true;
  }

  map_t* LocationApplication::convertMap(NS_DataType::OccupancyGrid& map_msg)
  {
    map_t* map = map_alloc();

    map->size_x = map_msg.info.width;
    map->size_y = map_msg.info.height;
    map->scale = map_msg.info.resolution;
    map->origin_x = map_msg.info.origin.position.x + (map->size_x / 2) * map->scale;
    map->origin_y = map_msg.info.origin.position.y + (map->size_y / 2) * map->scale;
    // Convert to player format
    map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map->size_x*map->size_y);

    for(int i = 0; i < map->size_x * map->size_y; i++)
    {
      if(map_msg.data[i] == 0)
        map->cells[i].occ_state = -1;
      else if(map_msg.data[i] == 100)
        map->cells[i].occ_state = +1;
      else
        map->cells[i].occ_state = 0;
    }

    return map;
  }

  void LocationApplication::laserDataCallback(NS_DataType::LaserScan& laser)
  {
    if(!running)
      return;

    if(cur_map == NULL)
      return;

    if(scan_count == 0)
    {
      amcl_laser = new AMCLLaser(max_beams_, cur_map);
      NS_Transform::Stamped<NS_Transform::Pose> laser_pose;

      //transform laser->base_link
      laser_pose.setIdentity();

      pf_vector_t laser_pose_v;
      laser_pose_v.v[0] = laser_pose.getOrigin().x();
      laser_pose_v.v[1] = laser_pose.getOrigin().y();
      // laser mounting angle gets computed later -> set to 0 here!
      laser_pose_v.v[2] = 0;
      amcl_laser->SetLaserPose(laser_pose_v);
    }

    // Where was the robot when this scan was taken?
    pf_vector_t pose;
    if(!getOdomPose(latest_odom_pose, pose.v[0], pose.v[1], pose.v[2]))
    {
      console.warning("Couldn't determine robot's pose associated with laser scan");
      return;
    }

    pf_vector_t delta = pf_vector_zero();
    bool update = false;
    if(pf_initted)
    {
      // Compute change in pose
      //delta = pf_vector_coord_sub(pose, pf_odom_pose_);
      delta.v[0] = pose.v[0] - pf_odom_pose.v[0];
      delta.v[1] = pose.v[1] - pf_odom_pose.v[1];
      delta.v[2] = angle_diff(pose.v[2], pf_odom_pose.v[2]);

      // See if we should update the filter
      bool update = fabs(delta.v[0]) > d_thresh_ ||
                    fabs(delta.v[1]) > d_thresh_ ||
                    fabs(delta.v[2]) > a_thresh_;
    }

    if(!pf_initted)
    {
      // Pose at last filter update
      pf_odom_pose = pose;

      // Filter is now initialized
      pf_initted = true;

      resample_count = 0;
    }
    // If the robot has moved, update the filter
    else if(pf_initted && update)
    {
      //printf("pose\n");
      //pf_vector_fprintf(pose, stdout, "%.3f");

      AMCLOdomData odata;
      odata.pose = pose;
      // HACK
      // Modify the delta in the action data so the filter gets
      // updated correctly
      odata.delta = delta;

      // Use the action data to update the filter
      amcl_odom->UpdateAction(pf, (AMCLSensorData*)&odata);

      // Pose at last filter update
      //this->pf_odom_pose = pose;
    }

    bool resampled = false;
    // If the robot has moved, update the filter
    if(update)
    {
      AMCLLaserData ldata;
      ldata.sensor = amcl_laser;
      ldata.range_count = laser.ranges.size();

      // To account for lasers that are mounted upside-down, we determine the
      // min, max, and increment angles of the laser in the base frame.
      //
      // Construct min and max angles of laser, in the base_link frame.
      NS_Transform::Quaternion q;
      q.setRPY(0.0, 0.0, laser.angle_min);
      NS_Transform::Stamped<NS_Transform::Quaternion> min_q(q, laser.header.stamp, laser.header.frame_id);
      q.setRPY(0.0, 0.0, laser.angle_min + laser.angle_increment);
      NS_Transform::Stamped<NS_Transform::Quaternion> inc_q(q, laser.header.stamp, laser.header.frame_id);

      //TODO:transform laser->base_link


      double angle_min = NS_Transform::getYaw(min_q);
      double angle_increment = NS_Transform::getYaw(inc_q) - angle_min;

      // wrapping angle to [-pi .. pi]
      angle_increment = fmod(angle_increment + 5*M_PI, 2*M_PI) - M_PI;

      console.debug("Laser angles in base frame: min: %.3f inc: %.3f", angle_min, angle_increment);

      // Apply range min/max thresholds, if the user supplied them
      if(laser_max_range_ > 0.0)
        ldata.range_max = std::min(laser.range_max, (float)laser_max_range_);
      else
        ldata.range_max = laser.range_max;
      double range_min;
      if(laser_min_range_ > 0.0)
        range_min = std::max(laser.range_min, (float)laser_min_range_);
      else
        range_min = laser.range_min;
      // The AMCLLaserData destructor will free this memory
      ldata.ranges = new double[ldata.range_count][2];
      for(int i = 0; i < ldata.range_count; i++)
      {
        // amcl doesn't (yet) have a concept of min range.  So we'll map short
        // readings to max range.
        if(laser.ranges[i] <= range_min)
          ldata.ranges[i][0] = ldata.range_max;
        else
          ldata.ranges[i][0] = laser.ranges[i];
        // Compute bearing
        ldata.ranges[i][1] = angle_min + (i * angle_increment);
      }

      amcl_laser->UpdateSensor(pf, (AMCLSensorData*)&ldata);

      update = false;

      pf_odom_pose = pose;

      // Resample the particles
      if(!(++resample_count % resample_interval_))
      {
        pf_update_resample(pf);
        resampled = true;
      }

      pf_sample_set_t* set = pf->sets + pf->current_set;
      console.debug("pf samples: %d\n", set->sample_count);
    }

    if(resampled)
    {
      // Read out the current hypotheses
      double max_weight = 0.0;
      int max_weight_hyp = -1;
      std::vector<amcl_hyp_t> hyps;
      hyps.resize(pf->sets[pf->current_set].cluster_count);
      for(int hyp_count = 0; hyp_count < pf->sets[pf->current_set].cluster_count; hyp_count++)
      {
        double weight;
        pf_vector_t pose_mean;
        pf_matrix_t pose_cov;
        if (!pf_get_cluster_stats(pf, hyp_count, &weight, &pose_mean, &pose_cov))
        {
          console.warning("Couldn't get stats on cluster %d", hyp_count);
          break;
        }

        hyps[hyp_count].weight = weight;
        hyps[hyp_count].pf_pose_mean = pose_mean;
        hyps[hyp_count].pf_pose_cov = pose_cov;

        if(hyps[hyp_count].weight > max_weight)
        {
          max_weight = hyps[hyp_count].weight;
          max_weight_hyp = hyp_count;
        }
      }

      if(max_weight > 0.0)
      {
        console.debug("Max weight pose: %.3f %.3f %.3f", hyps[max_weight_hyp].pf_pose_mean.v[0], hyps[max_weight_hyp].pf_pose_mean.v[1], hyps[max_weight_hyp].pf_pose_mean.v[2]);

        /*
           puts("");
           pf_matrix_fprintf(hyps[max_weight_hyp].pf_pose_cov, stdout, "%6.3f");
           puts("");
         */

        NS_DataType::PoseWithCovarianceStamped p;
        // Fill in the header
        p.header.stamp = laser.header.stamp;
        // Copy in the pose
        p.pose.position.x = hyps[max_weight_hyp].pf_pose_mean.v[0];
        p.pose.position.y = hyps[max_weight_hyp].pf_pose_mean.v[1];
        NS_Transform::quaternionTFToMsg(NS_Transform::createQuaternionFromYaw(hyps[max_weight_hyp].pf_pose_mean.v[2]), p.pose.orientation);
        // Copy in the covariance, converting from 3-D to 6-D
        pf_sample_set_t* set = pf->sets + pf->current_set;
        for(int i = 0; i < 2; i++)
        {
          for(int j = 0; j < 2; j++)
          {
            // Report the overall filter covariance, rather than the
            // covariance for the highest-weight cluster
            //p.covariance[6*i+j] = hyps[max_weight_hyp].pf_pose_cov.m[i][j];
            p.covariance[6 * i + j] = set->cov.m[i][j];
          }
        }
        // Report the overall filter covariance, rather than the
        // covariance for the highest-weight cluster
        //p.covariance[6*5+5] = hyps[max_weight_hyp].pf_pose_cov.m[2][2];
        p.covariance[6 * 5 + 5] = set->cov.m[2][2];

        /*
           printf("cov:\n");
           for(int i=0; i<6; i++)
           {
           for(int j=0; j<6; j++)
           printf("%6.3f ", p.covariance[6*i+j]);
           puts("");
           }
         */

        last_published_pose = p;

        console.debug("New pose: %6.3f %6.3f %6.3f", hyps[max_weight_hyp].pf_pose_mean.v[0], hyps[max_weight_hyp].pf_pose_mean.v[1], hyps[max_weight_hyp].pf_pose_mean.v[2]);

        // subtracting base to odom from map to base and send map to odom instead
        NS_Transform::Stamped<NS_Transform::Pose> odom_to_map;

        NS_Transform::Transform tmp_tf(NS_Transform::createQuaternionFromYaw(hyps[max_weight_hyp].pf_pose_mean.v[2]),
                                       NS_Transform::Vector3(hyps[max_weight_hyp].pf_pose_mean.v[0], hyps[max_weight_hyp].pf_pose_mean.v[1], 0.0));


        NS_Transform::StampedTransform odom_to_base;

        NS_ServiceType::ServiceTransform base_odom_tf;

        if(odom_tf_cli->call(base_odom_tf))
        {
          NS_Transform::transformMsgToTF(base_odom_tf.transform, odom_to_base);

          boost::mutex::scoped_lock map_to_odom_tf_mutex(map_to_odom_lock);

          map_odom_tf = NS_Transform::Transform(odom_to_map * odom_to_base.inverse());
        }
      }
      else
      {
        console.warning("No pose!");
      }
    }

    scan_count++;

  }

  void LocationApplication::freeMapDependentMemory()
  {
    if(cur_map != NULL)
    {
      map_free(cur_map);
      cur_map = NULL;
    }
    if(pf != NULL) {
      pf_free(pf);
      pf = NULL;
    }
    delete amcl_odom;
    amcl_odom = NULL;
    delete amcl_laser;
    amcl_laser = NULL;
  }

  bool LocationApplication::getInitPose(NS_Transform::Stamped< NS_Transform::Pose >& global_pose)
  {
    NS_ServiceType::ServiceTransform odom_transform;
    NS_ServiceType::ServiceTransform map_transform;

    if(odom_tf_cli->call(odom_transform) == false)
    {
      //printf("Get odometry transform failure!\n");
      return false;
    }

    if(odom_transform.result == false)
    {
      return false;
    }

    if(map_tf_cli->call(map_transform) == false)
    {
      //printf("Get map transform failure!\n");
      return false;
    }

    if(map_transform.result == false)
    {
      return false;
    }

    //TODO: not verify code for transform
    NS_Transform::Transform odom_tf, map_tf;
    NS_Transform::transformMsgToTF(odom_transform.transform, odom_tf);
    NS_Transform::transformMsgToTF(map_transform.transform, map_tf);

    global_pose.setData(odom_tf * map_tf);

    return true;
  }

  bool LocationApplication::processMap()
  {
    NS_ServiceType::ServiceMap map_req;
    if(!map_cli->call(map_req))
    {
      console.warning("Request map failure!");
      return false;
    }
    if(!map_req.result)
    {
      console.warning("Get map failure!");
      return false;
    }

    console.debug("Received a %d X %d map @ %.3f m/pix\n", map_req.map.info.width, map_req.map.info.height, map_req.map.info.resolution);

    freeMapDependentMemory();

    cur_map = convertMap(map_req.map);

    free_space_indices.resize(0);
    for(int i = 0; i < cur_map->size_x; i++)
      for(int j = 0; j < cur_map->size_y; j++)
        if(cur_map->cells[MAP_INDEX(cur_map, i, j)].occ_state == -1)
          free_space_indices.push_back(std::make_pair(i,j));

    // Create the particle filter
    pf = pf_alloc(min_particles_, max_particles_, alpha_slow_, alpha_fast_, (pf_init_model_fn_t)LocationApplication::uniformPoseGenerator, (void*)cur_map);
    pf->pop_err = pf_err_;
    pf->pop_z = pf_z_;

    // Initialize the filter
    NS_Transform::Stamped< NS_Transform::Pose > init_pose;
    if(!getInitPose(init_pose))
    {
      console.warning("Get init pose failure!");
      return false;
    }

    NS_DataType::PoseStamped start;
    NS_Transform::poseStampedTFToMsg(init_pose, start);

    pf_vector_t pf_init_pose_mean = pf_vector_zero();
    pf_init_pose_mean.v[0] = start.pose.position.x;
    pf_init_pose_mean.v[1] = start.pose.position.y;
    pf_init_pose_mean.v[2] = NS_Transform::getYaw(start.pose.orientation);
    pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
    pf_init_pose_cov.m[0][0] = 0.5 * 0.5;
    pf_init_pose_cov.m[1][1] = 0.5 * 0.5;
    pf_init_pose_cov.m[2][2] = (M_PI/12.0) * (M_PI/12.0);
    pf_init(pf, pf_init_pose_mean, pf_init_pose_cov);
    pf_initted = false;

    // Instantiate the sensor objects
    // Odometry
    amcl_odom = new AMCLOdom();
    amcl_odom->SetModel( odom_model_type_, alpha1_, alpha2_, alpha3_, alpha4_, alpha5_ );
    // Laser
    amcl_laser = new AMCLLaser(max_beams_, cur_map);
    if(laser_model_type_ == LASER_MODEL_BEAM)
      amcl_laser->SetModelBeam(z_hit_, z_short_, z_max_, z_rand_,
                           sigma_hit_, lambda_short_, 0.0);
    else if(laser_model_type_ == LASER_MODEL_LIKELIHOOD_FIELD_PROB){
      console.debug("Initializing likelihood field model; this can take some time on large maps...");
      amcl_laser->SetModelLikelihoodFieldProb(z_hit_, z_rand_, sigma_hit_,
                      laser_likelihood_max_dist_,
                      do_beamskip_, beam_skip_distance_,
                      beam_skip_threshold_, beam_skip_error_threshold_);
      console.debug("Done initializing likelihood field model.");
    }
    else
    {
      console.debug("Initializing likelihood field model; this can take some time on large maps...");
      amcl_laser->SetModelLikelihoodField(z_hit_, z_rand_, sigma_hit_,
                                      laser_likelihood_max_dist_);
      console.debug("Done initializing likelihood field model.");
    }


    return true;
  }

  pf_vector_t LocationApplication::uniformPoseGenerator(void* arg)
  {
    map_t* map = (map_t*)arg;

    unsigned int rand_index = drand48() * free_space_indices.size();
    std::pair<int,int> free_point = free_space_indices[rand_index];
    pf_vector_t p;
    p.v[0] = MAP_WXGX(map, free_point.first);
    p.v[1] = MAP_WYGY(map, free_point.second);
    p.v[2] = drand48() * 2 * M_PI - M_PI;

    return p;
  }

  void LocationApplication::mapTransformService(NS_ServiceType::ServiceTransform& transform)
  {
    boost::mutex::scoped_lock map_to_odom_tf_mutex(map_to_odom_lock);
    NS_Transform::transformTFToMsg(map_odom_tf, transform.transform);
  }

  void LocationApplication::run()
  {
    loadParameters();

    running = true;
  }

  void LocationApplication::quit()
  {
    running = false;
  }


} /* namespace NS_Location */
