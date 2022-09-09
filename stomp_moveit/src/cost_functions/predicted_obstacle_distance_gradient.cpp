/**
 * @file obstacle_distance_gradient.cpp
 * @brief This defines a Robot Model for the Stomp Planner.
 *
 * @author Jorge Nicho
 * @date Jul 22, 2016
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2016, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stomp_moveit/cost_functions/predicted_obstacle_distance_gradient.h>
#include <ros/console.h>
#include <pluginlib/class_list_macros.h>
#include <moveit/robot_state/conversions.h>

PLUGINLIB_EXPORT_CLASS(stomp_moveit::cost_functions::PredictedObstacleDistanceGradient,stomp_moveit::cost_functions::StompCostFunction)
static const double LONGEST_VALID_JOINT_MOVE = 0.01;

namespace stomp_moveit
{
namespace cost_functions
{

PredictedObstacleDistanceGradient::PredictedObstacleDistanceGradient() :
    name_("PredictedObstacleDistanceGradient"),
    robot_state_()
{

}

PredictedObstacleDistanceGradient::~PredictedObstacleDistanceGradient()
{

}

bool PredictedObstacleDistanceGradient::initialize(moveit::core::RobotModelConstPtr robot_model_ptr,
                                          const std::string& group_name, XmlRpc::XmlRpcValue& config)
{
  robot_model_ptr_ = robot_model_ptr;
  group_name_ = group_name;
  collision_request_.distance = true;
  collision_request_.group_name = group_name;
  collision_request_.cost = false;
  collision_request_.max_contacts = 1;
  collision_request_.max_contacts_per_pair = 1;
  collision_request_.contacts = false;
  collision_request_.verbose = false;
  nh_ = new ros::NodeHandle("~");
  sub_human = nh_->subscribe<std_msgs::Float32MultiArray>("/human_sequence",1,&PredictedObstacleDistanceGradient::humanCallback,this);
  
  XmlRpc::XmlRpcValue stomp_config;
  if(!nh_->getParam("/move_group/planning_pipelines/stomp/stomp", stomp_config))
  {
    ROS_ERROR("The 'stomp' configuration parameter was not found");
  }    
  std::cout<<"here"<<std::endl;
  std::vector<std::string> dist_links;

  ROS_INFO_STREAM(static_cast<std::string>(stomp_config.toXml().c_str()).c_str());
  for(XmlRpc::XmlRpcValue::iterator v = stomp_config.begin(); v != stomp_config.end(); v++)
  {
    std::cout<<"here2\n";
    if (static_cast<std::string>(v->second["group_name"])==group_name) {
      std::cout<<"here3\n";
      max_separation = static_cast<double>(v->second["task"]["cost_functions"][0]["max_separation"]);
      min_separation = static_cast<double>(v->second["task"]["cost_functions"][0]["min_separation"]);
      std::cout<<"here4\n";
      for (int i=0; i<v->second["task"]["cost_functions"][0]["distance_links"].size();i++) {
        dist_links.push_back(v->second["task"]["cost_functions"][0]["distance_links"][i]);
        ROS_INFO_STREAM("link from distance_links:"<<dist_links.back());
      }
    }
    
  }
  if (!nh_->getParam("/human_link_lengths", human_link_lengths)) {
      ROS_ERROR("couldn't read human dimensions");
      throw std::invalid_argument("/human_link_lengths is not defined");
  }
  if (!nh_->getParam("/human_link_radii", human_link_radii)) {
      ROS_ERROR("couldn't read human radii");
      throw std::invalid_argument("/human_link_radii is not defined");
  } 

  std::vector<const robot_state::LinkModel*> links = robot_model_ptr->getLinkModels();
  ROS_INFO_STREAM(links.size()<<"links");
  for (int i=0;i<int(links.size());i++) {
      std::string link_name = links[i]->getName();
      ROS_INFO_STREAM(link_name);
      if (std::find(dist_links.begin(),dist_links.end(),link_name)!=dist_links.end()) {
          ROS_INFO_STREAM("link name "<<link_name);
          // if (link_name.substr(link_name.length()-4) == "link") {
          std::vector<shapes::ShapeConstPtr> link_shape = links[i]->getShapes();
          if (link_shape.size()>0){
              const Eigen::Vector3f& extents_tmp = links[i]->getShapeExtentsAtOrigin().cast<float>();
              Eigen::Vector3f extents = extents_tmp;
              const Eigen::Vector3f& offset_tmp = links[i]->getCenteredBoundingBoxOffset().cast<float>();
              Eigen::Vector3f offset = offset_tmp;

              ROS_INFO_STREAM("link name "<<link_name<<", extents:"<<extents.transpose()<<", offset:"<<offset.transpose());
              link_bb_offsets.push_back(offset);
              // Eigen::Isometry3d transform;
              // transform.setIdentity();
              // transform.translate(offset);
              // moveit::core::AABB aabb;
              // aabb.extendWithTransformedBox(transform,extents);
              // link_raw_pts.push_back(boxPts(extents));
              link_boxes.push_back(fcl::Boxf(extents[0],extents[1],extents[2]));
              robot_links.push_back(links[i]);
          }
      }
  }
    
  ROS_INFO("get joint model group");

  const moveit::core::JointModelGroup* jmg=robot_model_ptr->getJointModelGroup(group_name);
  if (jmg==NULL)
    ROS_ERROR("unable to find JointModelGroup for group %s",group_name.c_str());


  ROS_INFO("get joint names of JointModelGroup=%s",jmg->getName().c_str());

  joint_names_=jmg->getActiveJointModelNames();
  dof=joint_names_.size();
  ROS_INFO("number of joints  = %u",dof);

  max_velocity_.resize(dof);

  ROS_INFO("read bounds");
  for (unsigned int idx=0;idx<dof;idx++)
  {
    ROS_INFO("joint %s",joint_names_.at(idx).c_str());
    const robot_model::VariableBounds& bounds = robot_model_ptr_->getVariableBounds(joint_names_.at(idx));
    if (bounds.position_bounded_)
    {
      max_velocity_(idx)=bounds.max_velocity_;
    }
  }

  inv_max_speed_=max_velocity_.cwiseInverse();


  return configure(config);
}

bool PredictedObstacleDistanceGradient::configure(const XmlRpc::XmlRpcValue& config)
{

  try
  {
    // check parameter presence
    auto members = {"cost_weight" ,"max_distance"};
    for(auto& m : members)
    {
      if(!config.hasMember(m))
      {
        ROS_ERROR("%s failed to find the '%s' parameter",getName().c_str(),m);
        return false;
      }
    }

    XmlRpc::XmlRpcValue c = config;
    max_distance_ = static_cast<double>(c["max_distance"]);
    cost_weight_ = static_cast<double>(c["cost_weight"]);
    longest_valid_joint_move_ = c.hasMember("longest_valid_joint_move") ? static_cast<double>(c["longest_valid_joint_move"]):LONGEST_VALID_JOINT_MOVE;

    if(!c.hasMember("longest_valid_joint_move"))
    {
      ROS_WARN("%s using default value for 'longest_valid_joint_move' of %f",getName().c_str(),longest_valid_joint_move_);
    }
  }
  catch(XmlRpc::XmlRpcException& e)
  {
    ROS_ERROR("%s failed to parse configuration parameters",name_.c_str());
    return false;
  }

  return true;
}

bool PredictedObstacleDistanceGradient::setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                    const moveit_msgs::MotionPlanRequest &req,
                                                    const stomp::StompConfiguration &config,
                                                    moveit_msgs::MoveItErrorCodes& error_code)
{
  using namespace moveit::core;

  planning_scene_ = planning_scene;
  plan_request_ = req;
  error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

  // storing robot state
  robot_state_.reset(new RobotState(robot_model_ptr_));

  if(!robotStateMsgToRobotState(req.start_state,*robot_state_,true))
  {
    ROS_ERROR("%s Failed to get current robot state from request",getName().c_str());
    return false;
  }

  // copying into intermediate robot states
  for(auto& rs : intermediate_coll_states_)
  {
    rs.reset(new RobotState(*robot_state_));
  }

  return true;
}

bool PredictedObstacleDistanceGradient::computeCosts(const Eigen::MatrixXd& parameters, std::size_t start_timestep,
                                            std::size_t num_timesteps, int iteration_number, int rollout_number,
                                            Eigen::VectorXd& costs, bool& validity)
{

  if(!robot_state_)
  {
    ROS_ERROR("%s Robot State has not been updated",getName().c_str());
    return false;
  }



  // allocating
  costs = Eigen::VectorXd::Zero(num_timesteps);
  const moveit::core::JointModelGroup* joint_group = robot_model_ptr_->getJointModelGroup(group_name_);

  if(parameters.cols()<start_timestep + num_timesteps)
  {
    ROS_ERROR_STREAM("Size in the 'parameters' matrix is less than required");
    return false;
  }

  // request the distance at each state
  double dist;
  bool skip_next_check = false;
  validity = true;
  double path_time = 0.0;
  for (auto t=start_timestep; t<start_timestep + num_timesteps; ++t)
  {

    if(!skip_next_check)
    {
      collision_result_.clear();
      robot_state_->setJointGroupPositions(joint_group,parameters.col(t));
      robot_state_->update();
      collision_result_.distance = max_distance_;

      planning_scene_->checkSelfCollision(collision_request_,collision_result_,*robot_state_,planning_scene_->getAllowedCollisionMatrix());
      dist = collision_result_.collision ? -1.0 :collision_result_.distance ;

      if(dist >= max_distance_)
      {
        costs(t) = 0; // away from obstacle
      }
      else if(dist < 0)
      {
        costs(t) = 1.0; // in collision
        validity = false;
      }
      else
      {
        costs(t) = (max_distance_ - dist)/max_distance_;
      }
    }

    skip_next_check = false;

    // check intermediate poses to the next position (skip the last one)
    if(t  < start_timestep + num_timesteps - 1)
    {
      ROS_INFO_STREAM(parameters.col(t).transpose());
      if(!checkIntermediateCollisions(parameters.col(t),parameters.col(t+1),longest_valid_joint_move_)) // is collision
      {
        costs(t) = 1.0;
        costs(t+1) = 1.0;
        validity = false;
        skip_next_check = true;
      }
      else
      {
        double pred_cost = checkIntermediatePredictedCollisions(parameters.col(t),parameters.col(t+1),path_time,longest_valid_joint_move_);
        costs(t) += pred_cost;
        costs(t+1) += pred_cost;
        skip_next_check = false;
      }
      path_time += (inv_max_speed_.cwiseProduct(parameters.col(t)-parameters.col(t+1))).cwiseAbs().maxCoeff();
    }

  }

  return true;
}

bool PredictedObstacleDistanceGradient::checkIntermediateCollisions(const Eigen::VectorXd& start,
                                                           const Eigen::VectorXd& end,double longest_valid_joint_move)
{
  Eigen::VectorXd diff = end - start;
  int num_intermediate = std::ceil(((diff.cwiseAbs())/longest_valid_joint_move).maxCoeff()) - 1;
  if(num_intermediate < 1.0)
  {
    // no interpolation needed
    return true;
  }

  // grabbing states
  auto& start_state = intermediate_coll_states_[0];
  auto& mid_state = intermediate_coll_states_[1];
  auto& end_state = intermediate_coll_states_[2];

  if(!start_state || !mid_state || !end_state)
  {
    ROS_ERROR("%s intermediate states not initialized",getName().c_str());
    return false;
  }

  // setting up collision
  auto req = collision_request_;
  req.distance = false;
  collision_detection::CollisionResult res;
  const moveit::core::JointModelGroup* joint_group = robot_model_ptr_->getJointModelGroup(group_name_);
  start_state->setJointGroupPositions(joint_group,start);
  end_state->setJointGroupPositions(joint_group,end);

  // checking intermediate states
  double dt = 1.0/static_cast<double>(num_intermediate);
  double interval = 0.0;
  for(std::size_t i = 1; i < num_intermediate;i++)
  {
    interval = i*dt;
    start_state->interpolate(*end_state,interval,*mid_state) ;
    if(planning_scene_->isStateColliding(*mid_state))
    {
      return false;
    }
  }

  return true;
}

double PredictedObstacleDistanceGradient::checkIntermediatePredictedCollisions(const Eigen::VectorXd& start,
                                                           const Eigen::VectorXd& end, const double& start_time, double longest_valid_joint_move)
{
  Eigen::VectorXd diff = end - start;
  double diff_time = (inv_max_speed_.cwiseProduct(diff)).cwiseAbs().maxCoeff();
  int num_intermediate = std::ceil(((diff.cwiseAbs())/longest_valid_joint_move).maxCoeff()) - 1;

  // grabbing states
  auto& start_state = intermediate_coll_states_[0];
  auto& mid_state = intermediate_coll_states_[1];
  auto& end_state = intermediate_coll_states_[2];

  if(!start_state || !mid_state || !end_state)
  {
    ROS_ERROR("%s intermediate states not initialized",getName().c_str());
    return false;
  }

  // setting up collision
  const moveit::core::JointModelGroup* joint_group = robot_model_ptr_->getJointModelGroup(group_name_);
  start_state->setJointGroupPositions(joint_group,start);
  end_state->setJointGroupPositions(joint_group,end);

  double cost = 0.0;
  fcl::DistanceRequest<float> req(true);
  fcl::DistanceResult<float> res;
  double human_time;
  std::vector<fcl::Cylinderf> human_cylinders;
  std::vector<fcl::Transform3f> human_poses;
  if(num_intermediate < 1.0)
  {    
    double robot_time = start_time;
    int human_idx = int(robot_time/human_dt);
    ROS_INFO_STREAM(human_idx);
    if (human_idx<human_seq.size()) {
      
      std::tie(human_time, human_cylinders, human_poses) = human_seq.at(human_idx);
      ROS_INFO_STREAM(human_cylinders.size());
      ROS_INFO_STREAM(link_boxes.size());
      for (int j=0;j<link_boxes.size();j++) {
        Eigen::Isometry3f offset_transform;
        offset_transform.setIdentity();
        offset_transform.translate(link_bb_offsets[j]);
        fcl::Transform3f link_transform = fcl::Transform3f(start_state->getGlobalLinkTransform(robot_links[j]).cast<float>()*offset_transform);
        for (int k=0;k<human_cylinders.size();k++) {
          fcl::distance(&link_boxes[j],link_transform,&human_cylinders[k],human_poses[k],req,res);
          ROS_INFO_STREAM("min dist"<<res.min_distance);
          if (res.min_distance<max_separation) cost = std::max(cost, 10.0/std::max((res.min_distance-min_separation)/(max_separation-min_separation),1.0E-6));
        }
      }
    }
    ROS_INFO_STREAM("no int");
    // no interpolation needed
    return cost;
  }
  // checking intermediate states
  double dt = 1.0/static_cast<double>(num_intermediate);
  double interval = 0.0;

  for(std::size_t i = 1; i < num_intermediate;i++)
  {
    interval = i*dt;
    start_state->interpolate(*end_state,interval,*mid_state) ;
    double robot_time = start_time + interval*diff_time;
    int human_idx = int(robot_time/human_dt);
    if (human_idx<human_seq.size()) {
      std::tie(human_time, human_cylinders, human_poses) = human_seq.at(human_idx);
      for (int j=0;j<link_boxes.size();j++) {
        Eigen::Isometry3f offset_transform;
        offset_transform.setIdentity();
        offset_transform.translate(link_bb_offsets[j]);
        fcl::Transform3f link_transform = fcl::Transform3f(mid_state->getGlobalLinkTransform(robot_links[j]).cast<float>()*offset_transform);
        for (int k=0;k<human_cylinders.size();k++) {
          fcl::distance(&link_boxes[j],link_transform,&human_cylinders[k],human_poses[k],req,res);
          ROS_INFO_STREAM("min dist"<<res.min_distance);
          if (res.min_distance<max_separation) cost = std::max(cost, 10.0/std::max((res.min_distance-min_separation)/(max_separation-min_separation),1.0E-6));
        }
      }
    }
    
  }
  ROS_INFO_STREAM(cost);
  return cost;
}

void PredictedObstacleDistanceGradient::done(bool success,int total_iterations,double final_cost,const Eigen::MatrixXd& parameters)
{
  robot_state_.reset();
}

void PredictedObstacleDistanceGradient::humanCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    ROS_INFO_STREAM("reading human----------------------------");

    //dim[0] is the number of quat elements per pose
    //dim[1] is the number of time steps predicted into the future
    //get the dimensions from the message
    // float dstride0 = msg->prediction.layout.dim[0].stride;
    // float dstride1 = msg->prediction.layout.dim[1].stride;
    human_seq.clear();
    if (!msg->data.empty()) {
        int h = (int)msg->layout.dim[0].size;
        int w = (int)msg->layout.dim[1].size;
        //cast the msg data to a vector of floats
        // std::lock_guard<std::mutex> lock(msg_mtx);
        for (int i=0;i<w;i++) {
            // ROS_INFO_STREAM("i:"<<i<<", start/end:"<<start<<"/"<<end);
            std::vector<float> tmp_vec;
            for (int j=0;j<h;j++) {
              tmp_vec.push_back(msg->data[i*h+j]);
            }

            forward_kinematics(tmp_vec);
        }
    }
    ROS_INFO_STREAM(msg->data.size());
    ROS_INFO_STREAM("updated the human ........................................"<<human_seq.size());
}

void PredictedObstacleDistanceGradient::forward_kinematics(std::vector<float> pose_elements) {
    // ROS_INFO_STREAM("forward kinematics "<<pose_elements.size());
    // if (prediction.data[31]>0) {
    //     ROS_INFO_STREAM("error");
    //     ROS_INFO_STREAM(prediction);
    // }
    Eigen::Vector3f pelvis_loc = {pose_elements[1],pose_elements[2],pose_elements[3]};

    Eigen::Quaternionf z_axis_quat(0,0,0,1);
    std::vector<Eigen::Quaternionf> quats;
    Eigen::Quaternionf q;

    Eigen::Quaternionf quat_to_world = Eigen::Quaternionf::Identity();
    for (int i=0;i<7;i++){
        q = quat_to_world*Eigen::Quaternionf(pose_elements[i*4+4],pose_elements[i*4+5],pose_elements[i*4+6],pose_elements[i*4+7]);
        quats.push_back(q);
        // ROS_INFO_STREAM("quat "<<q.w()<<" "<<q.vec().transpose());
    }
    std::vector<Eigen::Vector3f> joint_locations;
    std::vector<Eigen::Vector3f> centroids;
    std::vector<fcl::Cylinderf> human_cylinders;
    std::vector<fcl::Transform3f> human_poses;
    joint_locations.push_back(pelvis_loc);
    Eigen::Quaternionf z_spine = quats[0]*z_axis_quat*quats[0].inverse();
    Eigen::Vector3f spine_top = pelvis_loc+human_link_lengths[0]*z_spine.vec();
    joint_locations.push_back(spine_top);
    centroids.push_back(0.5*(spine_top+pelvis_loc));
    // ROS_INFO_STREAM("spine top "<<spine_top.transpose());
    Eigen::Quaternionf z_neck = quats[1]*z_axis_quat*quats[1].inverse();
    Eigen::Vector3f head = spine_top+human_link_lengths[1]*z_neck.vec();
    joint_locations.push_back(head);
    centroids.push_back(0.5*(spine_top+head));
    centroids.push_back(0.5*(spine_top));
    // ROS_INFO_STREAM("head top "<<head.transpose());
    Eigen::Quaternionf z_shoulders = quats[2]*z_axis_quat*quats[2].inverse();
    Eigen::Vector3f l_shoulder = spine_top-0.5*human_link_lengths[2]*z_shoulders.vec();
    joint_locations.push_back(l_shoulder);
    // ROS_INFO_STREAM("l_shoulder "<<l_shoulder.transpose());
    Eigen::Quaternionf z_e1 = quats[3]*z_axis_quat*quats[3].inverse();
    Eigen::Vector3f e1 = l_shoulder+human_link_lengths[3]*z_e1.vec();
    joint_locations.push_back(e1 );
    centroids.push_back(0.5*(e1+l_shoulder));
    Eigen::Quaternionf z_w1 = quats[4]*z_axis_quat*quats[4].inverse();
    Eigen::Vector3f w1 = e1+(human_link_lengths[4]+0.1)*z_w1.vec();
    joint_locations.push_back(w1);
    centroids.push_back(0.5*(e1+w1));
    Eigen::Vector3f r_shoulder = spine_top+0.5*human_link_lengths[2]*z_shoulders.vec();
    joint_locations.push_back(r_shoulder);
    // ROS_INFO_STREAM("r_shoulder "<<r_shoulder.transpose());
    Eigen::Quaternionf z_e2 = quats[5]*z_axis_quat*quats[5].inverse();
    Eigen::Vector3f e2 = r_shoulder+human_link_lengths[5]*z_e2.vec();
    joint_locations.push_back(e2);
    centroids.push_back(0.5*(e2+r_shoulder));
    Eigen::Quaternionf z_w2 = quats[6]*z_axis_quat*quats[6].inverse();
    Eigen::Vector3f w2 = e2+(human_link_lengths[6]+0.1)*z_w2.vec();
    joint_locations.push_back(w2);
    centroids.push_back(0.5*(e2+w2));
    Eigen::Vector3f spine_mid = 0.5*(spine_top+pelvis_loc);

    for (int i = 0;i<human_link_radii.size();i++) {
      if (i==2) continue;
      human_cylinders.push_back(fcl::Cylinderf(human_link_radii[i], human_link_lengths[i]));
      fcl::Transform3f obj_pose = fcl::Transform3f::Identity();
      obj_pose.linear() = quats[i].toRotationMatrix();
      obj_pose.translation() = centroids[i];
      human_poses.push_back(obj_pose);
    }

    human_seq.push_back(std::tuple<float,std::vector<fcl::Cylinderf>,std::vector<fcl::Transform3f>>(pose_elements[0],human_cylinders,human_poses));
    
}

} /* namespace cost_functions */
} /* namespace stomp_moveit */
