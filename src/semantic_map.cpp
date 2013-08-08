/**
 * @author Carsten Könemann
 */

#include <thesis/semantic_map.h>

#include <thesis/math.h>
#include <thesis/math3d.h>

double SemanticMap::EvaluationComparator::evaluate
(
  const thesis::ObjectInstance& object,
  cv::Point3f p_c
)
{
  // TODO
  
  // Time
  double t = (ros::Time::now() - object.pose_stamped.header.stamp).toSec();
  
  // Object position
  cv::Point3f p_o;
  p_o.x = object.pose_stamped.pose.position.x;
  p_o.y = object.pose_stamped.pose.position.y;
  p_o.z = object.pose_stamped.pose.position.z;
  // Distance
  double d = dist3f(p_c, p_o);
  
  // Confidence
  // - seen for how many consecutive frames
  // - ratio of matching keypoints on average
  double c = object.confidence;
  
  // Combined (weighted average)
  
  
  // x
  double x = 0.5;
  
  // y
  double y = 0.5 - atan(x*2.5 -5) / M_PI;
  
  //
//  std::cout << "Semantic map: Evaluate: "      << std::endl;
//  std::cout << "  x: "                    << x << std::endl;
//  std::cout << "  y: "                    << y << std::endl;
  
  //
  return y;
}

boost::uuids::uuid SemanticMap::ObjectQueue::getID()
{
  return id;
}

double SemanticMap::ObjectQueue::age()
{
  return (ros::Time::now() - stamp_init).toSec();
}

size_t SemanticMap::ObjectQueue::size()
{
  return deque.size();
}

thesis::ObjectInstance SemanticMap::ObjectQueue::combined()
{
  thesis::ObjectInstance o;
  
  if(!deque.empty())
  {
    o.type_id = deque.front().type_id;
    
    double confidence_sum             = 0.0,
           orientation_confidence_sum = 0.0,
           yaw_sum                    = 0.0,
           pitch_sum                  = 0.0,
           roll_sum                   = 0.0;
    
    for(size_t i = 0; i < deque.size(); i++)
    {
      thesis::ObjectInstance current = deque.at(i);
      //
      confidence_sum += current.confidence;
      //
      o.confidence += current.confidence * current.confidence;
      //
      o.pose_stamped.pose.position.x
        += current.pose_stamped.pose.position.x * current.confidence;
      o.pose_stamped.pose.position.y
        += current.pose_stamped.pose.position.y * current.confidence;
      o.pose_stamped.pose.position.z
        += current.pose_stamped.pose.position.z * current.confidence;
      //
      if(!isnan(current.pose_stamped.pose.orientation))
      {
        // Quaternion message to TF quaternion
        tf::Quaternion quaternion_tf;
        tf::quaternionMsgToTF(
          current.pose_stamped.pose.orientation,
          quaternion_tf
        );
        // Quaternion to roll, pitch & yaw
        tf::Matrix3x3 matTemp(quaternion_tf);
        double roll, pitch, yaw;
        matTemp.getRPY(roll, pitch, yaw);
        //
        orientation_confidence_sum += current.confidence;
        yaw_sum   += yaw   * current.confidence;
        pitch_sum += pitch * current.confidence;
        roll_sum  += roll  * current.confidence;
      }
    }
    
    o.confidence /= confidence_sum;
    
    o.pose_stamped.pose.position.x /= confidence_sum;
    o.pose_stamped.pose.position.y /= confidence_sum;
    o.pose_stamped.pose.position.z /= confidence_sum;
    
    yaw_sum   /= orientation_confidence_sum;
    pitch_sum /= orientation_confidence_sum;
    roll_sum  /= orientation_confidence_sum;
    
    o.pose_stamped.pose.orientation
      = tf::createQuaternionMsgFromRollPitchYaw(roll_sum, pitch_sum, yaw_sum);
  }
  else
  {
    ROS_ERROR("SemanticMap::ObjectQueue::combined(): Called on empty queue.");
    std::cout << std::endl;
    o.type_id = "";
    o.pose_stamped = geometry_msgs::PoseStamped();
  }
  
  return o;
}

void SemanticMap::ObjectQueue::add(thesis::ObjectInstance o)
{
  // enqueue
  deque.push_back(o);
  // dequeue
  if((int) deque.size() > memory_size)
  {
    deque.pop_front();
  }
}

void SemanticMap::ObjectQueue::flag(double age_threshold)
{
  if((ros::Time::now() - stamp_flag).toSec() < age_threshold)
  {
    flags++;
  }
  else
  {
    flags = 1;
    stamp_flag = ros::Time::now();
  }
}

unsigned int SemanticMap::ObjectQueue::flagged()
{
  return flags;
}

bool SemanticMap::exists(std::string id)
{
  try
  {
    map.at(id);
  }
  catch(const std::out_of_range& oor)
  {
    return false;
  }
  return true;
}

void SemanticMap::setCurrentPosition(cv::Point3f p)
{
  currentPosition = p;
}

void SemanticMap::flag
(
  const std::string& type,
  const boost::uuids::uuid& id,
  double age_threshold
)
{
  if(exists(type))
  {
    std::vector<ObjectQueue>::iterator vec_iter = map[type].begin();
    for(; vec_iter != map[type].end(); vec_iter++)
    {
      if(vec_iter->getID() == id)
      {
        vec_iter->flag(age_threshold);
      }
    }
  }
}

void SemanticMap::cleanup(double age_threshold, unsigned int min_confirmations)
{
  int removals = 0;
  // 'Garbage collection'
  std::map<std::string, std::vector<ObjectQueue> >::iterator map_iter = map.begin();
  for(; map_iter != map.end(); map_iter++)
  {
    std::vector<ObjectQueue>::iterator vec_iter = map_iter->second.begin();
    while(vec_iter != map_iter->second.end())
    {
      // Recently flagged for removal
      bool autoremove = vec_iter->flagged() >= min_confirmations,
      // False positives
           false_pos  = vec_iter->age() > age_threshold && vec_iter->size() < min_confirmations;
      // Remove current object
      if(autoremove || false_pos)
      {
        removals++;
        vec_iter = map_iter->second.erase(vec_iter);
      }
      // Don't remove current object
      else
      {
        vec_iter++;
      }
    }
  }
  // Debug output
  if(debug)
  {
    ROS_INFO("SemanticMap::cleanup(%f, %i): Removed %i objects.",
      age_threshold,
      min_confirmations,
      removals
    );
    std::cout << std::endl;
  }
}

void SemanticMap::add(const thesis::ObjectInstance& object, float min_distance)
{
  //
  cv::Point3f p;
  p.x = object.pose_stamped.pose.position.x;
  p.y = object.pose_stamped.pose.position.y;
  p.z = object.pose_stamped.pose.position.z;
  //
  if(debug)
  {
    ROS_INFO("SemanticMap::add(%s, %f): ", object.type_id.c_str(), min_distance);
    ROS_INFO("  At position: (%f, %f, %f).", p.x, p.y, p.z);
  }
  // Check for already stored objects of the same type
  std::vector<std::vector<ObjectQueue>::iterator> existing_objects;
  if(exists(object.type_id))
  {
    std::vector<ObjectQueue>::iterator iter = map[object.type_id].begin();
    for(; iter != map[object.type_id].end(); iter++)
    {
      thesis::ObjectInstance temp = iter->combined();
      cv::Point3f p_;
      p_.x = temp.pose_stamped.pose.position.x;
      p_.y = temp.pose_stamped.pose.position.y;
      p_.z = temp.pose_stamped.pose.position.z;
      float dist = dist3f(p, p_);
      if(debug)
      {
        ROS_INFO("  dist3f(p, p_): %f.", dist);
      }
      if(dist <= min_distance)
      {
        existing_objects.push_back(iter);
      }
    }
  }
  if(debug)
  {
    ROS_INFO("  existing_objects.size(): %lo.", existing_objects.size());
  }
  // If more than one object of this type exist at the same position...
  if(existing_objects.size() > 1)
  {
    // ...warn the user
    if(debug)
    {
      ROS_WARN("  Multiple existing objects of type %s at position (%f, %f, %f).",
        object.type_id.c_str(),
        object.pose_stamped.pose.position.x,
        object.pose_stamped.pose.position.y,
        object.pose_stamped.pose.position.z
      );
    }
    // ...look for the closest
    std::vector<ObjectQueue>::iterator closest;
    float closest_distance = FLT_MAX;
    for(size_t i = 0; i < existing_objects.size(); i++)
    {
      thesis::ObjectInstance temp = existing_objects[i]->combined();
      cv::Point3f p_;
      p_.x = temp.pose_stamped.pose.position.x;
      p_.y = temp.pose_stamped.pose.position.y;
      p_.z = temp.pose_stamped.pose.position.z;
      float current_distance = dist3f(p, p_);
      if(current_distance < closest_distance)
      {
        closest_distance = current_distance;
        closest = existing_objects[i];
      }
    }
    // ...and update its entry
    closest->add(object);
  }
  // If one object of this type already exists at the same position...
  else if(existing_objects.size() == 1)
  {
    if(debug)
    {
      ROS_INFO("  Updating %s at (%f, %f, %f).",
        object.type_id.c_str(),
        object.pose_stamped.pose.position.x,
        object.pose_stamped.pose.position.y,
        object.pose_stamped.pose.position.z
      );
    }
    // ...update the entry
    existing_objects.front()->add(object);
  }
  // If there isn't already an object of this type...
  else
  {
    if(debug)
    {
      ROS_INFO("  Adding %s at (%f, %f, %f).",
        object.type_id.c_str(),
        object.pose_stamped.pose.position.x,
        object.pose_stamped.pose.position.y,
        object.pose_stamped.pose.position.z
      );
    }
    // ...create a new entry
    ObjectQueue entry = ObjectQueue(memory_size);
    entry.add(object);
    map[object.type_id].push_back(entry);
  }
  //
  std::cout << std::endl;
}

void SemanticMap::getAll(std::vector<thesis::ObjectInstance>& out)
{
  //
  std::map<std::string, std::vector<ObjectQueue> >::iterator it = map.begin();
  for(; it != map.end(); it++)
  {
    for(size_t i = 0; i < it->second.size(); i++)
    {
      out.push_back(it->second[i].combined());
    }
  }
  //
  EvaluationComparator evaluationComparator(this);
  std::sort(out.begin(), out.end(), evaluationComparator);
}

void SemanticMap::getByID
(
  std::string id,
  std::vector<thesis::ObjectInstance>& out
)
{
  //
  std::vector<ObjectQueue> objects = map[id];
  for(size_t i = 0; i < objects.size(); i++)
  {
    out.push_back(objects[i].combined());
  }
  //
  EvaluationComparator evaluationComparator(this);
  std::sort(out.begin(), out.end(), evaluationComparator);
}

void SemanticMap::getByIDAtPosition
(
  std::string id,
  cv::Point3f p,
  std::vector<thesis::ObjectInstance>& out,
  float max_distance
)
{
  //
  std::vector<thesis::ObjectInstance> objects;
  getByID(id, objects);
  for(size_t i = 0; i < objects.size(); i++)
  {
    cv::Point3f p_;
    p_.x = objects[i].pose_stamped.pose.position.x;
    p_.y = objects[i].pose_stamped.pose.position.y;
    p_.z = objects[i].pose_stamped.pose.position.z;
    if(dist3f(p, p_) < max_distance)
    {
      out.push_back(objects[i]);
    }
  }
  //
  EvaluationComparator evaluationComparator(this);
  std::sort(out.begin(), out.end(), evaluationComparator);
}

void SemanticMap::getByPosition
(
  cv::Point3f p,
  std::vector<thesis::ObjectInstance>& out,
  float max_distance
)
{
  //
  std::map<std::string, std::vector<ObjectQueue> >::iterator it = map.begin();
  for(; it != map.end(); it++)
  {
    for(size_t i = 0; i < it->second.size(); i++)
    {
      thesis::ObjectInstance current = it->second[i].combined();
      cv::Point3f p_;
      p_.x = current.pose_stamped.pose.position.x;
      p_.y = current.pose_stamped.pose.position.y;
      p_.z = current.pose_stamped.pose.position.z;
      if(dist3f(p, p_) <= max_distance)
      {
        out.push_back(current);
      }
    }
  }
  //
  EvaluationComparator evaluationComparator(this);
  std::sort(out.begin(), out.end(), evaluationComparator);
}
