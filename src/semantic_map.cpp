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

thesis::ObjectInstance SemanticMap::ObjectQueue::combined()
{
  thesis::ObjectInstance o;
  o.type_id = deque.front().type_id;
  
  double confidence_sum = 0.0,
         yaw_sum        = 0.0,
         pitch_sum      = 0.0,
         roll_sum       = 0.0;
  
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
    tf::Quaternion quaternion_tf;
    tf::quaternionMsgToTF(
      current.pose_stamped.pose.orientation,
      quaternion_tf
    );
    tf::Matrix3x3 matTemp(quaternion_tf);
    double roll, pitch, yaw;
    matTemp.getRPY(roll, pitch, yaw);
    yaw_sum   += yaw   * current.confidence;
    pitch_sum += pitch * current.confidence;
    roll_sum  += roll  * current.confidence;
  }
  
  o.confidence /= confidence_sum;
  
  o.pose_stamped.pose.position.x /= confidence_sum;
  o.pose_stamped.pose.position.y /= confidence_sum;
  o.pose_stamped.pose.position.z /= confidence_sum;
  
  yaw_sum   /= confidence_sum;
  pitch_sum /= confidence_sum;
  roll_sum  /= confidence_sum;
  
  o.pose_stamped.pose.orientation
    = tf::createQuaternionMsgFromRollPitchYaw(roll_sum, pitch_sum, yaw_sum);
  
  return o;
}

void SemanticMap::ObjectQueue::add(thesis::ObjectInstance& o)
{
  // enqueue
  deque.push_back(o);
  // dequeue
  if((int) deque.size() > memory_size)
  {
    deque.pop_front();
  }
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

void SemanticMap::add(thesis::ObjectInstance& object, float min_distance)
{
  cv::Point3f p;
  p.x = object.pose_stamped.pose.position.x;
  p.y = object.pose_stamped.pose.position.y;
  p.z = object.pose_stamped.pose.position.z;
  // Check for already stored objects of the same type
  std::vector<std::vector<ObjectQueue>::iterator> existing_objects;
  if(exists(object.type_id))
  {
    std::vector<ObjectQueue>::iterator iter = map[object.type_id].begin();
    for(; iter < map[object.type_id].end(); iter++)
    {
      thesis::ObjectInstance temp = iter->combined();
      cv::Point3f p_;
      p_.x = temp.pose_stamped.pose.position.x;
      p_.y = temp.pose_stamped.pose.position.y;
      p_.z = temp.pose_stamped.pose.position.z;
      if(dist3f(p, p_) <= min_distance)
      {
        existing_objects.push_back(iter);
      }
    }
  }
  // If more than one object of this type exist at the same position...
  if(existing_objects.size() > 1)
  {
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
    // ...update its entry
    closest->add(object);
    // ...and warn the user
    ROS_WARN("Semantic map: ");
    ROS_WARN("  Multiple existing objects of type %s at position (%f, %f, %f).",
      object.type_id.c_str(),
      object.pose_stamped.pose.position.x,
      object.pose_stamped.pose.position.y,
      object.pose_stamped.pose.position.z
    );
  }
  // If one object of this type already exists at the same position...
  else if(existing_objects.size() == 1)
  {
    ROS_INFO("Semantic map: ");
    ROS_INFO("  Updating %s at (%f, %f, %f).",
      object.type_id.c_str(),
      object.pose_stamped.pose.position.x,
      object.pose_stamped.pose.position.y,
      object.pose_stamped.pose.position.z
    );
    // ...update the entry
    existing_objects.front()->add(object);
  }
  // If there isn't already an object of this type...
  else
  {
    ROS_INFO("Semantic map: ");
    ROS_INFO("  Adding %s at (%f, %f, %f).",
      object.type_id.c_str(),
      object.pose_stamped.pose.position.x,
      object.pose_stamped.pose.position.y,
      object.pose_stamped.pose.position.z
    );
    // ...create a new entry
    ObjectQueue entry;
    entry.add(object);
    map[object.type_id].push_back(entry);
  }
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
