/**
 * @author Carsten Könemann
 */

#include <thesis/semantic_map.h>

#include <thesis/math.h>
#include <thesis/math3d.h>

#include <tf/transform_datatypes.h>

inline float evaluate(const thesis::ObjectStamped& o)
{
  // TODO
  return 0.0f;
}

struct EvaluationComparator
{
  bool operator() (const thesis::ObjectStamped& a, const thesis::ObjectStamped& b)
  {
    return evaluate(a) < evaluate(b);
  }
};

SemanticMap::SemanticMap()
{
  // Default constructor
}

SemanticMap::~SemanticMap()
{
  // Default destructor
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

bool SemanticMap::update(thesis::ObjectStamped& object, unsigned int at)
{
  if(exists(object.object_id))
  {
    std::vector<thesis::ObjectStamped>* existing_objects = &(map.at(object.object_id));
    if(existing_objects->size() > at)
    {
      thesis::ObjectStamped temp = existing_objects->at(at);
      if(temp.accuracy < INT_MAX - 2)
      {
        existing_objects->at(at).accuracy++;
      }
      // Update object position
      #define current_object existing_objects->at(at)
      #define object_position object_pose.pose.position
      current_object.object_position.x = average(temp.object_position.x, object.object_position.x, temp.accuracy, 1);
      current_object.object_position.y = average(temp.object_position.y, object.object_position.y, temp.accuracy, 1);
      current_object.object_position.z = average(temp.object_position.z, object.object_position.z, temp.accuracy, 1);
      // Update object orientation
      #define object_orientation object_pose.pose.orientation
      tf::Quaternion quaternion_old,
                     quaternion_new;
      tf::quaternionMsgToTF(temp.object_orientation,   quaternion_old);
      tf::quaternionMsgToTF(object.object_orientation, quaternion_new);
      double accuracy_normalized = 1.0 / INT_MAX * temp.accuracy;
      tf::Quaternion quaternion_interpolated = quaternion_old.slerp(quaternion_new, 1.0 - accuracy_normalized);
      tf::quaternionTFToMsg(quaternion_interpolated, current_object.object_orientation);
      // Success
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    return false;
  }
}

bool SemanticMap::add(thesis::ObjectStamped& object, float min_distance)
{
  cv::Point3f p, p_;
  p_.x = object.object_pose.pose.position.x;
  p_.y = object.object_pose.pose.position.y;
  p_.z = object.object_pose.pose.position.z;
  // Check already stored objects of the same type
  std::vector<thesis::ObjectStamped>* existing_objects = &(map[object.object_id]);
  unsigned int at;
  bool exists = false;
  for(size_t i = 0; i < existing_objects->size(); i++)
  {
    cv::Point3f p;
    p.x = existing_objects->at(i).object_pose.pose.position.x;
    p.y = existing_objects->at(i).object_pose.pose.position.y;
    p.z = existing_objects->at(i).object_pose.pose.position.z;
    // If there is already an object of the same type close to the new objects position,
    // update its position instead of adding the new object
    if(dist3f(p, p_) < min_distance)
    {
      if(exists)
      {
        return false;
      }
      exists = true;
      at = i;
    }
  }
  // If an object of this type already exists at the same position,
  // update the entry
  if(exists)
  {
    return update(object, at);
  }
  // If there isn't already an object of this type,
  // create a new entry
  else
  {
    existing_objects->push_back(object);
    return true;
  }
}

void SemanticMap::getAll(std::vector<thesis::ObjectStamped>& out)
{
  for(ObjectMap::iterator it = map.begin(); it != map.end(); it++)
  {
    out.insert(out.end(), it->second.begin(), it->second.end());
  }
}

void SemanticMap::getByID(std::string id, std::vector<thesis::ObjectStamped>& out)
{
  out = map[id];
}

void SemanticMap::getByIDAtPosition(std::string id, cv::Point3f p, std::vector<thesis::ObjectStamped>& out, float max_distance)
{
  std::vector<thesis::ObjectStamped> objects;
  getByID(id, objects);
  for(size_t i = 0; i < objects.size(); i++)
  {
    cv::Point3f p_;
    p_.x = objects[i].object_pose.pose.position.x;
    p_.y = objects[i].object_pose.pose.position.y;
    p_.z = objects[i].object_pose.pose.position.z;
    if(dist3f(p, p_) < max_distance)
    {
      out.push_back(objects[i]);
    }
  }
}

void SemanticMap::getByPosition(cv::Point3f p, std::vector<thesis::ObjectStamped>& out, float max_distance)
{
  for(ObjectMap::iterator it = map.begin(); it != map.end(); it++)
  {
    for(size_t i = 0; i < it->second.size(); i++)
    {
      cv::Point3f p_;
      p_.x = it->second[i].object_pose.pose.position.x;
      p_.y = it->second[i].object_pose.pose.position.y;
      p_.z = it->second[i].object_pose.pose.position.z;
      if(dist3f(p, p_) <= max_distance)
      {
        out.push_back(it->second[i]);
      }
    }
  }
}
