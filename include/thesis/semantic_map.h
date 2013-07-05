/**
 * @author Carsten Könemann
 */
#ifndef __SEMANTIC_MAP__
#define __SEMANTIC_MAP__

#include <thesis/ObjectStamped.h>

#include <opencv2/core/core.hpp>

class SemanticMap
{
  public:
    // Default constructor
    SemanticMap();
    // Default destructor
    ~SemanticMap();
  
    // Add object to this maps storage
    bool add(thesis::ObjectStamped& object, float min_distance=0.0f);
    
    // Getters
    void getAll(std::vector<thesis::ObjectStamped>& out);
    void getByID(std::string id, std::vector<thesis::ObjectStamped>& out);
    void getByIDAtPosition(std::string id, cv::Point3f p, std::vector<thesis::ObjectStamped>& out, float max_distance=0.0f);
    void getByPosition(cv::Point3f p, std::vector<thesis::ObjectStamped>& out, float max_distance=0.0f);
  
  protected:
    // Store objects sorted by type for fast access
    typedef std::map<std::string, std::vector<thesis::ObjectStamped> > ObjectMap;
    ObjectMap map;
    
    bool exists(std::string id);
    
    bool update(thesis::ObjectStamped& object, unsigned int at);
};

#endif //__SEMANTIC_MAP__
