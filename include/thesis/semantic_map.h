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
    
    // Getters
    void getAll(std::vector<thesis::ObjectStamped>& out);
    void getByID(std::string id, std::vector<thesis::ObjectStamped>& out);
    void getByIDAtPosition(std::string id, cv::Point3f p, std::vector<thesis::ObjectStamped>& out, float max_distance=0.0f);
    void getByPosition(cv::Point3f p, std::vector<thesis::ObjectStamped>& out, float max_distance=0.0f);
    
    // Setters
    void setCurrentPosition(cv::Point3f p);
    
    // Add object to this maps storage
    bool add(thesis::ObjectStamped& object, float min_distance=0.0f);
  
  protected:
    static double evaluate(const thesis::ObjectStamped& object, cv::Point3f p_c);
  
    struct EvaluationComparator
    {
      // non-static Comparator needs to know its containing object
      SemanticMap* containingObject;
      
      // Constructor
      EvaluationComparator(SemanticMap* o) : containingObject(o)
      {
        //
      };
    
      // Compare-function
      bool operator() (const thesis::ObjectStamped& a, const thesis::ObjectStamped& b)
      {
        return evaluate(a, containingObject->currentPosition) < evaluate(b, containingObject->currentPosition);
      }
    };
  
    // Store objects sorted by type for fast access
    typedef std::map<std::string, std::vector<thesis::ObjectStamped> > ObjectMap;
    ObjectMap map;
    
    //
    cv::Point3f currentPosition;
    
    bool exists(std::string id);
    
    bool update(thesis::ObjectStamped& object, unsigned int at);
};

#endif //__SEMANTIC_MAP__
