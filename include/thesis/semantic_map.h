/**
 * @author Carsten Könemann
 */
#ifndef __SEMANTIC_MAP__
#define __SEMANTIC_MAP__

#include <thesis/ObjectInstance.h>

#include <opencv2/core/core.hpp>

#include <tf/tf.h>

#include <thesis/uuid.h>

#include <deque>

class SemanticMap
{
  public:
    /**
     * Constructor.
     */
    SemanticMap(const int memory_size=0, const bool debug=true) : memory_size(memory_size), debug(debug)
    {
      // Starting position
      currentPosition.x = 0;
      currentPosition.y = 0;
      currentPosition.z = 0;
    };
    
    /**
     * Default destructor.
     */
    ~SemanticMap()
    {
      //
    };
    
    /**
     * @param Check if an entry for this ID exists in the database.
     */
    bool exists(std::string id);
    
    /**
     * Needed to evaluate an entry by distance.
     *
     * @param Use the current camera position.
     */
    void setCurrentPosition(cv::Point3f p);
    
    /**
     * Attempt to merge duplicates and remove false positives.
     */
    void cleanup(double age_threshold, unsigned int min_confirmations);
    
    // Getters
    void getAll(std::vector<thesis::ObjectInstance>& out);
    
    void getByID(std::string id, std::vector<thesis::ObjectInstance>& out);
    
    void getByIDAtPosition(std::string id,
                           cv::Point3f p,
                           std::vector<thesis::ObjectInstance>& out,
                           float max_distance=0.0f);
                           
    void getByPosition(cv::Point3f p,
                       std::vector<thesis::ObjectInstance>& out,
                       float max_distance=0.0f);
    
    
    
    // Add object to this maps storage
    void add(const thesis::ObjectInstance& object, float min_distance=0.0f);
  
  protected:
    class ObjectQueue
    {
      public:
        /**
         * Constructor.
         */
        ObjectQueue(const int memory_size=0) : memory_size(memory_size)
        {
          // Initialize UUID
          id = uuid_msgs::random();
          // Remember time of initialization
          stamp = ros::Time::now();
          // For debugging purposes
          ROS_INFO("SemanticMap::ObjectQueue(%i);", memory_size);
        };
        
        /**
         * Default destructor.
         */
        ~ObjectQueue()
        {
          //
        };
        
        /**
         * @return Time since initialization in seconds.
         */
        double age();
        
        /**
         * @return Number of instances currently stored in this queue.
         */
        size_t size();
        
        /**
         * @return Weighted average of all values contained in this queue.
         */
        thesis::ObjectInstance combined();
        
        /**
         * @param Add object and remove oldest entry (if exceeding max size).
         */
        void add(thesis::ObjectInstance o);
        
      protected:
        /**
         * Universally unique identifier.
         */
        boost::uuids::uuid id;
      
        /**
         * Using a std::deque as a base to expand upon.
         */
        std::deque<thesis::ObjectInstance> deque;
        
        /**
         * Maximum number of values to store for averaging.
         */
        int memory_size;
        
        /**
         * Time of initialization.
         */
        ros::Time stamp;
    };

    struct EvaluationComparator
    {
      public:
        /** 
         * non-static Comparator needs to know its containing object.
         */
        SemanticMap* containingObject;
        
        /**
         * Constructor.
         */
        EvaluationComparator(SemanticMap* o) : containingObject(o)
        {
          //
        }
      
        /**
         * Compare-function.
         */
        bool operator() (const thesis::ObjectInstance& a,
                         const thesis::ObjectInstance& b)
        {
          return evaluate(a, containingObject->currentPosition)
               < evaluate(b, containingObject->currentPosition);
        }
      
      protected:
        /**
         * Evaluation-function for perceived objects.
         * Used to sort perceived objects by relevance.
         */
        double evaluate(const thesis::ObjectInstance& object, cv::Point3f p_c);
    };
  
    /**
     * Hold all elements of the database, with fast access to entrys by ID.
     */
    std::map<std::string, std::vector<ObjectQueue> > map;
    
    /**
     * Maximum number of values to store per entry for averaging.
     */
    int memory_size;
    
    /**
     *
     */
    cv::Point3f currentPosition;
    
    /**
     *
     */
    bool debug;
};

#endif //__SEMANTIC_MAP__
