/**
 * Density based clustering using the DBScan algorithm.
 * As presented in the Uni Bremen WS2011/2012 course "Machine Learning".
 *
 * @author Carsten Könemann
 */
#ifndef __DBSCAN__
#define __DBSCAN__

#include <thesis/comparator.h>

typedef std::vector<cv::KeyPoint> Cluster;
typedef std::map<cv::KeyPoint, bool, CvKeyPointComparator> VisitMap;

class DBScanner
{
  public:
    // Default constructor
    DBScanner();
    // Default destructor
    ~DBScanner();
    
    /**
     * Do a DBScan on keypoints.
     * A keypoint is dense if at least min_points keypoints are density connected to it.
     * A keypoint is density connected to another, if it is within a range of max_distance.
     */
    std::vector<Cluster> scan(Cluster& keypoints, uint min_points, uint max_distance);

  protected:
    /**
     * Get all density connected / directly density reachable points for p.
     */
    void connected_points(cv::KeyPoint& p, Cluster& keypoints, uint max_distance, Cluster& cp);
    
    /**
     * Search for density reachable points from p and add them to the current cluster.
     */
    bool expand(cv::KeyPoint& p, Cluster& keypoints, VisitMap& visited, uint min_points, uint max_distance, Cluster& cluster);
};

#endif //__DBSCAN__
