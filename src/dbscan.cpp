/**
 * Density based clustering using the DBScan algorithm.
 * As presented in the Uni Bremen WS2011/2012 course "Machine Learning".
 *
 * @author Carsten Könemann
 */

#include <thesis/dbscan.h>

DBScanner::DBScanner()
{
  // Default constructor
}

DBScanner::~DBScanner()
{
  // Default destructor
}

std::vector<Cluster> DBScanner::scan(Cluster& keypoints, uint min_points, uint max_distance)
{
  std::vector<Cluster> clustering;
  // Initialze default mapping: No keypoint has been visited yet
  VisitMap visited;
  for(size_t i = 0; i < keypoints.size(); i++)
  {
    visited[keypoints[i]] = false;
  }
  // Expand all unvisited keypoints
  for(size_t i = 0; i < keypoints.size(); i++)
  {
    if(!visited[keypoints[i]])
    {
      Cluster nextCluster;
      if(expand(keypoints[i], keypoints, visited, min_points, max_distance, nextCluster))
      {
        clustering.push_back(nextCluster);
      }
    }
  }
  return clustering;
}

void DBScanner::connected_points(cv::KeyPoint& p, Cluster& keypoints, uint max_distance, Cluster& cp)
{
  // All points in keypoints different from p are connected to p if they are within a range of max_distance
  for(size_t i = 0; i < keypoints.size(); i++)
  {
    if(p.pt != keypoints[i].pt && distance(keypoints[i].pt, p.pt) <= max_distance)
    {
      cp.push_back(keypoints[i]);
    }
  }
}

bool DBScanner::expand(cv::KeyPoint& p, Cluster& keypoints, VisitMap& visited, uint min_points, uint max_distance, Cluster& cluster)
{
  Cluster cp;
  connected_points(p, keypoints, max_distance, cp);
  if(cp.size() >= min_points)
  {
    // The starting point is only marked visited if it is dense
    // Otherwise points at the edge of a cluster might remain unclassified
    visited[p] = true;
    // A dense point is always part of the cluster
    cluster.push_back(p);
    for(size_t i = 0; i < cp.size(); i++)
    {
      // Only expand further points if they have not been visited yet
      if(!visited[cp[i]])
      {
        visited[cp[i]] = true;
        // Density reachable points from p are points of the cluster
        cluster.push_back(cp[i]);
        // Examine indirectly density reachable points
        Cluster cp_;
        connected_points(cp[i], keypoints, max_distance, cp_);
        if(cp_.size() >= min_points)
        {
          cp.insert(cp.end(), cp_.begin(), cp_.end());
        }
      }
    }
    return true;
  }
  else
  {
    return false;
  }
}
