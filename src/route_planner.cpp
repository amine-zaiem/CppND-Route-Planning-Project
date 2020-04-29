#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(
    RouteModel &model,
    float start_x,
    float start_y,
    float end_x,
    float end_y):
    m_Model(model)
{
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &model.FindClosestNode(start_x, start_y);
    end_node   = &model.FindClosestNode(end_x, end_y);

}

/*
 * Implement the CalculateHValue method.
 */
float RoutePlanner::CalculateHValue(RouteModel::Node const *node)
{

    // Use the distance to the end_node for the h value.
    return node->distance(*end_node);
}

/*
 * A helper to expand the current node by adding all unvisited neighbors to the open list
 */
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node)
{
    // Populate current_node.neighbors vector with all the neighbors.
    current_node->FindNeighbors();

    //For each node in current_node.neighbors
    for ( auto item : current_node->neighbors)
    {
        // set the parent, the h_value, the g_value
        item->h_value  = CalculateHValue(item);
        item->g_value  = current_node->g_value + current_node->distance(*item);
//        item->g_value += item->distance(*current_node);
        item->parent   = current_node;

        //add the neighbor to open_list and set the node's visited attribute to true
        open_list.push_back(item);
        item->visited  = true;
    }
}

/**
 * A helper to compare the F values of two cells.
 */
bool Compare(const RouteModel::Node* a, const RouteModel::Node* b)
{
  float f1 = a->h_value + a->g_value; // f1 = g1 + h1
  float f2 = b->h_value + b->g_value; // f2 = g2 + h2
  return f1 > f2;
}

/**
 * A helper to to sort the open list and return the next node.
 */
RouteModel::Node *RoutePlanner::NextNode()
{
    // Sort the open_list according to the sum of the h value and g value
    sort(open_list.begin(), open_list.end(), Compare);
    // Create a pointer to the node in the list with the lowest sum
    RouteModel::Node *lowestSumNodePointer = open_list.back();
    // Remove the node from the open_list
    open_list.pop_back();

    return lowestSumNodePointer;

}

/**
 * A helper to return the final path found from your A* search.
 */
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(
     RouteModel::Node *current_node)
{
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // Iterate through the chain of parents until finding starting node
    while ( current_node->parent != nullptr  )
    {
        // Add the distance from the node to its parent to the distance variable
        RouteModel::Node* parentNode = current_node->parent;
        distance = distance + current_node->distance(*parentNode);
        // Insert current_node into path_found vector
        path_found.push_back(*current_node);

        current_node = current_node->parent;
    }

    // Inserting the first element into the path_found vector
    path_found.push_back(*current_node);
    // Reversin vector order
    std::reverse(path_found.begin(), path_found.end());
    // Multiply the distance by the scale of the map to get meters.
    distance *= m_Model.MetricScale();
    return path_found;

}

/**
 * The A* Search algorithm
 */
void RoutePlanner::AStarSearch()
{
   // initialization
   AddToOpen(start_node);
   RouteModel::Node *current_node = nullptr;

   while ( open_list.size() > 0 )
   {
      // Get the next node
      current_node = NextNode();
      // Check if we're done.
      if ( CalculateHValue( current_node) == 0.0f)
      {
         m_Model.path = ConstructFinalPath(current_node);
         return;
      }
      // If we're not done, expand search to current node's neighbors.
      else
      {
         AddNeighbors(current_node);
      }
   }
}
