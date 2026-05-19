#include <algorithm>
#include <chrono>
#include <cfloat>
#include <climits>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <queue>
#include <vector>
#include <unordered_map>
#include <unordered_set>

#include <boost/filesystem.hpp>
#include <ros/package.h>

#include "dmapf/clingo_wrap.h"
#include "dmapf/color.h"
#include "dmapf/hungarian.h"
#include "dmapf/utility.h"

#define ONLY_ROBOTS                 // Deal with problems with only robots (no shelves or orders)
#define VISUALIZE                   // Visualize the partitioned map
#define MAX_KMEANS_ITERATION 100    // The maximum number of iterations for k-means algorithms
#define MIN_PERCENT_NODES 0.25      // Determine how small areas will be merged with their neighbor
#define MAX_CPUS 32                 // The minimum number of CPU threads a single node in a HPC cluster has
#define MIN_NUM_FREE_NODES 4        // This value should be greater or equal to the value used in the solver

// Only use one of the following distance functions
#undef DISTANCE_MANHATTAN
#undef DISTANCE_EUCLIDEAN
#define DISTANCE_REAL
#undef DISTANCE_REAL_CUSTOM

struct Coor
{
  int x;
  int y;

  Coor() : x(0), y(0) {}
  Coor(int x, int y) : x(x), y(y) {}

  bool operator==(const Coor& coor) const
  {
    return x == coor.x && y == coor.y;
  }

  void operator=(const Coor& coor)
  {
    x = coor.x;
    y = coor.y;
  }

  friend std::ostream& operator<< (std::ostream& os, const Coor& coor) {
      return os << "(" << coor.x << "," << coor.y << ")";
  }
};

struct CoorHasher
{
  std::size_t operator() (const Coor& key) const
  {
    // Assume size_t = 8 Bytes
    // |-- x (32 bits) --|-- y (32 bits) --|
    return ((std::size_t)key.x << (sizeof(std::size_t) << 2)) | key.y;

//    // Below might be better for complex data, but in our case it is 3 times slower than the above
//    std::size_t seed = 0;
//    boost::hash_combine(seed, boost::hash_value(key.x));
//    boost::hash_combine(seed, boost::hash_value(key.y));
//    return seed;
  }
};

struct CoorCompare
{
  bool operator()(const Coor& lhs, const Coor& rhs) const
  {
    return (lhs.y < rhs.y) || (lhs.y == rhs.y && lhs.x < rhs.x);
  }
};

struct Centroid
{
  double x;
  double y;

  Centroid(): x(0), y(0) {}
  Centroid(double x, double y): x(x), y(y) {}

  void operator=(const Centroid& centroid)
  {
    x = centroid.x;
    y = centroid.y;
  }
};

struct Connection
{
  int area;
  int solver;
  int num_links;
  std::vector<Coor> corners;
};

struct AreaInfo
{
  int area;
  int num_nodes;
  Coor min_node;
  Coor max_node;
  std::vector<Connection> connections;

  std::string str() const
  {
    std::string str(std::to_string(area) + " " +
                    std::to_string(num_nodes) + " " +
                    std::to_string(min_node.x) + " " +
                    std::to_string(min_node.y) + " " +
                    std::to_string(max_node.x) + " " +
                    std::to_string(max_node.y) + " " +
                    std::to_string(connections.size()));
    for (const auto& connection : connections)
    {
      str += " " + std::to_string(connection.area) + " " + std::to_string(connection.solver) + " " + std::to_string(connection.num_links) +
             " " + std::to_string(connection.corners.size());
      for (const Coor& corner : connection.corners)
      {
        str += " " + std::to_string(corner.x) + " " + std::to_string(corner.y);
      }
    }
    return str;
  }
};

struct RobotInfo
{
  int robot;
  int area_to;
  Coor coor_from;
  Coor coor_to;

  std::string str() const
  {
    return std::to_string(robot) + " " + std::to_string(area_to) +
           " " + std::to_string(coor_from.x) + " " + std::to_string(coor_from.y) +
           " " + std::to_string(coor_to.x) + " " + std::to_string(coor_to.y);
  }
};

void exit_failure(const std::string& message)
{
  std::cerr << message << std::endl;
  std::exit(EXIT_FAILURE);
}

void exit_failure(const std::string& message, clingo_control_t* ctl)
{
  clingo_control_free(ctl);
  exit_failure(message);
}

int get_tier(int x, int y, const Coor& tile, const std::unordered_set<Coor, CoorHasher>& explored_tiles_set)
{
  Coor left_tile(tile.x - 1, tile.y);
  Coor right_tile(tile.x + 1, tile.y);
  Coor top_tile(tile.x, tile.y - 1);
  Coor down_tile(tile.x, tile.y + 1);

  int tier = 0;
  if (left_tile.x <= 0 || explored_tiles_set.find(left_tile) != explored_tiles_set.end())
  {
    ++tier;
  }
  if (right_tile.x > x || explored_tiles_set.find(right_tile) != explored_tiles_set.end())
  {
    ++tier;
  }
  if (top_tile.y <= 0 || explored_tiles_set.find(top_tile) != explored_tiles_set.end())
  {
    ++tier;
  }
  if (down_tile.y > y || explored_tiles_set.find(down_tile) != explored_tiles_set.end())
  {
    ++tier;
  }
  return tier;
}

// Find the nearest coordinate to the obstacle
std::unordered_map<Coor, Coor, CoorHasher> g_obstacle_coor_map;
Coor find_nearest_coor(const Coor& obstacle, const Coor& min_coor, const Coor& max_coor, const std::unordered_set<Coor, CoorHasher>& coor_set)
{
  const auto obstacle_coor_map_it = g_obstacle_coor_map.find(obstacle);
  if (obstacle_coor_map_it != g_obstacle_coor_map.end())
  {
    return obstacle_coor_map_it->second;
  }

  struct Node
  {
    const Coor coor;
    const int parent;
  };

  std::vector<Node> coor_nodes{Node{obstacle, -1}};
  std::unordered_set<Coor, CoorHasher> visited{obstacle};
  std::queue<int> queue;
  queue.push(0);
  while (!queue.empty())
  {
    const int i = queue.front();
    queue.pop();

    // Found the nearest node
    if (coor_set.find(coor_nodes[i].coor) != coor_set.end())
    {
      // Save the path already found for fast future lookup
      int p = coor_nodes[i].parent;
      while (p >= 0)
      {
        g_obstacle_coor_map.emplace(coor_nodes[p].coor, coor_nodes[i].coor);
        p = coor_nodes[p].parent;
      }
      return coor_nodes[i].coor;
    }

    // Otherwise, generate child nodes
    if (coor_nodes[i].coor.x - 1 >= min_coor.x)  // Left
    {
      const Coor coor(coor_nodes[i].coor.x - 1, coor_nodes[i].coor.y);
      if (visited.find(coor) == visited.end())
      {
        visited.emplace(coor);
        coor_nodes.emplace_back(Node{std::move(coor), i});
        queue.push((int)coor_nodes.size() - 1);
      }
    }
    if (coor_nodes[i].coor.x + 1 <= max_coor.x)  // Right
    {
      const Coor coor(coor_nodes[i].coor.x + 1, coor_nodes[i].coor.y);
      if (visited.find(coor) == visited.end())
      {
        visited.emplace(coor);
        coor_nodes.emplace_back(Node{std::move(coor), i});
        queue.push((int)coor_nodes.size() - 1);
      }
    }
    if (coor_nodes[i].coor.y - 1 >= min_coor.y)  // Up
    {
      const Coor coor(coor_nodes[i].coor.x, coor_nodes[i].coor.y - 1);
      if (visited.find(coor) == visited.end())
      {
        visited.emplace(coor);
        coor_nodes.emplace_back(Node{std::move(coor), i});
        queue.push((int)coor_nodes.size() - 1);
      }
    }
    if (coor_nodes[i].coor.y + 1 <= max_coor.y)  // Down
    {
      const Coor coor(coor_nodes[i].coor.x, coor_nodes[i].coor.y + 1);
      if (visited.find(coor) == visited.end())
      {
        visited.emplace(coor);
        coor_nodes.emplace_back(Node{std::move(coor), i});
        queue.push((int)coor_nodes.size() - 1);
      }
    }
  }

  // This should not happen
  std::cerr << RED << "Cannot find the nearest node for " << obstacle << RESET << std::endl;
  return obstacle;
}

#if defined(DISTANCE_REAL) || defined(DISTANCE_REAL_CUSTOM)
struct Rank
{
  Coor coor;
  double dist;

  friend bool operator< (const Rank& lhs, const Rank& rhs)
  {
    return lhs.dist > rhs.dist;
  }
};

std::unordered_map<Coor, std::unordered_map<Coor, double, CoorHasher>, CoorHasher> g_sink_source_dist_map;
double find_real_distance(const Coor& source, const Coor& sink, const std::unordered_set<Coor, CoorHasher>& coor_set)
{
  // Check whether distances to the sink have been calculated
  const auto sink_source_dist_map_it = g_sink_source_dist_map.find(sink);
  if (sink_source_dist_map_it != g_sink_source_dist_map.end())
  {
    const auto source_dist_map_it = sink_source_dist_map_it->second.find(source);
    if (source_dist_map_it != sink_source_dist_map_it->second.end())
    {
      return source_dist_map_it->second;
    }

    // This should not happen
    std::cerr << RED << "Cannot find the real distance between " << source << " and " << sink << RESET << std::endl;
    return DBL_MAX;
  }

  // Calculate distances to the sink
  auto& source_dist_map = g_sink_source_dist_map.emplace(sink, std::unordered_map<Coor, double, CoorHasher>{}).first->second;
  source_dist_map.reserve(coor_set.size());

#ifdef DISTANCE_REAL
  std::queue<Rank> queue;
  source_dist_map.emplace(sink, 0.0);
#else
  std::priority_queue<Rank> queue;
#endif

  queue.push(Rank{sink, 0.0});
  while (!queue.empty())
  {
#ifdef DISTANCE_REAL
    Rank rank = queue.front();
    queue.pop();
#else
    Rank rank = queue.top();
    queue.pop();
    if (source_dist_map.find(rank.coor) != source_dist_map.end())
    {
      continue;
    }
    source_dist_map.emplace(rank.coor, rank.dist);
#endif

    const Coor left(rank.coor.x - 1, rank.coor.y);
    if (coor_set.find(left) != coor_set.end())
    {
#ifdef DISTANCE_REAL
      if (source_dist_map.find(left) == source_dist_map.end())
      {
        const double dist = rank.dist + 1.0;
        source_dist_map.emplace(left, dist);
        queue.emplace(Rank{left, dist});
      }
#else
      const int dx1 = rank.coor.x - sink.x;
      const int dy1 = rank.coor.y - sink.y;
      const int dx2 = left.x - sink.x;
      const int dy2 = left.y - sink.y;
      const double dist = rank.dist + std::fabs(std::sqrt(dx1 * dx1 + dy1 * dy1) - std::sqrt(dx2 * dx2 + dy2 * dy2));
      queue.emplace(Rank{left, dist});
#endif
    }

    const Coor right(rank.coor.x + 1, rank.coor.y);
    if (coor_set.find(right) != coor_set.end())
    {
#ifdef DISTANCE_REAL
      if (source_dist_map.find(right) == source_dist_map.end())
      {
        const double dist = rank.dist + 1.0;
        source_dist_map.emplace(right, dist);
        queue.emplace(Rank{right, dist});
      }
#else
      const int dx1 = rank.coor.x - sink.x;
      const int dy1 = rank.coor.y - sink.y;
      const int dx2 = right.x - sink.x;
      const int dy2 = right.y - sink.y;
      const double dist = rank.dist + std::fabs(std::sqrt(dx1 * dx1 + dy1 * dy1) - std::sqrt(dx2 * dx2 + dy2 * dy2));
      queue.emplace(Rank{right, dist});
#endif
    }

    const Coor up(rank.coor.x, rank.coor.y - 1);
    if (coor_set.find(up) != coor_set.end())
    {
#ifdef DISTANCE_REAL
      if (source_dist_map.find(up) == source_dist_map.end())
      {
        const double dist = rank.dist + 1.0;
        source_dist_map.emplace(up, dist);
        queue.emplace(Rank{up, dist});
      }
#else
      const int dx1 = rank.coor.x - sink.x;
      const int dy1 = rank.coor.y - sink.y;
      const int dx2 = up.x - sink.x;
      const int dy2 = up.y - sink.y;
      const double dist = rank.dist + std::fabs(std::sqrt(dx1 * dx1 + dy1 * dy1) - std::sqrt(dx2 * dx2 + dy2 * dy2));
      queue.emplace(Rank{up, dist});
#endif
    }

    const Coor down(rank.coor.x, rank.coor.y + 1);
    if (coor_set.find(down) != coor_set.end())
    {
#ifdef DISTANCE_REAL
      if (source_dist_map.find(down) == source_dist_map.end())
      {
        const double dist = rank.dist + 1.0;
        source_dist_map.emplace(down, dist);
        queue.emplace(Rank{down, dist});
      }
#else
      const int dx1 = rank.coor.x - sink.x;
      const int dy1 = rank.coor.y - sink.y;
      const int dx2 = down.x - sink.x;
      const int dy2 = down.y - sink.y;
      const double dist = rank.dist + std::fabs(std::sqrt(dx1 * dx1 + dy1 * dy1) - std::sqrt(dx2 * dx2 + dy2 * dy2));
      queue.emplace(Rank{down, dist});
#endif
    }
  }

  const auto source_dist_map_it = source_dist_map.find(source);
  if (source_dist_map_it != source_dist_map.end())
  {
    return source_dist_map_it->second;
  }

  // This should not happen
  std::cerr << RED << "Cannot find the real distance between " << source << " and " << sink << RESET << std::endl;
  return DBL_MAX;
}
#endif

int main(int argc, char **argv)
{
  enum class Partitioner {BKMEANS, BKMEANS_SQ, KMEANS, NAIVE};
  const std::string PROGRAM_NAME("gendiv");

  bool only_gen = false;
  bool only_div = false;
  std::string problem_lp;
  int x = 0;
  int y = 0;
  int r = 0;
  Partitioner partitioner = Partitioner::BKMEANS;
  int dx = 0;
  int dy = 0;
  int k = 0;
  int seed = std::time(nullptr);
  bool connected = true;
  int n = 0;
  int num_solvers = 0;

  // The generator is reinitialized to its initial value if seed is set to 1, so we make sure it is not 1
  while (seed == 1)
  {
    seed = std::time(nullptr);
  }

  // Evaluate input arguments
  for (int i = 1; i < argc; ++i)
  {
    if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "-h"))
    {
      goto help;
    }
    else if (!strcmp(argv[i], "--gen") || !strcmp(argv[i], "-g"))
    {
      only_gen = true;
      continue;
    }
    else if (!strcmp(argv[i], "--div") || !strcmp(argv[i], "-d"))
    {
      only_div = true;
      continue;
    }
    else if (!strcmp(argv[i], "--random"))
    {
      connected = false;
      continue;
    }
    else if (!strcmp(argv[i], "--bk-means"))
    {
      partitioner = Partitioner::BKMEANS;
      continue;
    }
    else if (!strcmp(argv[i], "--bk-means-sq"))
    {
      partitioner = Partitioner::BKMEANS_SQ;
      continue;
    }
    else if (!strcmp(argv[i], "--k-means"))
    {
      partitioner = Partitioner::KMEANS;
      continue;
    }
    else if (!strcmp(argv[i], "--naive"))
    {
      partitioner = Partitioner::NAIVE;
      continue;
    }
    else if (i < argc - 1)  // for options that require 1 argument
    {
      if (!strcmp(argv[i], "--file") || !strcmp(argv[i], "-f"))
      {
        problem_lp = argv[++i];
        continue;
      }
      else
      {
        try
        {
          if (!strcmp(argv[i], "--col") || !strcmp(argv[i], "-x"))
          {
            x = std::stoi(argv[++i]);
            continue;
          }
          else if (!strcmp(argv[i], "--row") || !strcmp(argv[i], "-y"))
          {
            y = std::stoi(argv[++i]);
            continue;
          }
          else if (!strcmp(argv[i], "--nodes") || !strcmp(argv[i], "-n"))
          {
            n = std::stoi(argv[++i]);
            continue;
          }
          else if (!strcmp(argv[i], "--robots") || !strcmp(argv[i], "-r"))
          {
            r = std::stoi(argv[++i]);
            continue;
          }
          else if (!strcmp(argv[i], "--solvers") || !strcmp(argv[i], "-s"))
          {
            num_solvers = std::stoi(argv[++i]);
            continue;
          }
          else if (!strcmp(argv[i], "--clusters") || !strcmp(argv[i], "-k"))
          {
            k = std::stoi(argv[++i]);
            continue;
          }
          else if (!strcmp(argv[i], "--dcol") || !strcmp(argv[i], "-dx"))
          {
            dx = std::stoi(argv[++i]);
            continue;
          }
          else if (!strcmp(argv[i], "--drow") || !strcmp(argv[i], "-dy"))
          {
            dy = std::stoi(argv[++i]);
            continue;
          }
          else if (!strcmp(argv[i], "--seed"))
          {
            seed = std::stoi(argv[++i]);
            continue;
          }
        }
        catch (const std::exception& e)
        {
          goto help;
        }
      }
    }

    help:
    std::cout << "Usage: rosrun dmapf " << PROGRAM_NAME << " [options]" << std::endl
              << "Options:" << std::endl
              << "  --help, -h      Print help and exit" << std::endl
              << "  --file, -f      Set a full path to generate a problem (1)" << std::endl
              << "                  or divide a problem (2)" << std::endl
              << "(1)-------------------------------------------------------------" << std::endl
              << "  --gen, -g       Only generate a problem instance " << std::endl
              << "  --col, -x       Set the number of columns" << std::endl
              << "  --row, -y       Set the number of rows" << std::endl
              << "  --nodes, -n     Set the number of nodes" << std::endl
              << "  --robots, -r    Set the number of robots" << std::endl
              << "  --seed          Set the seed for randomization" << std::endl
              << "  --random        Totally random (some goals may be unreachable)" << std::endl
              << "(2)-------------------------------------------------------------" << std::endl
              << "  --div, -d       Only divide a problem instance" << std::endl
              << "  --solvers, -s   Set the number of solvers" << std::endl
              << "  --clusters, -k  Set the number of clusters" << std::endl
              << "    --bk-means    Partition the problem using balanced k-means (default)" << std::endl
              << "    --bk-means-sq Partition the problem using balanced k-means with square matrix" << std::endl
              << "    --k-medoids   Partition the problem using k-means" << std::endl
              << "  --dcol, -dx     Set the number of divided columns" << std::endl
              << "  --drow, -dy     Set the number of divided row" << std::endl
              << "    --naive       Partition the problem naively" << std::endl
              << std::endl
              << "Full mode requires -x, -y" << std::endl
              << "Generate-only mode (-g) requires -x, -y" << std::endl
              << "Divide-only mode (-d) requires -f" << std::endl;
    return EXIT_FAILURE;
  }

  // Check the input arguments
  if (n == 0)
  {
    n = x * y;
  }

  bool invalid_arguments = false;
  if (only_gen || !only_div)
  {
    if (x <= 0)
    {
      std::cerr << "-x must be positive" << std::endl;
      invalid_arguments = true;
    }
    if (y <= 0)
    {
      std::cerr << "-y must be positive" << std::endl;
      invalid_arguments = true;
    }
    if (r < 0)
    {
      std::cerr << "-r must be non-negative" << std::endl;
      invalid_arguments = true;
    }
    if (n > x * y)
    {
      std::cerr << "-n must be smaller or equal to the area x*y" << std::endl;
      invalid_arguments = true;
    }
    if (n < r)
    {
      std::cerr << "-r must be smaller or equal to the number of nodes" << std::endl;
      invalid_arguments = true;
    }
    if (!problem_lp.empty())
    {
      boost::filesystem::path p(problem_lp);
      if (!boost::filesystem::is_directory(p.parent_path()))
      {
        std::cerr << "Directory " << p.parent_path() << " does not exist" << std::endl;
        invalid_arguments = true;
      }
    }
    if (seed == 1)
    {
      std::cerr << "--seed must not be 1" << std::endl;
      invalid_arguments = true;
    }
  }

  if (!only_gen || only_div)
  {
    switch (partitioner)
    {
      case Partitioner::BKMEANS:
      case Partitioner::BKMEANS_SQ:
      case Partitioner::KMEANS:
        if (k < 0)
        {
          std::cerr << "-k must be positive" << std::endl;
          invalid_arguments = true;
        }
        break;

      case Partitioner::NAIVE:
        if (dx < 0)
        {
          std::cerr << "-dx must be non-negative" << std::endl;
          invalid_arguments = true;
        }
        if (dy < 0)
        {
          std::cerr << "-dy must be non-negative" << std::endl;
          invalid_arguments = true;
        }
        break;
    }

    if (!only_gen && only_div && problem_lp.empty())
    {
      std::cerr << "-f is required in divide-only mode" << std::endl;
      invalid_arguments = true;
    }
    if (!only_gen && only_div && !problem_lp.empty())
    {
      boost::filesystem::path p(problem_lp);
      if (!boost::filesystem::exists(p))
      {
        std::cerr << "File " << p << " does not exist" << std::endl;
        invalid_arguments = true;
      }
    }
  }

  if (invalid_arguments)
  {
    return EXIT_FAILURE;
  }

  // Set the seed for randomization
  std::srand((unsigned)seed);

  // Set output path
  const std::string project_name("dmapf");
  const std::string project_path(ros::package::getPath(project_name));
  if (project_path.empty())
  {
    std::cerr << "Cannot find project " << project_name << std::endl;
    return EXIT_FAILURE;
  }

  std::string launch_file;
  std::string domain_dir;
  std::string path;
  if (problem_lp.empty())
  {
    std::string name("x" + std::to_string(x) + "_y" + std::to_string(y) + "_n" + std::to_string(n) + "_r" + std::to_string(r));
    path = project_path + "/examples/" + name;
    problem_lp = name + ".lp";
    launch_file = name;
    domain_dir = name;
  }
  else
  {
    boost::filesystem::path p(problem_lp);
    path = p.parent_path().string();
    problem_lp = p.filename().string();
    launch_file = p.stem().string();

    std::string target = project_path + "/examples";
    std::size_t found = path.find(target);
    if (found == std::string::npos || (found + target.size() < path.size() && path[found + target.size()] != '/'))
    {
      std::cerr << problem_lp << " needs to be inside " << target << " directory" << std::endl;
      return EXIT_FAILURE;
    }

    if (found + target.size() == path.size())
    {
      domain_dir = "";
    }
    else
    {
      domain_dir = path.substr(found + target.size() + 1);
    }
  }

  // Create the output directory
  boost::filesystem::create_directory(path);

  /*
   * Generate a problem instance
   */
  if (only_gen || !only_div)
  {
    std::string seed_str("% seed = " + std::to_string(seed));
    seed_str += (connected? ", connected = true\n" : ", connected = false\n");

    const int MAP_SIZE = x * y;
    std::vector<Coor> tiles;
    tiles.reserve(MAP_SIZE);

    std::string node_str;
    std::vector<Coor> coors;
    coors.reserve(n);

    for (int i = 1; i <= y; ++i)
    {
      for (int j = 1; j <= x; ++j)
      {
        tiles.emplace_back(Coor(j,i));
      }
    }

    if (connected)
    {
      if (n > 0)
      {
        int j = rand() % MAP_SIZE;
        Coor selected_tile = tiles[j];
        coors.push_back(selected_tile);
        node_str += "init(object(node,1),value(at,(" + std::to_string(selected_tile.x) + ","
            + std::to_string(selected_tile.y) + "))).\n";

        Coor left_tile(selected_tile.x - 1, selected_tile.y);
        Coor right_tile(selected_tile.x + 1, selected_tile.y);
        Coor up_tile(selected_tile.x, selected_tile.y - 1);
        Coor down_tile(selected_tile.x, selected_tile.y + 1);

        std::unordered_set<Coor,CoorHasher> explored_tiles_set{selected_tile};
        std::unordered_map<Coor, int, CoorHasher> connected_tile_tier_map;
        std::vector<Coor> connected_tiles[4];   // four directions

        // XXX: Adjust the map sparseness here. Each weight has to be at least 1
        const int WEIGHTS[4] = {100, 1, 1, 1};

        explored_tiles_set.reserve(n);
        connected_tile_tier_map.reserve(MAP_SIZE);
        connected_tiles[0].reserve(MAP_SIZE);
        connected_tiles[1].reserve(MAP_SIZE);
        connected_tiles[2].reserve(MAP_SIZE);
        connected_tiles[3].reserve(MAP_SIZE);

        if (left_tile.x > 0)
        {
          const int LEFT_TIER = get_tier(x, y, left_tile, explored_tiles_set) - 1;
          connected_tiles[LEFT_TIER].emplace_back(left_tile);
          connected_tile_tier_map.emplace(left_tile, LEFT_TIER);
        }
        if (right_tile.x <= x)
        {
          const int RIGHT_TIER = get_tier(x, y, right_tile, explored_tiles_set) - 1;
          connected_tiles[RIGHT_TIER].emplace_back(right_tile);
          connected_tile_tier_map.emplace(right_tile, RIGHT_TIER);
        }
        if (up_tile.y > 0)
        {
          const int UP_TIER = get_tier(x, y, up_tile, explored_tiles_set) - 1;
          connected_tiles[UP_TIER].emplace_back(up_tile);
          connected_tile_tier_map.emplace(up_tile, UP_TIER);
        }
        if (down_tile.y <= y)
        {
          const int DOWN_TIER = get_tier(x, y, down_tile, explored_tiles_set) - 1;
          connected_tiles[DOWN_TIER].emplace_back(down_tile);
          connected_tile_tier_map.emplace(down_tile, DOWN_TIER);
        }

        for (int i = 2; i <= n; ++i)
        {
          j = rand()
              % (connected_tiles[0].size() * WEIGHTS[0]
                 + connected_tiles[1].size() * WEIGHTS[1]
                 + connected_tiles[2].size() * WEIGHTS[2]
                 + connected_tiles[3].size() * WEIGHTS[3]);

          int tier = 0;
          while (tier < 4)
          {
            if (j < connected_tiles[tier].size() * WEIGHTS[tier])
            {
              j /= WEIGHTS[tier];
              break;
            }
            j -= connected_tiles[tier].size() * WEIGHTS[tier];
            ++tier;
          }

          selected_tile = connected_tiles[tier][j];
          coors.push_back(selected_tile);
          node_str += "init(object(node," + std::to_string(i) + "),value(at,(" + std::to_string(selected_tile.x) + ","
              + std::to_string(selected_tile.y) + "))).\n";
          explored_tiles_set.emplace(selected_tile);
          connected_tiles[tier][j] = connected_tiles[tier].back();
          connected_tiles[tier].pop_back();

          if (tier < 3)   // unnecessary to check if tier >= 3
          {
            Coor left_tile(selected_tile.x - 1, selected_tile.y);
            Coor right_tile(selected_tile.x + 1, selected_tile.y);
            Coor up_tile(selected_tile.x, selected_tile.y - 1);
            Coor down_tile(selected_tile.x, selected_tile.y + 1);

            if (left_tile.x > 0
                && explored_tiles_set.find(left_tile) == explored_tiles_set.end())
            {
              auto connected_tile_tier_map_it = connected_tile_tier_map.find(left_tile);
              if (connected_tile_tier_map_it == connected_tile_tier_map.end())
              {
                const int LEFT_TIER = get_tier(x, y, left_tile, explored_tiles_set) - 1;
                connected_tiles[LEFT_TIER].emplace_back(left_tile);
                connected_tile_tier_map.emplace(left_tile, LEFT_TIER);
              }
              else
              {
                const int CURR_TIER = connected_tile_tier_map_it->second;
                for (int k = 0; k < connected_tiles[CURR_TIER].size(); ++k)
                {
                  if (left_tile == connected_tiles[CURR_TIER][k])
                  {
                    connected_tiles[CURR_TIER][k] = connected_tiles[CURR_TIER].back();
                    connected_tiles[CURR_TIER].pop_back();
                    if (CURR_TIER < 3)
                    {
                      connected_tiles[CURR_TIER + 1].emplace_back(left_tile);
                      connected_tile_tier_map_it->second = CURR_TIER + 1;
                    }
                  }
                }
              }
            }
            if (right_tile.x <= x
                && explored_tiles_set.find(right_tile) == explored_tiles_set.end())
            {
              auto connected_tile_tier_map_it = connected_tile_tier_map.find(right_tile);
              if (connected_tile_tier_map_it == connected_tile_tier_map.end())
              {
                const int RIGHT_TIER = get_tier(x, y, right_tile, explored_tiles_set) - 1;
                connected_tiles[RIGHT_TIER].emplace_back(right_tile);
                connected_tile_tier_map.emplace(right_tile, RIGHT_TIER);
              }
              else
              {
                const int CURR_TIER = connected_tile_tier_map_it->second;
                for (int k = 0; k < connected_tiles[CURR_TIER].size(); ++k)
                {
                  if (right_tile == connected_tiles[CURR_TIER][k])
                  {
                    connected_tiles[CURR_TIER][k] = connected_tiles[CURR_TIER].back();
                    connected_tiles[CURR_TIER].pop_back();
                    if (CURR_TIER < 3)
                    {
                      connected_tiles[CURR_TIER + 1].emplace_back(right_tile);
                      connected_tile_tier_map_it->second = CURR_TIER + 1;
                    }
                  }
                }
              }
            }
            if (up_tile.y > 0
                && explored_tiles_set.find(up_tile) == explored_tiles_set.end())
            {
              auto connected_tile_tier_map_it = connected_tile_tier_map.find(up_tile);
              if (connected_tile_tier_map_it == connected_tile_tier_map.end())
              {
                const int UP_TIER = get_tier(x, y, up_tile, explored_tiles_set) - 1;
                connected_tiles[UP_TIER].emplace_back(up_tile);
                connected_tile_tier_map.emplace(up_tile, UP_TIER);
              }
              else
              {
                const int CURR_TIER = connected_tile_tier_map_it->second;
                for (int k = 0; k < connected_tiles[CURR_TIER].size(); ++k)
                {
                  if (up_tile == connected_tiles[CURR_TIER][k])
                  {
                    connected_tiles[CURR_TIER][k] = connected_tiles[CURR_TIER].back();
                    connected_tiles[CURR_TIER].pop_back();
                    if (CURR_TIER < 3)
                    {
                      connected_tiles[CURR_TIER + 1].emplace_back(up_tile);
                      connected_tile_tier_map_it->second = CURR_TIER + 1;
                    }
                  }
                }
              }
            }
            if (down_tile.y <= y
                && explored_tiles_set.find(down_tile) == explored_tiles_set.end())
            {
              auto connected_tile_tier_map_it = connected_tile_tier_map.find(down_tile);
              if (connected_tile_tier_map_it == connected_tile_tier_map.end())
              {
                const int DOWN_TIER = get_tier(x, y, down_tile, explored_tiles_set) - 1;
                connected_tiles[DOWN_TIER].emplace_back(down_tile);
                connected_tile_tier_map.emplace(down_tile, DOWN_TIER);
              }
              else
              {
                const int CURR_TIER = connected_tile_tier_map_it->second;
                for (int k = 0; k < connected_tiles[CURR_TIER].size(); ++k)
                {
                  if (down_tile == connected_tiles[CURR_TIER][k])
                  {
                    connected_tiles[CURR_TIER][k] = connected_tiles[CURR_TIER].back();
                    connected_tiles[CURR_TIER].pop_back();
                    if (CURR_TIER < 3)
                    {
                      connected_tiles[CURR_TIER + 1].emplace_back(down_tile);
                      connected_tile_tier_map_it->second = CURR_TIER + 1;
                    }
                  }
                }
              }
            }
          }
        }
      }
    }
    else
    {
      int available_nodes = x * y;
      for (int i = 1; i <= n; ++i)
      {
        int j = rand() % available_nodes--;
        Coor selected_tile = tiles[j];
        coors.push_back(selected_tile);
        node_str += "init(object(node," + std::to_string(i) + "),value(at,(" + std::to_string(selected_tile.x) + ","
            + std::to_string(selected_tile.y) + "))).\n";

        tiles[j] = tiles.back();
        tiles.pop_back();
      }
    }

    int available_nodes = n;
    std::string robot_str;
    std::string shelf_str;
#ifdef ONLY_ROBOTS
    std::string goal_str;
#else
    std::string order_str;
    std::string product_str;
#endif

    for (int i = 1; i <= r; ++i)
    {
#ifndef ONLY_ROBOTS
      order_str += "init(object(order," + std::to_string(i) + "),value(line,(" + std::to_string(i) + ",1))).\n";
      product_str += "init(object(product," + std::to_string(i) + "),value(on,(" + std::to_string(i) + ",1))).\n";
#endif

      int j = rand() % available_nodes--;
      robot_str += "init(object(robot," + std::to_string(i) + "),value(at,(" + std::to_string(coors[j].x) + ","
          + std::to_string(coors[j].y) + "))).\n";

      // Swap the selected node with an unselected one, so that we only random among the unselected nodes
      Coor tmp = coors[j];
      coors[j] = coors[available_nodes];
      coors[available_nodes] = tmp;
    }

    available_nodes = n;
    for (int i = 1; i <= r; ++i)
    {
      int j = rand() % available_nodes--;
      // Shelves are used as goals in the visualizer
      shelf_str += "init(object(shelf," + std::to_string(i) + "),value(at,(" + std::to_string(coors[j].x) + "," +
                   std::to_string(coors[j].y) + "))).\n";

#ifdef ONLY_ROBOTS
      goal_str += "goal(robot(" + std::to_string(i) + "),(" + std::to_string(coors[j].x) + "," +
                   std::to_string(coors[j].y) + ")).\n";
#endif
      // Overwrite the selected node with an unselected one, so that we only random among the unselected nodes
      coors[j] = coors[available_nodes];
    }

    const std::string out_file = path + "/" + problem_lp;
#ifdef ONLY_ROBOTS
    if (write_file(out_file, seed_str + node_str + robot_str + shelf_str + goal_str))
    {
      std::cout << "Written " << out_file << std::endl;
    }
#else
    if (writeFile(out_file, seed_str + node_str + robot_str + order_str + product_str + shelf_str))
    {
      std::cout << "Written " << out_file << std::endl;
    }
#endif
    else
    {
      exit_failure("Cannot write a problem file " + out_file);
    }
  }

  /*
   * Divide the problem and write each domain file
   */
  if (!only_gen || only_div)
  {
    // Create a control object and pass command line arguments
    clingo_control_t* ctl = NULL;
    const char* clingo_argv[] = {(const char*)"--warn=none"};
    clingo_control_new(clingo_argv, 1, NULL, NULL, 0, &ctl);

    // Load the problem and the divider program
    const std::string src_path = project_path + "/src";
    if (!clingo_control_load(ctl, std::string(path + "/" + problem_lp).c_str()))
    {
      exit_failure("Cannot open " + path + "/" + problem_lp, ctl);
    }
    if (!clingo_control_load(ctl, std::string(src_path + "/divider.lp").c_str()))
    {
      exit_failure("Cannot open " + src_path + "/divider.lp", ctl);
    }

    // Ground the base part
    clingo_part_t part_base = {"base", NULL, 0};
    clingo_control_ground(ctl, &part_base, 1, NULL, NULL);

    // Get the nodes
    std::vector<std::pair<Coor, int>> coor_cluster_pairs;
    Coor min_coor, max_coor;
    {
      clingo_symbol_t sym;
      clingo_literal_t atom;
      clingo_symbol_create_function("show_nodes", NULL, 0, true, &sym);
      get_literal(ctl, sym, &atom);
      clingo_control_assign_external(ctl, atom, clingo_truth_value_true);

      std::string model;
      clingo_solve_result_bitset_t ret;
      if (solve(ctl, &ret, model))
      {
        if (ret & clingo_solve_result_satisfiable)
        {
          std::vector<std::string> coor_tokens(tokenizeString(model));
          coor_cluster_pairs.reserve(coor_tokens.size() / 2);

          if (coor_tokens.size() >= 2)
          {
            const int x = std::stoi(coor_tokens[0]);
            const int y = std::stoi(coor_tokens[1]);

            coor_cluster_pairs.emplace_back(Coor(x, y), -1);

            min_coor.x = max_coor.x = x;
            min_coor.y = max_coor.y = y;
          }
          else
          {
            exit_failure("There is no node in problem " + path + "/" + problem_lp, ctl);
          }

          for (int i = 2; i < coor_tokens.size(); i += 2)
          {
            const int x = std::stoi(coor_tokens[i]);
            const int y = std::stoi(coor_tokens[i + 1]);

            coor_cluster_pairs.emplace_back(Coor(x, y), -1);

            if (x < min_coor.x)
            {
              min_coor.x = x;
            }
            else if (x > max_coor.x)
            {
              max_coor.x = x;
            }

            if (y < min_coor.y)
            {
              min_coor.y = y;
            }
            else if (y > max_coor.y)
            {
              max_coor.y = y;
            }
          }
        }
        else
        {
          exit_failure("Unsat in getting nodes in problem instance " + path + "/" + problem_lp, ctl);
        }
      }
      else
      {
        exit_failure("Cannot get nodes in problem instance " + path + "/" + problem_lp, ctl);
      }

      clingo_control_release_external(ctl, atom);
    }

    // Divide the problem
    // XXX: Adjust the preferred domain size here
    const int PREFERRED_DX = 8;
    const int PREFERRED_DY = 8;
    const int PREFERRED_DOMAIN_SIZE = PREFERRED_DX * PREFERRED_DY;
    int num_domains = 0;
    std::chrono::steady_clock::time_point time_partitioning_start;

    switch (partitioner)
    {
      case Partitioner::BKMEANS:
      {
        const int num_nodes = coor_cluster_pairs.size();

        if (k == 0)
        {
          k = (int)ceil((double)num_nodes / PREFERRED_DOMAIN_SIZE);
        }

        if (k > num_nodes)
        {
          std::cerr << "The number of clusters must be less than or equal to the number of nodes" << std::endl;
          return EXIT_FAILURE;
        }

        // Start partitioning
        time_partitioning_start = std::chrono::steady_clock::now();

#if defined(DISTANCE_REAL) || defined(DISTANCE_REAL_CUSTOM)
        // Initialize the map for real distance search
        std::unordered_set<Coor, CoorHasher> coor_set;
        coor_set.reserve(coor_cluster_pairs.size());
        for (const auto &coor_cluster_pair : coor_cluster_pairs)
        {
          coor_set.emplace(coor_cluster_pair.first);
        }
#endif

        const int quotient = num_nodes / k;
        const int remainder = num_nodes % k;

        // Randomly pick centroids
        Centroid centroids[k];
        for (int i = 0; i < k; ++i)
        {
          centroids[i] = Centroid(((double)rand() / RAND_MAX * (max_coor.x - min_coor.x)) + min_coor.x,
                                  ((double)rand() / RAND_MAX * (max_coor.y - min_coor.y)) + min_coor.y);
        }

        int iteration = 0;
        while (true)
        {
          // Construct the cost matrix in column-major order for the Hungarian library to work correctly
          double* cost_matrix = new double[num_nodes * k];

          int cost_matrix_index = -1;
          for (int i = 0; i < k; ++i)
          {
#if defined(DISTANCE_REAL) || defined(DISTANCE_REAL_CUSTOM)
            Coor sink(std::round(centroids[i].x), std::round(centroids[i].y));
            if (coor_set.find(sink) == coor_set.end())
            {
              // The sink must not be an obstacle
              sink = find_nearest_coor(sink, min_coor, max_coor, coor_set);
            }
#endif

            for (const auto &coor_cluster_pair : coor_cluster_pairs)
            {
#if defined(DISTANCE_MANHATTAN)
              const double dist = std::fabs(coor_cluster_pair.first.x - centroids[i].x) +
                                  std::fabs(coor_cluster_pair.first.y - centroids[i].y);
#elif defined(DISTANCE_EUCLIDEAN)
              const double dist = (coor_cluster_pair.first.x - centroids[i].x) * (coor_cluster_pair.first.x - centroids[i].x) +
                                  (coor_cluster_pair.first.y - centroids[i].y) * (coor_cluster_pair.first.y - centroids[i].y);
#elif defined(DISTANCE_REAL) || defined(DISTANCE_REAL_CUSTOM)
              const double dist = find_real_distance(coor_cluster_pair.first, sink, coor_set);
#endif
              cost_matrix[++cost_matrix_index] = dist;
            }
          }

          // Solve the assignment problem
          std::unordered_map<int, int> index_map;   // Map the assigned index to the actual coor index
          index_map.reserve(num_nodes);
          for (int i = 0; i < num_nodes; ++i)
          {
            index_map.emplace(i, i);
          }
          int num_nodes_remained = num_nodes;
          bool changed = false;
          std::pair<Centroid, int> new_centroids[k]{};  // Pairs of new centroids and the number of nodes they contain

          for (int i = 1; i <= quotient; ++i)
          {
            double cost;
            int* assignment = new int[num_nodes_remained];
            HungarianAlgorithm::assignmentoptimal(assignment, &cost, cost_matrix, num_nodes_remained, k);

            // Get the assignment result
            std::vector<int> assigned;
            assigned.reserve(k);
            for (int i = 0; i < num_nodes_remained; ++i)
            {
              if (assignment[i] > -1)
              {
                assigned.push_back(i);

                const int nearest_cluster = assignment[i];
                const int coor_index = index_map.find(i)->second;

                if (nearest_cluster != coor_cluster_pairs[coor_index].second)
                {
                  coor_cluster_pairs[coor_index].second = nearest_cluster;
                  changed = true;
                }

                new_centroids[nearest_cluster].first.x += coor_cluster_pairs[coor_index].first.x;
                new_centroids[nearest_cluster].first.y += coor_cluster_pairs[coor_index].first.y;
                ++new_centroids[nearest_cluster].second;
              }
            }
            delete[] assignment;

            int num_original = num_nodes_remained;
            num_nodes_remained -= k;
            if (i < quotient || remainder)
            {
              // Remove the assigned columns from the cost matrix
              double* new_cost_matrix = new double[num_nodes_remained * k];

              int indent = 0;
              int assigned_index = 0;
              for (int i = 0; i < num_nodes_remained; ++i)
              {
                for (int j = i; assigned_index < k && assigned[assigned_index] == i + indent; ++j)
                {
                  ++assigned_index;
                  ++indent;
                }

                for (int j = 0; j < k; ++j)
                {
                  new_cost_matrix[j * num_nodes_remained + i] = cost_matrix[j * num_original + i + indent];
                  index_map.find(i)->second = index_map.find(i + indent)->second;
                }
              }

              delete[] cost_matrix;
              cost_matrix = new_cost_matrix;
            }
          }

          if (remainder)
          {
            // IMPORTANT: The algorithm works much faster if the number of row is greater than the number of columns
            // Transpose the matrix
            double* new_cost_matrix = new double[num_nodes_remained * k];
            cost_matrix_index = -1;
            for (int i = 0; i < num_nodes_remained; ++i)
            {
              for (int j = 0; j < k; ++j)
              {
                new_cost_matrix[++cost_matrix_index] = cost_matrix[j * num_nodes_remained + i];
              }
            }

            delete[] cost_matrix;
            cost_matrix = new_cost_matrix;

            // Solve the assignment problem
            double cost;
            int* assignment = new int[k];
            HungarianAlgorithm::assignmentoptimal(assignment, &cost, cost_matrix, k, num_nodes_remained);

            for (int i = 0; i < k; ++i)
            {
              if (assignment[i] > -1)
              {
                const int nearest_cluster = i;
                const int coor_index = index_map.find(assignment[i])->second;

                if (nearest_cluster != coor_cluster_pairs[coor_index].second)
                {
                  coor_cluster_pairs[coor_index].second = nearest_cluster;
                  changed = true;
                }

                new_centroids[nearest_cluster].first.x += coor_cluster_pairs[coor_index].first.x;
                new_centroids[nearest_cluster].first.y += coor_cluster_pairs[coor_index].first.y;
                ++new_centroids[nearest_cluster].second;
              }
            }
            delete[] assignment;
          }

          delete[] cost_matrix;

          if (changed && ++iteration <= MAX_KMEANS_ITERATION)   // Prevent nodes keep oscillating between centroids
          {
            std::cout << "Iteration " << iteration << " (max: " << MAX_KMEANS_ITERATION << ")" << std::endl;
            for (int i = 0; i < k; ++i)
            {
              if (new_centroids[i].second)
              {
                centroids[i].x = new_centroids[i].first.x / new_centroids[i].second;
                centroids[i].y = new_centroids[i].first.y / new_centroids[i].second;
              }
            }
          }
          else
          {
            if (iteration > MAX_KMEANS_ITERATION)
            {
              std::cout << "(max iteration reached)" << std::endl;
            }
            else
            {
              std::cout << "(done)" << std::endl;
            }

            // Rename clusters to avoid the empty ones
            int label = -1;
            std::unordered_map<int, int> cluster_label_map;
            cluster_label_map.reserve(k);
            for (auto &coor_cluster_pair : coor_cluster_pairs)
            {
              if (coor_cluster_pair.second < 0)
              {
                const std::chrono::milliseconds time_partitioning =
                    std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time_partitioning_start);

                std::cout << "Time = " << format_time(time_partitioning.count()) << std::endl;

                exit_failure("Node (" + std::to_string(coor_cluster_pair.first.x) + "," + std::to_string(coor_cluster_pair.first.y) +
                             ") has not been assigned to any sub-problem");
              }

              auto cluster_label_map_it = cluster_label_map.find(coor_cluster_pair.second);
              if (cluster_label_map_it == cluster_label_map.end())
              {
                cluster_label_map_it = cluster_label_map.emplace(coor_cluster_pair.second, ++label).first;
              }
              coor_cluster_pair.second = cluster_label_map_it->second;
            }
            num_domains = label + 1;
            break;
          }
        }
      }
        break;

      case Partitioner::BKMEANS_SQ:
      {
        const int num_nodes = coor_cluster_pairs.size();

        if (k == 0)
        {
          k = (int)ceil((double)num_nodes / PREFERRED_DOMAIN_SIZE);
        }

        if (k > num_nodes)
        {
          std::cerr << "The number of clusters must be less than or equal to the number of nodes" << std::endl;
          return EXIT_FAILURE;
        }

        // Start partitioning
        time_partitioning_start = std::chrono::steady_clock::now();

#if defined(DISTANCE_REAL) || defined(DISTANCE_REAL_CUSTOM)
        // Initialize the map for real distance search
        std::unordered_set<Coor, CoorHasher> coor_set;
        coor_set.reserve(coor_cluster_pairs.size());
        for (const auto &coor_cluster_pair : coor_cluster_pairs)
        {
          coor_set.emplace(coor_cluster_pair.first);
        }
#endif

        double* cost_matrix = new double[num_nodes * num_nodes];
        const int quotient = num_nodes / k;
        const int remainder = num_nodes % k;

        // Randomly pick centroids
        Centroid centroids[k];
        for (int i = 0; i < k; ++i)
        {
          centroids[i] = Centroid(((double)rand() / RAND_MAX * (max_coor.x - min_coor.x)) + min_coor.x,
                                  ((double)rand() / RAND_MAX * (max_coor.y - min_coor.y)) + min_coor.y);
        }

        int iteration = 0;
        while (true)
        {
#if defined(DISTANCE_REAL) || defined(DISTANCE_REAL_CUSTOM)
          std::vector<Coor> valid_centroid;
          valid_centroid.reserve(k);
          for (int i = 0; i < k; ++i)
          {
            Coor sink(std::round(centroids[i].x), std::round(centroids[i].y));
            if (coor_set.find(sink) == coor_set.end())
            {
              // The sink must not be an obstacle
              sink = find_nearest_coor(sink, min_coor, max_coor, coor_set);
            }
            valid_centroid.emplace_back(std::move(sink));
          }
#endif

          // Construct the cost matrix in column-major order for the Hungarian library to work correctly
          int cost_matrix_index = -1;
          for (const auto &coor_cluster_pair : coor_cluster_pairs)
          {
            for (int i = 0; i < k; ++i)
            {
#if defined(DISTANCE_MANHATTAN)
              const double dist = std::fabs(coor_cluster_pair.first.x - centroids[i].x) +
                                  std::fabs(coor_cluster_pair.first.y - centroids[i].y);
#elif defined(DISTANCE_EUCLIDEAN)
              const double dist = (coor_cluster_pair.first.x - centroids[i].x) * (coor_cluster_pair.first.x - centroids[i].x) +
                                  (coor_cluster_pair.first.y - centroids[i].y) * (coor_cluster_pair.first.y - centroids[i].y);
#elif defined(DISTANCE_REAL) || defined(DISTANCE_REAL_CUSTOM)
              const double dist = find_real_distance(coor_cluster_pair.first, valid_centroid[i], coor_set);
#endif
              for (int j = 0; j < quotient; ++j)
              {
                cost_matrix[++cost_matrix_index] = dist;
              }
              if (i >= (k - remainder))
              {
                cost_matrix[++cost_matrix_index] = dist;
              }
            }
          }

          bool changed = false;
          std::pair<Centroid, int> new_centroids[k]{};  // Pairs of new centroids and the number of nodes they contain

          double cost;
          int* assignment = new int[num_nodes];
          HungarianAlgorithm::assignmentoptimal(assignment, &cost, cost_matrix, num_nodes, num_nodes);
          for (int i = 0; i < num_nodes; ++i)
          {
            const int nearest_cluster = (i / quotient < k) ? i / quotient : i / quotient - 1;
            const int coor_index = assignment[i];

            if (nearest_cluster != coor_cluster_pairs[coor_index].second)
            {
              coor_cluster_pairs[coor_index].second = nearest_cluster;
              changed = true;
            }

            new_centroids[nearest_cluster].first.x += coor_cluster_pairs[coor_index].first.x;
            new_centroids[nearest_cluster].first.y += coor_cluster_pairs[coor_index].first.y;
            ++new_centroids[nearest_cluster].second;
          }
          delete[] assignment;

          if (changed && ++iteration <= MAX_KMEANS_ITERATION)   // Prevent nodes keep oscillating between centroids
          {
            std::cout << "Iteration " << iteration << " (max: " << MAX_KMEANS_ITERATION << ")" << std::endl;
            for (int i = 0; i < k; ++i)
            {
              if (new_centroids[i].second)
              {
                centroids[i].x = new_centroids[i].first.x / new_centroids[i].second;
                centroids[i].y = new_centroids[i].first.y / new_centroids[i].second;
              }
            }
          }
          else
          {
            delete[] cost_matrix;

            if (iteration > MAX_KMEANS_ITERATION)
            {
              std::cout << "(max iteration reached)" << std::endl;
            }
            else
            {
              std::cout << "(done)" << std::endl;
            }

            // Rename clusters to avoid the empty ones
            int label = -1;
            std::unordered_map<int, int> cluster_label_map;
            cluster_label_map.reserve(k);
            for (auto &coor_cluster_pair : coor_cluster_pairs)
            {
              if (coor_cluster_pair.second < 0)
              {
                const std::chrono::milliseconds time_partitioning =
                    std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time_partitioning_start);

                std::cout << "Time = " << format_time(time_partitioning.count()) << std::endl;

                exit_failure("Node (" + std::to_string(coor_cluster_pair.first.x) + "," + std::to_string(coor_cluster_pair.first.y) +
                             ") has not been assigned to any sub-problem");
              }

              auto cluster_label_map_it = cluster_label_map.find(coor_cluster_pair.second);
              if (cluster_label_map_it == cluster_label_map.end())
              {
                cluster_label_map_it = cluster_label_map.emplace(coor_cluster_pair.second, ++label).first;
              }
              coor_cluster_pair.second = cluster_label_map_it->second;
            }
            num_domains = label + 1;
            break;
          }
        }
      }
        break;

      case Partitioner::KMEANS:
      {
        const int num_nodes = coor_cluster_pairs.size();

        if (k == 0)
        {
          k = (int)ceil((double)num_nodes / PREFERRED_DOMAIN_SIZE);
        }

        if (k > num_nodes)
        {
          std::cerr << "The number of clusters must be less than or equal to the number of nodes" << std::endl;
          return EXIT_FAILURE;
        }

        // Start partitioning
        time_partitioning_start = std::chrono::steady_clock::now();

#if defined(DISTANCE_REAL) || defined(DISTANCE_REAL_CUSTOM)
        // Initialize the map for real distance search
        std::unordered_set<Coor, CoorHasher> coor_set;
        coor_set.reserve(coor_cluster_pairs.size());
        for (const auto &coor_cluster_pair : coor_cluster_pairs)
        {
          coor_set.emplace(coor_cluster_pair.first);
        }
#endif
        // Randomly pick centroids
        Centroid centroids[k];
        for (int i = 0; i < k; ++i)
        {
          centroids[i] = Centroid(((double)rand() / RAND_MAX * (max_coor.x - min_coor.x)) + min_coor.x,
                                  ((double)rand() / RAND_MAX * (max_coor.y - min_coor.y)) + min_coor.y);
        }

        int iteration = 0;
        while (true)
        {
#if defined(DISTANCE_REAL) || defined(DISTANCE_REAL_CUSTOM)
          std::vector<Coor> valid_centroid;
          valid_centroid.reserve(k);
          for (int i = 0; i < k; ++i)
          {
            Coor sink(std::round(centroids[i].x), std::round(centroids[i].y));
            if (coor_set.find(sink) == coor_set.end())
            {
              // The sink must not be an obstacle
              sink = find_nearest_coor(sink, min_coor, max_coor, coor_set);
            }
            valid_centroid.emplace_back(std::move(sink));
          }
#endif

          bool changed = false;
          std::pair<Centroid, int> new_centroids[k]{};  // Pairs of new centroids and the number of nodes they contain

          // Assign nodes to their nearest centroids
          for (auto &coor_cluster_pair : coor_cluster_pairs)
          {
            double min_dist = DBL_MAX;
            int nearest_cluster = 0;

            for (int i = 0; i < k; ++i)
            {
#if defined(DISTANCE_MANHATTAN)
              const double dist = std::fabs(coor_cluster_pair.first.x - centroids[i].x) +
                                  std::fabs(coor_cluster_pair.first.y - centroids[i].y);
#elif defined(DISTANCE_EUCLIDEAN)
              const double dist = (coor_cluster_pair.first.x - centroids[i].x) * (coor_cluster_pair.first.x - centroids[i].x) +
                                  (coor_cluster_pair.first.y - centroids[i].y) * (coor_cluster_pair.first.y - centroids[i].y);
#elif defined(DISTANCE_REAL) || defined(DISTANCE_REAL_CUSTOM)
              const double dist = find_real_distance(coor_cluster_pair.first, valid_centroid[i], coor_set);
#endif
              if (dist < min_dist)
              {
                min_dist = dist;
                nearest_cluster = i;
              }
            }

            if (nearest_cluster != coor_cluster_pair.second)
            {
              coor_cluster_pair.second = nearest_cluster;
              changed = true;
            }

            new_centroids[nearest_cluster].first.x += coor_cluster_pair.first.x;
            new_centroids[nearest_cluster].first.y += coor_cluster_pair.first.y;
            ++new_centroids[nearest_cluster].second;
          }

          if (changed && ++iteration <= MAX_KMEANS_ITERATION)   // Prevent nodes keep oscillating between centroids
          {
            std::cout << "Iteration " << iteration << " (max: " << MAX_KMEANS_ITERATION << ")" << std::endl;
            for (int i = 0; i < k; ++i)
            {
              if (new_centroids[i].second)
              {
                centroids[i].x = new_centroids[i].first.x / new_centroids[i].second;
                centroids[i].y = new_centroids[i].first.y / new_centroids[i].second;
              }
            }
          }
          else
          {
            if (iteration > MAX_KMEANS_ITERATION)
            {
              std::cout << "(max iteration reached)" << std::endl;
            }
            else
            {
              std::cout << "(done)" << std::endl;
            }

            // Rename clusters to avoid the empty ones
            int label = -1;
            std::unordered_map<int, int> cluster_label_map;
            cluster_label_map.reserve(k);
            for (auto& coor_cluster_pair : coor_cluster_pairs)
            {
              if (coor_cluster_pair.second < 0)
              {
                exit_failure("Node (" + std::to_string(coor_cluster_pair.first.x) + "," + std::to_string(coor_cluster_pair.first.y) +
                             ") has not been assigned to any sub-problem");
              }

              auto cluster_label_map_it = cluster_label_map.find(coor_cluster_pair.second);
              if (cluster_label_map_it == cluster_label_map.end())
              {
                cluster_label_map_it = cluster_label_map.emplace(coor_cluster_pair.second, ++label).first;
              }
              coor_cluster_pair.second = cluster_label_map_it->second;
            }
            num_domains = label + 1;
            break;
          }
        }
      }
        break;

      case Partitioner::NAIVE:
      {
        if ((dx | dy) == 0)
        {
          dx = PREFERRED_DX;
          dy = PREFERRED_DY;
        }
        else if (dx == 0)
        {
          dx = (int)ceil((double)PREFERRED_DOMAIN_SIZE / dy);
        }
        else if (dy == 0)
        {
          dy = (int)ceil((double)PREFERRED_DOMAIN_SIZE / dx);
        }

#ifdef VISUALIZE
        // Need to clear the list of node first because node-cluster pairs will be added later
        coor_cluster_pairs.clear();
#endif

        // Start partitioning
        time_partitioning_start = std::chrono::steady_clock::now();

        for (int row = min_coor.y; row <= max_coor.y; row += dy)
        {
          for (int col = min_coor.x; col <= max_coor.x; col += dx)
          {
            clingo_symbol_t sym[5];
            clingo_symbol_create_number(++num_domains, &sym[0]);
            clingo_symbol_create_number(col, &sym[1]);
            clingo_symbol_create_number(col + dx, &sym[2]);
            clingo_symbol_create_number(row, &sym[3]);
            clingo_symbol_create_number(row + dy, &sym[4]);

            clingo_part_t part_divide = {"divide", sym, sizeof(sym) / sizeof(*sym)};
            clingo_control_ground(ctl, &part_divide, 1, NULL, NULL);

            // Check whether there exist a node in this solver
            clingo_literal_t atom;
            clingo_symbol_create_function("show_exist", sym, 1, true, &sym[1]);
            get_literal(ctl, sym[1], &atom);
            clingo_control_assign_external(ctl, atom, clingo_truth_value_true);

            std::string model;
            clingo_solve_result_bitset_t ret;
            if (solve(ctl, &ret, model))
            {
              if (ret & clingo_solve_result_satisfiable)
              {
                // Discard this solver if it does not exist any node
                if (model.empty())
                {
                  --num_domains;
                  clingo_control_assign_external(ctl, atom, clingo_truth_value_false);
                }
                else
                {
                  clingo_control_release_external(ctl, atom);
#ifdef VISUALIZE
                  std::vector<std::string> coor_tokens(tokenizeString(model));
                  for (int i = 0; i < coor_tokens.size(); i += 2)
                  {
                    coor_cluster_pairs.emplace_back(Coor(std::stoi(coor_tokens[i]), std::stoi(coor_tokens[i + 1])),
                                                    num_domains - 1);
                  }
#endif
                }
              }
              else
              {
                exit_failure("Unsat in checking whether there is a node in domain " + std::to_string(num_domains), ctl);
              }
            }
            else
            {
              exit_failure("Cannot check whether there is a node in domain " + std::to_string(num_domains), ctl);
            }
          }
        }
      }
        break;

      default:
        exit_failure("Unsupported partitioning algorithm " + std::to_string((int)partitioner), ctl);
    }

    // Merge small areas into big ones
    switch (partitioner)
    {
      case Partitioner::BKMEANS:
      case Partitioner::KMEANS:
      case Partitioner::NAIVE:
      {
        const int num_nodes = coor_cluster_pairs.size();
        const int min_nodes_required = (partitioner == Partitioner::NAIVE) ? dx * dy * MIN_PERCENT_NODES : (double)num_nodes / k * MIN_PERCENT_NODES;

        // Create a map to aid in searching
        std::unordered_map<Coor, int, CoorHasher> coor_cluster_map;
        for (const auto& coor_cluster_pair : coor_cluster_pairs)
        {
          coor_cluster_map.emplace(coor_cluster_pair.first, coor_cluster_pair.second);
        }

        bool has_merged = false;
        std::unordered_set<Coor, CoorHasher> checked_coor_set;
        std::unordered_set<int> valid_cluster_set;
        int i = 0;
        while (i < num_nodes)
        {
          const Coor& current_coor = coor_cluster_pairs[i].first;
          if (checked_coor_set.find(current_coor) != checked_coor_set.end())
          {
            ++i;
          }
          else
          {
            const int current_cluster = coor_cluster_map.find(current_coor)->second;
            std::unordered_map<int, int> cluster_count_map;
            std::unordered_set<Coor, CoorHasher> inside_coor_set;
            std::unordered_set<Coor, CoorHasher> visited_coor_set{current_coor};
            std::queue<Coor> queue;
            queue.push(current_coor);
            while (!queue.empty())
            {
              const Coor coor(queue.front());
              queue.pop();
              inside_coor_set.emplace(coor);

              // Generate child nodes
              for (int i = 0; i < 4; ++i)
              {
                Coor new_coor;
                switch(i)
                {
                  case 0:   // Go left
                    new_coor = Coor(coor.x - 1, coor.y);
                    break;

                  case 1:   // Go right
                    new_coor = Coor(coor.x + 1, coor.y);
                    break;

                  case 2:   // Go up
                    new_coor = Coor(coor.x, coor.y - 1);
                    break;

                  case 3:   // Go down
                    new_coor = Coor(coor.x, coor.y + 1);
                    break;
                }

                const auto coor_cluster_map_it = coor_cluster_map.find(new_coor);
                if (coor_cluster_map_it != coor_cluster_map.end())  // make sure it is a valid node
                {
                  if (visited_coor_set.find(new_coor) == visited_coor_set.end())
                  {
                    visited_coor_set.emplace(new_coor);

                    // the new coor is in the same cluster
                    if (current_cluster == coor_cluster_map_it->second)
                    {
                      queue.push(new_coor);
                    }
                    else  // the new coor is not in the same cluster
                    {
                      auto cluster_count_map_it = cluster_count_map.find(coor_cluster_map_it->second);
                      if (cluster_count_map_it == cluster_count_map.end())
                      {
                        cluster_count_map.emplace(coor_cluster_map_it->second, 1);
                      }
                      else
                      {
                        ++cluster_count_map_it->second;
                      }
                    }
                  }
                }
              }
            }

            if (inside_coor_set.size() >= min_nodes_required)
            {
              valid_cluster_set.insert(current_cluster);
              checked_coor_set.insert(inside_coor_set.begin(), inside_coor_set.end());
              ++i;
            }
            else
            {
              int cluster;
              int count = 0;
              for (const auto& cluster_count_pair : cluster_count_map)
              {
                if (count < cluster_count_pair.second)
                {
                  count = cluster_count_pair.second;
                  cluster = cluster_count_pair.first;
                }
              }

              if (count > 0)
              {
                for (const auto& inside_coor : inside_coor_set)
                {
                  coor_cluster_map.find(inside_coor)->second = cluster;
                }
                has_merged = true;
              }
              else  // If there is no neighboring cluster to merge, we consider it valid
              {
                valid_cluster_set.insert(current_cluster);
                checked_coor_set.insert(inside_coor_set.begin(), inside_coor_set.end());
                ++i;
              }
            }
          }
        }

        // Need to re-label if any area has been merged
        if (has_merged)
        {
          coor_cluster_pairs.clear();

          int label = -1;
          std::unordered_map<int, int> cluster_label_map;
          cluster_label_map.reserve(valid_cluster_set.size());
          for (auto& coor_cluster_pair : coor_cluster_map)
          {
            auto cluster_label_map_it = cluster_label_map.find(coor_cluster_pair.second);
            if (cluster_label_map_it == cluster_label_map.end())
            {
              cluster_label_map_it = cluster_label_map.emplace(coor_cluster_pair.second, ++label).first;
            }
            coor_cluster_pairs.emplace_back(coor_cluster_pair.first, cluster_label_map_it->second);
          }
          num_domains = label + 1;
        }
      }
        break;
    }


    const std::chrono::milliseconds time_partitioning =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time_partitioning_start);

    // Add the divided nodes
    {
      std::string node_string;
      for (const auto& coor_cluster_pair : coor_cluster_pairs)
      {
        node_string += "node(" + std::to_string(coor_cluster_pair.second + 1) + ",("
            + std::to_string(coor_cluster_pair.first.x) + "," + std::to_string(coor_cluster_pair.first.y) + ")).";
      }

      clingo_part_t part = {"node", NULL, 0};
      clingo_control_add(ctl, "node", NULL, 0, node_string.c_str());
      clingo_control_ground(ctl, &part, 1, NULL, NULL);
    }

    // Determine areas
    std::string area_string;
    std::vector<std::pair<int, int>> area_capacity;   // [(num_goals, num_nodes)]
    for (int domain = 1; domain <= num_domains; ++domain)
    {
      clingo_symbol_t sym[2];
      clingo_literal_t atom;
      clingo_symbol_create_number(domain, &sym[0]);

      clingo_part_t part = {"part", sym, 1};
      clingo_control_ground(ctl, &part, 1, NULL, NULL);

      clingo_symbol_create_function("show_edges", sym, 1, true, &sym[1]);
      get_literal(ctl, sym[1], &atom);
      clingo_control_assign_external(ctl, atom, clingo_truth_value_true);

      std::string model;
      clingo_solve_result_bitset_t ret;
      if (solve(ctl, &ret, model))
      {
        if (ret & clingo_solve_result_satisfiable)
        {
          std::unordered_map<Coor, std::unordered_set<Coor, CoorHasher>*, CoorHasher> edge_set_map;

          std::vector<std::string> edge_tokens(tokenizeString(model));
          edge_set_map.reserve(edge_tokens.size() / 4);
          for (int i = 0; i < edge_tokens.size(); i += 4)
          {
            Coor c1(std::stoi(edge_tokens[i]), std::stoi(edge_tokens[i + 1]));
            Coor c2(std::stoi(edge_tokens[i + 2]), std::stoi(edge_tokens[i + 3]));
            auto c1_it = edge_set_map.find(c1);
            auto c2_it = edge_set_map.find(c2);

            if (c1_it == edge_set_map.end())
            {
              if (c2_it == edge_set_map.end())
              {
                std::unordered_set<Coor, CoorHasher>* coor_set = new std::unordered_set<Coor, CoorHasher>{c1, c2};
                edge_set_map.emplace(c1, coor_set);
                edge_set_map.emplace(c2, coor_set);
              }
              else
              {
                c2_it->second->emplace(c1);
                edge_set_map.emplace(c1, c2_it->second);
              }
            }
            else if (c2_it == edge_set_map.end())
            {
              c1_it->second->emplace(c2);
              edge_set_map.emplace(c2, c1_it->second);
            }
            else if (c1_it->second != c2_it->second)  // Merge different node sets that has an edge between them
            {
              std::unordered_set<Coor, CoorHasher>* merged_set;
              std::unordered_set<Coor, CoorHasher>* remain_set;
              if (c1_it->second->size() < c2_it->second->size())
              {
                merged_set = c1_it->second;
                remain_set = c2_it->second;
              }
              else
              {
                merged_set = c2_it->second;
                remain_set = c1_it->second;
              }

              for (const Coor& coor : *merged_set)
              {
                remain_set->emplace(coor);
                edge_set_map.find(coor)->second = remain_set;
              }
              delete merged_set;
            }
          }

          std::unordered_set<std::unordered_set<Coor, CoorHasher>*> checked_coor_set;
          for (const auto& edge_set_pair : edge_set_map)
          {
            if (checked_coor_set.find(edge_set_pair.second) == checked_coor_set.end())
            {
              area_capacity.emplace_back(0, edge_set_pair.second->size());
              const int area = area_capacity.size();

              for (const auto& coor : *edge_set_pair.second)
              {
                area_string += "area(" + std::to_string(domain) + "," + std::to_string(area) + ",(" +
                               std::to_string(coor.x) + "," + std::to_string(coor.y) + ")).";
              }
              checked_coor_set.emplace(edge_set_pair.second);
              delete edge_set_pair.second;
            }
          }
        }
        else
        {
          exit_failure("Unsat in getting edges in domain " + std::to_string(domain), ctl);
        }
      }
      else
      {
        exit_failure("Cannot get edges in domain " + std::to_string(domain), ctl);
      }
      clingo_control_release_external(ctl, atom);
    }

    // Add the areas
    {
      clingo_part_t part = {"area", NULL, 0};
      clingo_control_add(ctl, "area", NULL, 0, area_string.c_str());
      clingo_control_ground(ctl, &part, 1, NULL, NULL);
    }

    // Ground the link and result parts
    {
      clingo_part_t parts[] = {{"link", NULL, 0}, {"result", NULL, 0}};
      clingo_control_ground(ctl, parts, 2, NULL, NULL);
    }

    // Create a domain file for each solver
    {
      if (num_solvers <= 0 || num_solvers > num_domains)
      {
        num_solvers = num_domains;
      }

      for (int solver = 1; solver <= num_solvers; ++solver)
      {
        int num_areas = 0;
        std::string area_info_str;
        std::string lp_str;
        std::vector<std::pair<int, std::vector<RobotInfo>>> area_robot_infos_pairs;

        for (int domain = solver; domain <= num_domains; domain += num_solvers)
        {
          // Show areas
          std::vector<AreaInfo> area_infos;
          {
            clingo_symbol_t sym[2];
            clingo_literal_t atom;
            clingo_symbol_create_number(domain, &sym[0]);
            clingo_symbol_create_function("show_areas", sym, 1, true, &sym[1]);
            get_literal(ctl, sym[1], &atom);
            clingo_control_assign_external(ctl, atom, clingo_truth_value_true);

            std::string model;
            clingo_solve_result_bitset_t ret;
            if (solve(ctl, &ret, model))
            {
              if (ret & clingo_solve_result_satisfiable)
              {
                std::vector<std::string> tokens(tokenizeString(model));
                area_infos.reserve(tokens.size() / 6);
                for (int i = 0; i < tokens.size(); i += 6)
                {
                  area_infos.emplace_back(AreaInfo{std::stoi(tokens[i]),
                                                   std::stoi(tokens[i + 1]),
                                                   Coor{std::stoi(tokens[i + 2]), std::stoi(tokens[i + 3])},
                                                   Coor{std::stoi(tokens[i + 4]), std::stoi(tokens[i + 5])},
                                                   std::vector<Connection>{}});
                }
              }
              else
              {
                exit_failure("Unsat in getting areas in domain " + std::to_string(domain), ctl);
              }
            }
            else
            {
              exit_failure("Cannot get areas in domain " + std::to_string(domain), ctl);
            }
            clingo_control_release_external(ctl, atom);
          }

          // Show connections
          std::vector<std::vector<std::string>> area_links;
          for (auto& area_info : area_infos)
          {
            area_links.emplace_back(std::vector<std::string>{});

            clingo_symbol_t sym[4];
            clingo_literal_t atom;
            clingo_symbol_create_number(area_info.area, &sym[0]);
            clingo_symbol_create_function("show_connections", sym, 1, true, &sym[2]);
            get_literal(ctl, sym[2], &atom);
            clingo_control_assign_external(ctl, atom, clingo_truth_value_true);

            std::string model;
            clingo_solve_result_bitset_t ret;
            if (solve(ctl, &ret, model))
            {
              if (ret & clingo_solve_result_satisfiable)
              {
                clingo_control_release_external(ctl, atom);   // Release show_connections external

                std::vector<std::string> tokens(tokenizeString(model));
                for (int i = 0; i < tokens.size(); i += 2)
                {
                  const int connected_area = std::stoi(tokens[i]);
                  clingo_symbol_create_number(connected_area, &sym[1]);

                  // Show corners
                  std::vector<Coor> corners;
                  {
                    clingo_literal_t atom;
                    clingo_symbol_create_function("show_corners", sym, 2, true, &sym[3]);
                    get_literal(ctl, sym[3], &atom);
                    clingo_control_assign_external(ctl, atom, clingo_truth_value_true);

                    std::string model;
                    clingo_solve_result_bitset_t ret;
                    if (solve(ctl, &ret, model))
                    {
                      if (ret & clingo_solve_result_satisfiable)
                      {
                        std::vector<std::string> tokens(tokenizeString(model));
                        corners.reserve(tokens.size() / 2);
                        for (int i = 0; i < tokens.size(); i += 2)
                        {
                          corners.emplace_back(Coor(std::stoi(tokens[i]), std::stoi(tokens[i + 1])));
                        }
                      }
                      else
                      {
                        exit_failure("Unsat in getting corners between areas " + std::to_string(area_info.area) + " and " + std::to_string(connected_area), ctl);
                      }
                    }
                    else
                    {
                      exit_failure("Cannot get corners between areas " + std::to_string(area_info.area) + " and " + std::to_string(connected_area), ctl);
                    }
                    clingo_control_release_external(ctl, atom);
                  }

                  // Show links
                  int num_links = 0;
                  {
                    clingo_symbol_create_function("show_links", sym, 2, true, &sym[2]);
                    get_literal(ctl, sym[2], &atom);
                    clingo_control_assign_external(ctl, atom, clingo_truth_value_true);

                    std::string model;
                    clingo_solve_result_bitset_t ret;
                    if (solve(ctl, &ret, model, "."))
                    {
                      if (ret & clingo_solve_result_satisfiable)
                      {
                        std::unordered_set<Coor, CoorHasher> linked_coors_set;
                        std::vector<std::string> tokens(tokenizeString(model, "l(,)."));
                        for (int i = 0; i < tokens.size(); i += 4)
                        {
                          const Coor coor_from(std::stoi(tokens[i]), std::stoi(tokens[i + 1]));
                          const Coor coor_to(std::stoi(tokens[i + 2]), std::stoi(tokens[i + 3]));

                          // Do not short-circuit here because we want to insert both nodes
                          if (linked_coors_set.find(coor_from) == linked_coors_set.end())
                          {
                            linked_coors_set.emplace(coor_from);
                            if (linked_coors_set.find(coor_to) == linked_coors_set.end())
                            {
                              linked_coors_set.emplace(coor_to);
                              ++num_links;
                            }
                          }
                          else if (linked_coors_set.find(coor_to) == linked_coors_set.end())
                          {
                            linked_coors_set.emplace(coor_to);
                          }
                        }

                        area_links.back().emplace_back(std::move(model));
                      }
                      else
                      {
                        exit_failure("Unsat in getting links from area " + std::to_string(area_info.area) + " to area " + std::to_string(connected_area), ctl);
                      }
                    }
                    else
                    {
                      exit_failure("Cannot get link from area " + std::to_string(area_info.area) + " to area " + std::to_string(connected_area), ctl);
                    }
                    clingo_control_release_external(ctl, atom);
                  }

                  area_info.connections.emplace_back(Connection{connected_area,
                                                                (std::stoi(tokens[i + 1]) - 1) % num_solvers + 1,
                                                                num_links,
                                                                std::move(corners)});
                }
              }
              else
              {
                exit_failure("Unsat in getting connections in area " + std::to_string(area_info.area), ctl);
              }
            }
            else
            {
              exit_failure("Cannot get connections in area " + std::to_string(area_info.area), ctl);
            }
          }

          // Show robots
          std::unordered_map<int, std::vector<RobotInfo>> area_robot_infos_map;
          {
            clingo_symbol_t sym[2];
            clingo_literal_t atom;
            clingo_symbol_create_number(domain, &sym[0]);
            clingo_symbol_create_function("show_robots", sym, 1, true, &sym[1]);
            get_literal(ctl, sym[1], &atom);
            clingo_control_assign_external(ctl, atom, clingo_truth_value_true);

            std::string model;
            clingo_solve_result_bitset_t ret;
            if (solve(ctl, &ret, model))
            {
              if (ret & clingo_solve_result_satisfiable)
              {
                std::vector<std::string> tokens(tokenizeString(model));
                for (int i = 0; i < tokens.size(); i += 7)
                {
                  const int area_from = std::stoi(tokens[i + 1]);
                  auto area_robot_infos_map_it = area_robot_infos_map.find(area_from);
                  if (area_robot_infos_map_it == area_robot_infos_map.end())
                  {
                    area_robot_infos_map_it = area_robot_infos_map.emplace(area_from, std::vector<RobotInfo>{}).first;
                    area_robot_infos_map_it->second.reserve(tokens.size() / 7);
                  }

                  const int area_to = std::stoi(tokens[i + 2]);
                  area_robot_infos_map_it->second.emplace_back(RobotInfo{std::stoi(tokens[i]), area_to,
                                                                         Coor(std::stoi(tokens[i + 3]), std::stoi(tokens[i + 4])),
                                                                         Coor(std::stoi(tokens[i + 5]), std::stoi(tokens[i + 6]))});

                  ++area_capacity[area_to - 1].first;
                }
              }
              else
              {
                exit_failure("Unsat in getting robots in domain " + std::to_string(domain), ctl);
              }
            }
            else
            {
              exit_failure("Cannot get robots in domain " + std::to_string(domain), ctl);
            }
            clingo_control_release_external(ctl, atom);
          }

          // Write the header of the content for the domain file
          num_areas += area_infos.size();
          for (const auto& area_info : area_infos)
          {
            area_info_str += area_info.str() + "\n";
          }

          for (const auto& area_robot_infos_pair : area_robot_infos_map)
          {
            area_robot_infos_pairs.emplace_back(area_robot_infos_pair.first, area_robot_infos_pair.second);
          }

          int area_link_index = -1;
          for (const auto& area_info : area_infos)
          {
            // Show nodes
            {
              clingo_symbol_t sym[2];
              clingo_literal_t atom;
              clingo_symbol_create_number(area_info.area, &sym[0]);
              clingo_symbol_create_function("show_nodes", sym, 1, true, &sym[1]);
              get_literal(ctl, sym[1], &atom);
              clingo_control_assign_external(ctl, atom, clingo_truth_value_true);

              std::string model;
              clingo_solve_result_bitset_t ret;
              if (solve(ctl, &ret, model, "."))
              {
                if (ret & clingo_solve_result_satisfiable)
                {
                  lp_str += "%\n" + model + "\n";
                }
                else
                {
                  exit_failure("Unsat in getting nodes in area " + std::to_string(area_info.area), ctl);
                }
              }
              else
              {
                exit_failure("Cannot get nodes in area " + std::to_string(area_info.area), ctl);
              }
              clingo_control_release_external(ctl, atom);
            }

            // Show nexttos
            {
              clingo_symbol_t sym[2];
              clingo_literal_t atom;
              clingo_symbol_create_number(area_info.area, &sym[0]);
              clingo_symbol_create_function("show_nexttos", sym, 1, true, &sym[1]);
              get_literal(ctl, sym[1], &atom);
              clingo_control_assign_external(ctl, atom, clingo_truth_value_true);

              std::string model;
              clingo_solve_result_bitset_t ret;
              if (solve(ctl, &ret, model, "."))
              {
                if (ret & clingo_solve_result_satisfiable)
                {
                  if (model.empty())  // Prevent reading error in the solver
                  {
                    lp_str += "%\n-\n";
                  }
                  else
                  {
                    lp_str += "%\n" + model + "\n";
                  }
                }
                else
                {
                  exit_failure("Unsat in getting nexttos in area " + std::to_string(area_info.area), ctl);
                }
              }
              else
              {
                exit_failure("Cannot get nexttos in area " + std::to_string(area_info.area), ctl);
              }
              clingo_control_release_external(ctl, atom);
            }

            // Add links to the content of this area
            ++area_link_index;
            for (const std::string& links : area_links[area_link_index])
            {
              lp_str += "%\n" + links + "\n";
            }
          }
        }

        std::string robot_info_str;
        for (const auto& area_robot_infos_pair : area_robot_infos_pairs)
        {
          robot_info_str += std::to_string(area_robot_infos_pair.first) + " " + std::to_string(area_robot_infos_pair.second.size()) + "\n";
          for (const auto& robot_info : area_robot_infos_pair.second)
          {
            robot_info_str += robot_info.str() + "\n";
          }
        }

        // Write the domain file of the solver
        const std::string out_file = path + "/d" + std::to_string(solver) + ".lp";
        const std::string content(
            std::to_string(num_areas) + "\n" +
            area_info_str +
            std::to_string(area_robot_infos_pairs.size()) + "\n" +
            robot_info_str +
            lp_str
        );
        if (write_file(out_file, content))
        {
          std::cout << "Written " << out_file << std::endl;
        }
        else
        {
          exit_failure("Cannot write a domain file " + out_file, ctl);
        }
      }
    }

    // Create an abstract links file
    {
      clingo_symbol_t sym;
      clingo_literal_t atom;
      clingo_symbol_create_function("show_abstract_links", NULL, 0, true, &sym);
      get_literal(ctl, sym, &atom);
      clingo_control_assign_external(ctl, atom, clingo_truth_value_true);

      std::string model;
      clingo_solve_result_bitset_t ret;
      if (solve(ctl, &ret, model, "."))
      {
        if (ret & clingo_solve_result_satisfiable)
        {
          // Write the abstract links file
          const std::string out_file = path + "/links.lp";
          if (write_file(out_file, model + "\n"))
          {
            std::cout << "Written " << out_file << std::endl;
          }
          else
          {
            exit_failure("Cannot write a links file " + out_file, ctl);
          }
        }
        else
        {
          exit_failure("Unsat in solving for abstract links", ctl);
        }
      }
      else
      {
        exit_failure("Cannot solve for abstract links", ctl);
      }
      clingo_control_release_external(ctl, atom);
    }

    // Free the Clingo control object
    clingo_control_free(ctl);

    // Write a launch file and job scripts
    {
      std::string namespace_base(launch_file);
      std::replace(namespace_base.begin(), namespace_base.end(), '-', '_');   // Having '-' in namespace causes an error
      const std::string namespace_single(namespace_base + "_s");

      std::string launch_content(
        "<?xml version=\"1.0\"?>\n"
        "\n"
        "<launch>\n"
        "  <arg name=\"path\" value=\"$(find dmapf)\"/>\n"
        "  <group ns=\"" + namespace_single + "\">\n"
        "    <param name=\"problem\"  value=\"$(arg path)/examples/" + domain_dir + "/" + problem_lp + "\"/>\n"
        "    <param name=\"answer\"   value=\"$(arg path)/examples/" + domain_dir + "/answer.lp\"/>\n"
        "    <param name=\"links\"    value=\"$(arg path)/examples/" + domain_dir + "/links.lp\"/>\n"
        "    <param name=\"areas\"    value=\"" + std::to_string(area_capacity.size()) + "\"/>\n"
        "    <param name=\"solvers\"  value=\"" + std::to_string(num_solvers) + "\"/>\n"
        "\n"
      );

      for (int solver = 1; solver <= num_solvers; ++solver)
      {
        launch_content +=
          "    <node pkg=\"dmapf\" type=\"solver\" name=\"s" + std::to_string(solver) + "\" output=\"screen\" required=\"true\">\n"
          "      <param name=\"domain\" value=\"$(arg path)/examples/" + domain_dir + "/d" + std::to_string(solver) + ".lp\"/>\n"
          "    </node>\n";
      }

      launch_content +=
        "  </group>\n"
        "</launch>\n";

      {
        const std::string out_file = path + "/" + launch_file + "-seed" + std::to_string(seed) + ".launch";
        if (write_file(out_file, launch_content))
        {
          std::cout << "Written " << out_file << std::endl;
        }
        else
        {
          exit_failure("Cannot write a launch file " + out_file);
        }
      }

      const std::string job_single(
        "#!/bin/bash\n"
        "\n"
        "#SBATCH --job-name=" + namespace_single + "\n"
        "#SBATCH --output=single-%j.out\n"
        "#SBATCH --ntasks=1\n"
        "#SBATCH --cpus-per-task=" + std::to_string((num_solvers * 2 > MAX_CPUS ? MAX_CPUS : num_solvers * 2)) + "\n"
        "#SBATCH --mem-per-cpu=1G\n"
        "#SBATCH --partition=normal\n"
        "#SBATCH --time=0-01:00:00\n"
        "\n"
        "module load conda\n"
        "conda activate ros-noetic\n"
        "source $HOME/catkin_ws/devel/setup.bash\n"
        "\n"
        "t1=`date +%s.%N`\n"
        "srun --cpus-per-task=$SLURM_CPUS_PER_TASK roslaunch dmapf " + launch_file + ".launch\n"
        "t2=`date +%s.%N`\n"
        "echo \"real    $(echo \"$t2-$t1\" | bc -l)s\"\n"
      );

      {
        const std::string out_file = path + "/job-single.sh";
        if (write_file(out_file, job_single))
        {
          std::cout << "Written " << out_file << std::endl;
        }
        else
        {
          exit_failure("Cannot write a job script " + out_file);
        }
      }

      const std::string namespace_multi(namespace_base + "_m");
      const std::string job_multi(
        "#!/bin/bash\n"
        "\n"
        "#SBATCH --job-name=" + namespace_multi + "\n"
        "#SBATCH --output=multi-%j.out\n"
        "#SBATCH --ntasks=" + std::to_string(num_solvers) + "\n"
        "#SBATCH --cpus-per-task=2\n"
        "#SBATCH --mem-per-cpu=1G\n"
        "#SBATCH --partition=normal\n"
        "#SBATCH --time=0-01:00:00\n"
        "\n"
        "if [[ -z $ROS_MASTER_URI ]]; then\n"
        "  echo \"ROS_MASTER_URI has not been set\" >&2\n"
        "  exit 1\n"
        "fi\n"
        "\n"
        "module load conda\n"
        "conda activate ros-noetic\n"
        "source $HOME/catkin_ws/devel/setup.bash\n"
        "export ROS_MASTER_URI=$ROS_MASTER_URI\n"
        "\n"
        "path=$(rospack find dmapf)\n"
        "if [[ -z $path ]]; then\n"
        "  echo \"Cannot find project dmapf\" >&2\n"
        "  exit 2\n"
        "fi\n"
        "rosparam set " + namespace_multi + "/problem $path/examples/" + domain_dir + "/" + problem_lp + "\n"
        "rosparam set " + namespace_multi + "/answer $path/examples/" + domain_dir + "/answer.lp\n"
        "rosparam set " + namespace_multi + "/links $path/examples/" + domain_dir + "/links.lp\n"
        "rosparam set " + namespace_multi + "/areas " + std::to_string(area_capacity.size()) + "\n"
        "rosparam set " + namespace_multi + "/solvers " + std::to_string(num_solvers) + "\n"
        "\n"
        "t1=`date +%s.%N`\n"
        "for i in {1.." + std::to_string(num_solvers) + "}; do\n"
        "  srun --nodes=1 --ntasks=1 --cpus-per-task=$SLURM_CPUS_PER_TASK rosrun dmapf solver __ns:=" + namespace_multi +
        " __name:=s$i _domain:=$path/examples/" + domain_dir + "/d$i.lp &\n"
        "done\n"
        "wait\n"
        "t2=`date +%s.%N`\n"
        "echo \"real    $(echo \"$t2-$t1\" | bc -l)s\"\n"
      );

      {
        const std::string out_file = path + "/job-multi.sh";
        if (write_file(out_file, job_multi))
        {
          std::cout << "Written " << out_file << std::endl;
        }
        else
        {
          exit_failure("Cannot write a job script " + out_file);
        }
      }
    }

#ifdef VISUALIZE
    {
      /* Print the partitioned map */
      print_line();

      // Build the palette
      const std::string colors[] = {WHITE, RED, GREEN, YELLOW, CYAN, BLUE, MAGENTA};
      std::string palette[num_domains];
      char letter = 'A';
      int color_index = -1;
      for (int i = 0; i < num_domains; ++i)
      {
        palette[i] = colors[++color_index] + (char)(letter) + RESET;
        if (color_index >= (sizeof(colors) / sizeof(colors[0])) - 1)
        {
          color_index = -1;
          ++letter;

          if (letter == 'Z' + 1)
          {
            letter = 'a';
          }
          else if (letter == 'z' + 1)
          {
            letter = 'A';
          }
        }
      }

      // Build the visualization
      const int range_x = max_coor.x - min_coor.x + 1;
      const int range_y = max_coor.y - min_coor.y + 1;
      std::string map[range_x * range_y];
      for (const auto& coor_cluster_pair : coor_cluster_pairs)
      {
        const int index = (coor_cluster_pair.first.y - min_coor.y) * range_x + (coor_cluster_pair.first.x - min_coor.x);
        map[index] = palette[coor_cluster_pair.second];

        if (index < 0 || index >= range_x * range_y)
        {
          std::cout << "(" << coor_cluster_pair.first.x << "," << coor_cluster_pair.first.y << ") -> " << index << std::endl;
        }
      }

      // Print
      int map_index = 0;
      for (int y = min_coor.y; y <= max_coor.y; ++y)
      {
        for (int x = min_coor.x; x <= max_coor.x; ++x, ++map_index)
        {
          if (map[map_index].empty())
          {
            std::cout << BLACK << "#" << RESET;
          }
          else
          {
            std::cout << map[map_index];
          }
        }
        std::cout << std::endl;
      }
    }
#endif

    /* Print the statistics */
    std::string stat_string;
    std::string warn_string;
    switch (partitioner)
    {
      case Partitioner::BKMEANS:
        stat_string += "Using balanced k-means partitioning with k = " + std::to_string(k) + ", seed = " + std::to_string(seed) + "\n";
        break;

      case Partitioner::BKMEANS_SQ:
        stat_string += "Using balanced k-means (square matrix) partitioning with k = " + std::to_string(k) + ", seed = " + std::to_string(seed) + "\n";
        break;

      case Partitioner::KMEANS:
        stat_string += "Using k-means partitioning with k = " + std::to_string(k) + ", seed = " + std::to_string(seed) + "\n";
        break;

      case Partitioner::NAIVE:
        stat_string += "Using naive partitioning with dx = " + std::to_string(dx) + ", dy = " + std::to_string(dy) + "\n";
        break;
    }

    {
      const double avg_num_nodes = (double)coor_cluster_pairs.size() / area_capacity.size();
      double var_num_nodes = 0.0;
      int min_num_nodes = area_capacity[0].second;
      int max_num_nodes = area_capacity[0].second;

      for (int i = 0; i < area_capacity.size(); ++i)
      {
        const int num_goals = area_capacity[i].first;
        const int num_nodes = area_capacity[i].second;
        var_num_nodes += (num_nodes - avg_num_nodes) * (num_nodes - avg_num_nodes);

        if (num_nodes < min_num_nodes)
        {
          min_num_nodes = num_nodes;
        }
        else if (num_nodes > max_num_nodes)
        {
          max_num_nodes = num_nodes;
        }

        if (num_nodes - num_goals <= MIN_NUM_FREE_NODES)
        {
          warn_string += "a" + std::to_string(i + 1) + ": " + std::to_string(num_goals) + " / "+ std::to_string(num_nodes) + "\n";
        }
      }

      const double sd = std::sqrt(var_num_nodes / area_capacity.size());

      stat_string += "Time     : " + format_time(time_partitioning.count()) + "\n" +
                     "#nodes   : " + std::to_string(coor_cluster_pairs.size()) + "\n" +
                     "#areas   : " + std::to_string(area_capacity.size()) + "\n" +
                     "#domains : " + std::to_string(num_domains) + "\n" +
                     "#solvers : " + std::to_string(num_solvers) + "\n" +
                     "#nodes/#area\n" +
                     "- avg    : " + std::to_string(avg_num_nodes) + "\n" +
                     "- SD     : " + std::to_string(sd) + " -> " + std::to_string(sd / avg_num_nodes * 100.0) + " %\n" +
                     "- max    : " + std::to_string(max_num_nodes) + "\n" +
                     "- min    : " + std::to_string(min_num_nodes) + "\n" +
                     "- range  : " + std::to_string(max_num_nodes - min_num_nodes) + "\n";
    }

    print_line();
    std::cout << stat_string;

    if (!warn_string.empty())
    {
      print_line();
      std::cout << "area: #goals / #nodes" << std::endl << warn_string;
    }
  }

  return EXIT_SUCCESS;
}
