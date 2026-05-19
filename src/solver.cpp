#include <algorithm>
#include <atomic>
#include <chrono>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>

#include <ros/ros.h>
#include <ros/package.h>
#include <dmapf/Abstract.h>
#include <dmapf/Aggregate.h>
#include <dmapf/Migrate.h>
#include <dmapf/Track.h>

#include "dmapf/clingo_wrap.h"
#include "dmapf/color.h"
#include "dmapf/utility.h"

#undef DEBUG_VERBOSE_1
#undef DEBUG_VERBOSE_2
#undef DEBUG_VERBOSE_3
#undef DEBUG_TIMES
#undef DEBUG_ABSTRACT   // The program will terminate right after abstract plans are made
#undef DEBUG_ABSTRACT_VERBOSE

// Exactly one of these macros must be defined
#undef ABSTRACT_ASP
#undef ABSTRACT_ASP_ITERABLE    // Only for legacy purposes
#undef ABSTRACT_BFS
#undef ABSTRACT_BFS_RANDOM
#undef ABSTRACT_RANDOM
#undef ABSTRACT_UCS
#define ABSTRACT_UCS_CENTRAL

#define MAX_HORIZON_ABSTRACT_OFFSET 0   // Additive constant
#define MIN_NUM_FREE_NODES 2
#define TIMEOUT_MAX 60.0
#define RATE 200

// These macros are defined automatically by CMake. They should all be commented in production.
// Define only one of them at a time for Eclipse's indexer to work correctly.
//#define SOLVER_ASP
//#undef SOLVER_CBSH2_RTC
//#undef SOLVER_EECBS
//#undef SOLVER_PBS

#define MIN_SOLVING_TIME_PER_ROBOT 0.01
#define PER_ROBOT_FACTOR 1.25
#define TIMEOUT_PENALTY_FACTOR 2.0
#define TIMEOUT_TOLERANCE_FACTOR 10.0

#ifdef SOLVER_CBSH2_RTC
#include "CBSH2-RTC/inc/CBS.h"
#define RUNS 1
#endif

#ifdef SOLVER_EECBS
#include "EECBS/inc/ECBS.h"
#define RUNS 1
#endif

#ifdef SOLVER_PBS
#include "PBS/inc/PBS.h"
#endif

const char* g_clingo_argv[] = {"--warn=none"};

int g_total_areas;
int g_solver_id;
std::string g_solver_name;

template<typename T>
  void retrieve(const std::string& key, T& value)
  {
    if (!ros::param::get(key, value))
    {
      std::cerr << RED << g_solver_name << " cannot retrieve " << key << RESET << std::endl;
      std::exit(EXIT_FAILURE);
    }
  }

#if defined(ABSTRACT_ASP) || defined(ABSTRACT_ASP_ITERABLE)
struct AbstractPlan
{
  clingo_control_t* ctl;
  clingo_literal_t query_atom;
  int round;
  bool grounded;
  int num_constraints;
  int shortest_plan_length;   // To be used with MAX_HORIZON_ABSTRACT_OFFSET
};
#else
struct AbstractNode
{
  int area;
  int parent;
};

#if defined(ABSTRACT_UCS) || defined(ABSTRACT_UCS_CENTRAL)
struct AbstractRank
{
  double congestion;
  int step;
  int index;

  friend bool operator< (const AbstractRank& lhs, const AbstractRank& rhs)
  {
    return lhs.congestion > rhs.congestion;
  }
};
#endif
#endif

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

  bool is_move() const
  {
    return x != 0 || y != 0;
  }

  void add(const Coor &coor)
  {
    x += coor.x;
    y += coor.y;
  }
};

struct CoorHasher
{
  std::size_t operator() (const Coor& key) const
  {
    return ((std::size_t)key.x << (sizeof(std::size_t) << 2)) | key.y;
  }
};

struct Connection
{
  int solver;
  int num_links;
  std::string link_lp;
};

struct AreaInfo
{
  int num_nodes;
#ifdef SOLVER_ASP
  std::string node_lp;
  std::string nextto_lp;
#else
  Coor min_node;
  Coor max_node;
  Instance instance;
#endif
  double solving_time_per_robot;
  std::unordered_map<int, Connection> neighbor_connection_map;
  std::unordered_map<Coor, std::vector<int>, CoorHasher> corner_neighbors_map;
};

struct Block
{
  int num_out_blocked;
  int num_in_blocked;
  std::string constraint;
};

struct Cross
{
  int solver_from;
  int area_from;
  int solver_to;
  int area_to;
};

struct Transit
{
  Coor coor_from;               // A starting location in the current area
  Coor coor_goal;               // The final goal location
  Coor direction;               // A direction migrated from the previous area
  std::vector<int> next_areas;  // A sequence of areas to reach the final area, in reverse order
};

struct Move
{
  Coor direction;
  int timestep;
};

struct Migrate
{
  int robot;
  Coor border_from;
  Coor border_to;
  int rank;

  static bool compare_descend(const Migrate& lhs, const Migrate& rhs)
  {
    return lhs.rank > rhs.rank;
  }
};

// To be used with area_border_immigrate_map
struct Immigrate
{
  int robot;
  int rank;

  static bool compare_ascend(const Immigrate& lhs, const Immigrate& rhs)
  {
    return lhs.rank < rhs.rank;
  }
};

// To be used with area_border_emigrate_map
struct Emigrate
{
  int robot;
  Coor border_to;
  int rank;
};

struct Area
{
  AreaInfo& area_info;
  std::unordered_map<int, Block> neighbor_block_map;

  std::unordered_map<int, Transit> robot_transit_map;
  // A list of robots that need to migrate grouped by neighbor area and ranked by the number of abstract steps remaining
  std::unordered_map<int, std::map<int, std::vector<int>, std::greater<int>>> neighbor_tier_robots_map;

  /* The variables below will be recalculated every time the area is worked on */
  std::unordered_set<int> out_robots;  // A set of robots that have successfully migrated
  std::vector<std::pair<int, Transit>> in_robot_transit_pairs;  // A list of robots coming in and their information
  std::unordered_map<int, std::pair<std::vector<Move>, Coor>> robot_moves_last_pair_map; // A sequence of moves for the robots in robot_transit_map

  Area(AreaInfo& area_info) : area_info(area_info) {}

  void add_robot(const int robot, const Transit& transit)
  {
    robot_transit_map.emplace(robot, transit);

    // If the robot needs to further migrate, rank it based on the number of abstract steps remaining
    if (!transit.next_areas.empty())
    {
      const int next_area = transit.next_areas.back();

      auto neighbor_tier_robots_map_it = neighbor_tier_robots_map.find(next_area);
      if (neighbor_tier_robots_map_it == neighbor_tier_robots_map.end())
      {
        neighbor_tier_robots_map_it = neighbor_tier_robots_map.emplace(next_area, std::map<int, std::vector<int>, std::greater<int>>{}).first;
      }

      auto ranked_robots_out_map_it = neighbor_tier_robots_map_it->second.find(transit.next_areas.size());
      if (ranked_robots_out_map_it == neighbor_tier_robots_map_it->second.end())
      {
        ranked_robots_out_map_it = neighbor_tier_robots_map_it->second.emplace(transit.next_areas.size(), std::vector<int>{}).first;
      }

      ranked_robots_out_map_it->second.push_back(robot);
    }
  }
};

void initialize_area(Area& area)
{
  // Set up connectivity with neighboring areas
  for (const auto& neighbor_connection_pair : area.area_info.neighbor_connection_map)
  {
    area.neighbor_block_map.emplace(neighbor_connection_pair.first, Block{});
  }
}

void create_child_area(Area& area, const Area& prev)
{
  initialize_area(area);

  // Add previous robots that did not migrate
  for (const auto& prev_robot_transit_pair : prev.robot_transit_map)
  {
    if (prev.out_robots.find(prev_robot_transit_pair.first) == prev.out_robots.end())
    {
      area.add_robot(prev_robot_transit_pair.first, prev_robot_transit_pair.second);

      // Reset the migrating direction to indicate that this robot did not just move in
      Coor& direction = area.robot_transit_map.find(prev_robot_transit_pair.first)->second.direction;
      direction.x = 0;
      direction.y = 0;
    }
  }

  // Add previous incoming robots
  for (const auto& prev_in_robot_transit_pair : prev.in_robot_transit_pairs)
  {
    area.add_robot(prev_in_robot_transit_pair.first, prev_in_robot_transit_pair.second);
  }

  // Update the current location of existing robots to be their last location in the previous round
  for (auto& robot_transit_pair : area.robot_transit_map)
  {
    const auto prev_robot_moves_last_pair_map_it = prev.robot_moves_last_pair_map.find(robot_transit_pair.first);
    if (prev_robot_moves_last_pair_map_it != prev.robot_moves_last_pair_map.end())
    {
      robot_transit_pair.second.coor_from = prev_robot_moves_last_pair_map_it->second.second;
    }
  }
}

struct Task
{
  std::unordered_map<int, std::unordered_map<int, int>> send_areas_map;   // area -> [(neighbor_area, neighbor_solver)]
  std::unordered_map<int, std::unordered_set<int>> recv_areas_map;        // area -> [neighbor]
  int num_recv_areas;

  Task() : num_recv_areas(0){}

  void add_task(const Cross& cross)
  {
    if (g_solver_id > cross.solver_from)  // This solver was the receiver of the cross message
    {
      auto recv_areas_map_it = recv_areas_map.find(cross.area_to);
      if (recv_areas_map_it == recv_areas_map.end())
      {
        recv_areas_map_it = recv_areas_map.emplace(cross.area_to, std::unordered_set<int>{}).first;
      }
      if (recv_areas_map_it->second.find(cross.area_from) == recv_areas_map_it->second.end())
      {
        recv_areas_map_it->second.insert(cross.area_from);
        ++num_recv_areas;
      }
    }
    else if (g_solver_id < cross.solver_from)   // This solver was the receiver of the cross message
    {
      auto send_areas_map_it = send_areas_map.find(cross.area_to);
      if (send_areas_map_it == send_areas_map.end())
      {
        send_areas_map_it = send_areas_map.emplace(cross.area_to, std::unordered_map<int, int>{}).first;
      }
      if (send_areas_map_it->second.find(cross.area_from) == send_areas_map_it->second.end())
      {
        send_areas_map_it->second.emplace(cross.area_from, cross.solver_from);
      }
    }
    else   // This solver was the sender of the cross message
    {
      if (g_solver_id > cross.solver_to)
      {
        auto recv_areas_map_it = recv_areas_map.find(cross.area_from);
        if (recv_areas_map_it == recv_areas_map.end())
        {
          recv_areas_map_it = recv_areas_map.emplace(cross.area_from, std::unordered_set<int>{}).first;
        }
        if (recv_areas_map_it->second.find(cross.area_to) == recv_areas_map_it->second.end())
        {
          recv_areas_map_it->second.insert(cross.area_to);
          ++num_recv_areas;
        }
      }
      else if (g_solver_id < cross.solver_to)
      {
        auto send_areas_map_it = send_areas_map.find(cross.area_from);
        if (send_areas_map_it == send_areas_map.end())
        {
          send_areas_map_it = send_areas_map.emplace(cross.area_from, std::unordered_map<int, int>{}).first;
        }
        if (send_areas_map_it->second.find(cross.area_to) == send_areas_map_it->second.end())
        {
          send_areas_map_it->second.emplace(cross.area_to, cross.solver_to);
        }
      }
      else
      {
        if (cross.area_from > cross.area_to)
        {
          auto recv_areas_map_it = recv_areas_map.find(cross.area_from);
          if (recv_areas_map_it == recv_areas_map.end())
          {
            recv_areas_map_it = recv_areas_map.emplace(cross.area_from, std::unordered_set<int>{}).first;
          }
          if (recv_areas_map_it->second.find(cross.area_to) == recv_areas_map_it->second.end())
          {
            recv_areas_map_it->second.insert(cross.area_to);
            ++num_recv_areas;
          }

          auto send_areas_map_it = send_areas_map.find(cross.area_to);
          if (send_areas_map_it == send_areas_map.end())
          {
            send_areas_map_it = send_areas_map.emplace(cross.area_to, std::unordered_map<int, int>{}).first;
          }
          if (send_areas_map_it->second.find(cross.area_from) == send_areas_map_it->second.end())
          {
            send_areas_map_it->second.emplace(cross.area_from, cross.solver_from);
          }
        }
        else
        {
          auto recv_areas_map_it = recv_areas_map.find(cross.area_to);
          if (recv_areas_map_it == recv_areas_map.end())
          {
            recv_areas_map_it = recv_areas_map.emplace(cross.area_to, std::unordered_set<int>{}).first;
          }
          if (recv_areas_map_it->second.find(cross.area_from) == recv_areas_map_it->second.end())
          {
            recv_areas_map_it->second.insert(cross.area_from);
            ++num_recv_areas;
          }

          auto send_areas_map_it = send_areas_map.find(cross.area_from);
          if (send_areas_map_it == send_areas_map.end())
          {
            send_areas_map_it = send_areas_map.emplace(cross.area_from, std::unordered_map<int, int>{}).first;
          }
          if (send_areas_map_it->second.find(cross.area_to) == send_areas_map_it->second.end())
          {
            send_areas_map_it->second.emplace(cross.area_to, cross.solver_to);
          }
        }
      }
    }
  }

  void reset()
  {
    send_areas_map.clear();
    recv_areas_map.clear();
    num_recv_areas = 0;
  }
};

std::unordered_map<int, AreaInfo> g_area_info_map;    // Hold information of the areas
int g_current_round = 0;

// Globally-tracked variables
std::atomic<int> g_num_track_msg_expected{0};
std::atomic<int> g_num_track_msg_ignored_1{0};
std::atomic<int> g_num_track_msg_ignored_2{0};
std::unordered_set<int> g_active_solvers_set_volatile;
std::unordered_set<int> g_active_solvers_set;
std::vector<Cross> g_crosses_volatile;
std::vector<Cross> g_crosses;

#ifdef ABSTRACT_UCS
std::atomic<int> g_num_areas_msg_expected{0};
#endif

// Between-solvers variables
std::unordered_map<int, std::unordered_map<int, Area>> g_round_areas_map;
std::atomic<int> g_num_migrate_negotiate_req_received{0};
std::atomic<int> g_num_migrate_reject_req_received{0};
std::atomic<int> g_num_migrate_confirm_req_received{0};
std::atomic<int> g_num_aggregate_req_received{0};

#if defined(DEBUG_ABSTRACT) || defined(ABSTRACT_UCS_CENTRAL)
std::atomic<int> g_num_abstract_req_received{0};
#endif

#if defined(ABSTRACT_ASP) || defined(ABSTRACT_ASP_ITERABLE)
std::unordered_map<Coor, AbstractPlan, CoorHasher> g_area_pair_abstract_plan_map;
#endif

void ros_exit()
{
#if defined(ABSTRACT_ASP) || defined(ABSTRACT_ASP_ITERABLE)
  for (const auto& area_pair_abstract_plan_pair : g_area_pair_abstract_plan_map)
  {
    clingo_control_free(area_pair_abstract_plan_pair.second.ctl);
  }
#endif
  ros::shutdown();
}

void wait_for_subscribers(const ros::Publisher& pub, const int num_subs, ros::Rate rate = RATE)
{
  while (ros::ok() && pub.getNumSubscribers() < num_subs)
  {
    rate.sleep();
  }
}

#if defined(ABSTRACT_ASP) || defined(ABSTRACT_ASP_ITERABLE)
bool make_abstract_plan(AbstractPlan& abstract_plan, std::vector<int>& next_areas)
{
  // Solve for an abstract plan
  const int max_horizon_abstract = (abstract_plan.shortest_plan_length == -1)? g_total_areas: abstract_plan.shortest_plan_length + MAX_HORIZON_ABSTRACT_OFFSET;
  while (abstract_plan.round <= max_horizon_abstract)
  {
    // Ground the abstract planning program
    if (!abstract_plan.grounded)
    {
      clingo_symbol_t sym[2];
      clingo_part_t part_plan = {"p", sym, 1};
      clingo_symbol_create_number(abstract_plan.round, &sym[0]);
      clingo_control_ground(abstract_plan.ctl, &part_plan, 1, NULL, NULL);

      clingo_symbol_create_function("q", sym, 1, true, &sym[1]);
      get_literal(abstract_plan.ctl, sym[1], &abstract_plan.query_atom);
      clingo_control_assign_external(abstract_plan.ctl, abstract_plan.query_atom, clingo_truth_value_true);

      abstract_plan.grounded = true;
    }

    // Solve
    std::string model;
    clingo_solve_result_bitset_t ret;
    if (solve(abstract_plan.ctl, &ret, model))
    {
      if (ret & clingo_solve_result_satisfiable)
      {
        std::vector<std::string> tokens(tokenizeString(model, "r(,)"));

        // Extend the vector to hold next areas
        next_areas.resize(abstract_plan.round);

        // Set the shortest plan length if this is the first time making abstract plan
        if (abstract_plan.shortest_plan_length == -1)
        {
          abstract_plan.shortest_plan_length = abstract_plan.round;
        }

        // Put the next solvers in reverse order
#ifdef ABSTRACT_ASP_ITERABLE
        std::string constraint(":-");
#endif
        for (int i = 0; i < tokens.size(); i += 2)
        {
          const int round = std::stoi(tokens[i + 1]);
          if (round > 0)
          {
            const int area = std::stoi(tokens[i]);
            next_areas[abstract_plan.round - round] = area;
          }
#ifdef ABSTRACT_ASP_ITERABLE
          constraint += "r(" + tokens[i] + "," + tokens[i + 1] + "),";
#endif
        }

#ifdef ABSTRACT_ASP_ITERABLE
        constraint += "q(" + std::to_string(abstract_plan.round) + ").";

        // Add the constraint
        std::string name("c" + std::to_string(++abstract_plan.num_constraints));
        clingo_part_t part = {name.c_str(), NULL, 0};
        clingo_control_add(abstract_plan.ctl, name.c_str(), NULL, 0, constraint.c_str());
        clingo_control_ground(abstract_plan.ctl, &part, 1, NULL, NULL);

#ifdef DEBUG_VERBOSE_3
        std::cout << "Added constraint " << name << " " << constraint << std::endl;
#endif
#endif
        return true;
      }
      else
      {
        clingo_control_release_external(abstract_plan.ctl, abstract_plan.query_atom);
        ++abstract_plan.round;
        abstract_plan.grounded = false;
      }
    }
    else
    {
      return false;
    }
  }

  return false;
}
#endif

std::string g_migrate_lp_content;
std::unordered_map<int, std::vector<Migrate>> g_area_emigrates_map;     // Area -> [(robot, from, to, rank)]
std::unordered_map<int, std::vector<Migrate>> g_area_immigrates_map;    // Area -> [(robot, from, to, rank)]
std::unordered_map<int, std::unordered_set<int>> g_area_canceled_robots_map;
std::unordered_map<int, std::vector<int>> g_area_canceled_robots_map_req;
std::unordered_map<int, std::unordered_set<int>> g_area_rejected_robots_map;
std::unordered_map<int, std::vector<int>> g_area_rejected_robots_map_req;
std::unordered_map<int, std::vector<std::pair<int, Transit>>> g_area_confirmed_transits_map;
std::unordered_map<int, std::unordered_map<int, std::vector<std::pair<int, Transit>>>> g_area_neighbor_confirmed_transits_map;
bool migrate_service(dmapf::Migrate::Request& req, dmapf::Migrate::Response& res)
{
  switch (req.type)
  {
    case 0:
    {
      const int sent_from = req.area_from;
      const int sent_to = req.area_to;

      Area& recv_area = g_round_areas_map.find(g_current_round)->second.find(sent_to)->second;
      const AreaInfo& recv_area_info = recv_area.area_info;
      const Block& block = recv_area.neighbor_block_map.find(sent_from)->second;
      const Connection& connection = recv_area_info.neighbor_connection_map.find(sent_from)->second;
      const int num_borders_in_available = connection.num_links - block.num_in_blocked;
      const int num_borders_out_available = connection.num_links - block.num_out_blocked;

      // Merge two sets of robots tier-by-tier while there are available borders
      int num_robots_in = 0;
      int num_robots_out = 0;
      int num_limits;
      std::string abcd;
      std::unordered_map<int, int> robot_rank_map;  // Only used to store the ranks of incoming robots
      const auto neighbor_tier_robots_map_it = recv_area.neighbor_tier_robots_map.find(sent_from);
      if (neighbor_tier_robots_map_it == recv_area.neighbor_tier_robots_map.end())  // There is no robot to send to the requester
      {
        if (num_borders_in_available > 0)
        {
          int tier_index = 0;
          while (tier_index < req.tiers.size())
          {
            const dmapf::Tier& tier = req.tiers[tier_index];
            num_robots_in += tier.positions.size();
            if (num_robots_in <= num_borders_in_available)
            {
              for (const auto& position : tier.positions)
              {
                abcd += "c(" + std::to_string(position.robot) + ",(" + std::to_string(position.coor.x) + "," + std::to_string(position.coor.y) + ")).";
                robot_rank_map.emplace(position.robot, tier.rank);
              }
              if (num_robots_in == num_borders_in_available)
              {
                break;
              }
            }
            else
            {
              for (const auto& position : tier.positions)
              {
                abcd += "d(" + std::to_string(position.robot) + ",(" + std::to_string(position.coor.x) + "," + std::to_string(position.coor.y) + ")).";
                robot_rank_map.emplace(position.robot, tier.rank);
              }
              break;
            }
            ++tier_index;
          }
          num_limits = std::min(num_borders_in_available, num_robots_in);
        }
        else
        {
          num_limits = 0;
        }
      }
      else
      {
        bool out_full = false;
        bool in_full = false;
        int tier_index = 0;
        auto tier_robots_map_it = neighbor_tier_robots_map_it->second.begin();

        // Prioritize robots with longer abstract steps left
        while (tier_robots_map_it != neighbor_tier_robots_map_it->second.end() && tier_index < req.tiers.size())
        {
          const dmapf::Tier& tier = req.tiers[tier_index];

          if (tier_robots_map_it->first > tier.rank)
          {
            const int num_borders_available = (num_borders_out_available >= num_borders_in_available)?
                                              num_borders_out_available - num_robots_in:
                                              (std::min(num_borders_out_available, num_borders_in_available - num_robots_in));
            if (num_robots_out < num_borders_available)
            {
              num_robots_out += tier_robots_map_it->second.size();
              if (num_robots_out <= num_borders_available)
              {
                for (const int robot : tier_robots_map_it->second)
                {
                  const Coor& coor = recv_area.robot_transit_map.find(robot)->second.coor_from;
                  abcd += "a(" + std::to_string(robot) + ",(" + std::to_string(coor.x) + "," + std::to_string(coor.y) + ")).";
                }
                if (num_robots_out == num_borders_available)
                {
                  out_full = true;
                  break;
                }
              }
              else
              {
                for (const int robot : tier_robots_map_it->second)
                {
                  const Coor& coor = recv_area.robot_transit_map.find(robot)->second.coor_from;
                  abcd += "b(" + std::to_string(robot) + ",(" + std::to_string(coor.x) + "," + std::to_string(coor.y) + ")).";
                }
                out_full = true;
                break;
              }
              ++tier_robots_map_it;
            }
            else
            {
              out_full = true;
              break;
            }
          }
          else
          {
            const int num_borders_available = (num_borders_in_available >= num_borders_out_available)?
                                              num_borders_in_available - num_robots_out:
                                              (std::min(num_borders_in_available, num_borders_out_available - num_robots_out));
            if (num_robots_in < num_borders_available)
            {
              num_robots_in += tier.positions.size();
              if (num_robots_in <= num_borders_available)
              {
                for (const auto& position : tier.positions)
                {
                  abcd += "c(" + std::to_string(position.robot) + ",(" + std::to_string(position.coor.x) + "," + std::to_string(position.coor.y) + ")).";
                  robot_rank_map.emplace(position.robot, tier.rank);
                }
                if (num_robots_in == num_borders_available)
                {
                  in_full = true;
                  break;
                }
              }
              else
              {
                for (const auto& position : tier.positions)
                {
                  abcd += "d(" + std::to_string(position.robot) + ",(" + std::to_string(position.coor.x) + "," + std::to_string(position.coor.y) + ")).";
                  robot_rank_map.emplace(position.robot, tier.rank);
                }
                in_full = true;
                break;
              }
              ++tier_index;
            }
            else
            {
              in_full = true;
              break;
            }
          }
        }

        // Add remaining outgoing robots if there is available border to go out
        if (!out_full)
        {
          const int num_borders_available = (num_borders_out_available >= num_borders_in_available)?
                                            num_borders_out_available - num_robots_in:
                                            (std::min(num_borders_out_available, num_borders_in_available - num_robots_in));
          if (num_robots_out < num_borders_available)
          {
            while (tier_robots_map_it != neighbor_tier_robots_map_it->second.end())
            {
              num_robots_out += tier_robots_map_it->second.size();
              if (num_robots_out <= num_borders_available)
              {
                for (const int robot : tier_robots_map_it->second)
                {
                  const Coor& coor = recv_area.robot_transit_map.find(robot)->second.coor_from;
                  abcd += "a(" + std::to_string(robot) + ",(" + std::to_string(coor.x) + "," + std::to_string(coor.y) + ")).";
                }
                if (num_robots_out == num_borders_available)
                {
                  break;
                }
              }
              else
              {
                for (const int robot : tier_robots_map_it->second)
                {
                  const Coor& coor = recv_area.robot_transit_map.find(robot)->second.coor_from;
                  abcd += "b(" + std::to_string(robot) + ",(" + std::to_string(coor.x) + "," + std::to_string(coor.y) + ")).";
                }
                break;
              }
              ++tier_robots_map_it;
            }
          }
        }

        // Add remaining incoming robots if there is available border to come in
        if (!in_full)
        {
          const int num_borders_available = (num_borders_in_available >= num_borders_out_available)?
                                            num_borders_in_available - num_robots_out:
                                            (std::min(num_borders_in_available, num_borders_out_available - num_robots_out));
          if (num_robots_in < num_borders_available)
          {
            while (tier_index < req.tiers.size())
            {
              const dmapf::Tier& tier = req.tiers[tier_index];
              num_robots_in += tier.positions.size();
              if (num_robots_in <= num_borders_available)
              {
                for (const auto& position : tier.positions)
                {
                  abcd += "c(" + std::to_string(position.robot) + ",(" + std::to_string(position.coor.x) + "," + std::to_string(position.coor.y) + ")).";
                  robot_rank_map.emplace(position.robot, tier.rank);
                }
                if (num_robots_in == num_borders_available)
                {
                  break;
                }
              }
              else
              {
                for (const auto& position : tier.positions)
                {
                  abcd += "d(" + std::to_string(position.robot) + ",(" + std::to_string(position.coor.x) + "," + std::to_string(position.coor.y) + ")).";
                  robot_rank_map.emplace(position.robot, tier.rank);
                }
                break;
              }
              ++tier_index;
            }
          }
        }

        num_limits = std::min(std::min(num_robots_in, num_borders_in_available) + std::min(num_robots_out, num_borders_out_available),
                              std::max(num_borders_in_available, num_borders_out_available));
      }

#ifdef DEBUG_VERBOSE_2
      {
        const int solver_from = recv_area_info.neighbor_connection_map.find(sent_from)->second.solver;
        std::cout << g_solver_name << "-a" << sent_to << " receives a " << CYAN << "NEG" << RESET
                  << " request from s" << solver_from << "-a" << sent_from << ": "
                  << abcd << "l(" << num_limits << ")." << block.constraint << connection.link_lp << std::endl;
      }
#endif

      // Create a control object to make border assignment between the areas
      clingo_control_t* ctl;
      clingo_control_new(g_clingo_argv, sizeof(g_clingo_argv) / sizeof(*g_clingo_argv), NULL, NULL, 0, &ctl);
      clingo_control_add(ctl, "base", NULL, 0, abcd.c_str());
      clingo_control_add(ctl, "base", NULL, 0, std::string("l(" + std::to_string(num_limits) + ").").c_str());
      clingo_control_add(ctl, "base", NULL, 0, block.constraint.c_str());
      clingo_control_add(ctl, "base", NULL, 0, connection.link_lp.c_str());
      clingo_control_add(ctl, "base", NULL, 0, g_migrate_lp_content.c_str());

      clingo_part_t part_base = {"base", NULL, 0};
      clingo_control_ground(ctl, &part_base, 1, NULL, NULL);

      // Solve
      std::string model;
      clingo_solve_result_bitset_t ret;
      if (solve(ctl, &ret, model))
      {
        if (ret & clingo_solve_result_satisfiable)
        {
#ifdef DEBUG_VERBOSE_2
          std::cout << CYAN << g_solver_name << "-a" << sent_to << " assigns " << model << RESET << std::endl;
#endif

          auto area_emigrates_map_it = g_area_emigrates_map.find(sent_to);
          if (g_area_emigrates_map.find(sent_to) == g_area_emigrates_map.end())
          {
            area_emigrates_map_it = g_area_emigrates_map.emplace(sent_to, std::vector<Migrate>{}).first;
          }
          std::vector<Migrate>& emigrates = area_emigrates_map_it->second;

          auto area_immigrates_map_it = g_area_immigrates_map.find(sent_to);
          if (area_immigrates_map_it == g_area_immigrates_map.end())
          {
            area_immigrates_map_it = g_area_immigrates_map.emplace(sent_to, std::vector<Migrate>{}).first;
          }
          std::vector<Migrate>& immigrates = area_immigrates_map_it->second;

          res.area_from = sent_to;
          res.area_to = sent_from;

          std::vector<std::string> tokens(tokenizeString(model, "m(,)"));
          for (int i = 0; i < tokens.size(); i += 6)
          {
            res.dists.emplace_back(dmapf::Dist{});
            dmapf::Dist& dist = res.dists.back();
            dist.robot = std::stoi(tokens[i]);
            const auto robot_transit_map_it = recv_area.robot_transit_map.find(dist.robot);

            // If it's an existing robot, then it's going out
            if (robot_transit_map_it != recv_area.robot_transit_map.end())
            {
              // used by the receiver when it needs to decide which robots to reject
              dist.rank = robot_transit_map_it->second.next_areas.size();

              // used by the receiver to calculate the migrating direction (was not used originally)
              const Coor border_from(std::stoi(tokens[i + 1]), std::stoi(tokens[i + 2]));
              dist.coor_from.x = border_from.x;
              dist.coor_from.y = border_from.y;

              // used by the receiver to be the node where the robot moves into
              const Coor border_to(std::stoi(tokens[i + 3]), std::stoi(tokens[i + 4]));
              dist.coor_to.x = border_to.x;
              dist.coor_to.y = border_to.y;

              emigrates.emplace_back(Migrate{dist.robot, border_from, border_to, dist.rank});

              const auto corner_neighbor_map_it = recv_area_info.corner_neighbors_map.find(border_from);
              if (corner_neighbor_map_it != recv_area_info.corner_neighbors_map.end())
              {
                for (const int neighbor : corner_neighbor_map_it->second)
                {
                  Block& block = recv_area.neighbor_block_map.find(neighbor)->second;
                  block.constraint += ":-m(_,(" + std::to_string(border_from.x) + "," + std::to_string(border_from.y) + "),_,_).";
                  ++block.num_out_blocked;
                }
              }
            }
            else  // Otherwise the robot is coming in
            {
              // used by the receiver when it needs to decide which robots to reject
              dist.rank = robot_rank_map.find(dist.robot)->second;

              // used by the receiver to be the goal in movement planning
              const Coor border_from(std::stoi(tokens[i + 1]), std::stoi(tokens[i + 2]));
              dist.coor_from.x = border_from.x;
              dist.coor_from.y = border_from.y;

              // used by the receiver to calculate the migrating direction (was not used originally)
              const Coor border_to(std::stoi(tokens[i + 3]), std::stoi(tokens[i + 4]));
              dist.coor_to.x = border_to.x;
              dist.coor_to.y = border_to.y;

              immigrates.emplace_back(Migrate{dist.robot, border_from, border_to, dist.rank});

              const auto corner_neighbor_map_it = recv_area_info.corner_neighbors_map.find(border_to);
              if (corner_neighbor_map_it != recv_area_info.corner_neighbors_map.end())
              {
                for (const int neighbor : corner_neighbor_map_it->second)
                {
                  Block& block = recv_area.neighbor_block_map.find(neighbor)->second;
                  block.constraint += ":-m(_,_,(" + std::to_string(border_to.x) + "," + std::to_string(border_to.y) + "),_).";
                  ++block.num_in_blocked;
                }
              }
            }
          }
        }
        else
        {
          std::cout << YELLOW << g_solver_name << "-a" << sent_to << " cannot make a border assignment for" << RESET << std::endl
                    << abcd << "l(" << num_limits << ")." << block.constraint << connection.link_lp << std::endl;
        }
      }
      else
      {
        std::cout << RED << g_solver_name << "-a" << sent_to << " cannot make a border assignment for" << RESET << std::endl
                  << abcd << "l(" << num_limits << ")." << block.constraint << connection.link_lp << std::endl;
      }

      clingo_control_free(ctl);
    }
      ++g_num_migrate_negotiate_req_received;
      break;

    case 1:
    {
      const int sent_to = req.area_to;

#ifdef DEBUG_VERBOSE_2
      {
        const int sent_from = req.area_from;
        const int solver_from = g_area_info_map.find(sent_to)->second.neighbor_connection_map.find(sent_from)->second.solver;
        std::string debug_str("-a" + std::to_string(sent_to) + " receives a " + RED + "REJ" + RESET +
                              " request from s" + std::to_string(solver_from) + "-a" + std::to_string(sent_from));
        debug_str += "\n- s" + std::to_string(solver_from) + "-a" + std::to_string(sent_from) + " cancels";
        for (const int canceled_robot : req.canceled_robots)
        {
          debug_str += " r" + std::to_string(canceled_robot);
        }

        debug_str += "\n- s" + std::to_string(solver_from) + "-a" + std::to_string(sent_from) + " rejects";
        for (const int rejected_robot : req.rejected_robots)
        {
          debug_str += " r" + std::to_string(rejected_robot);
        }

        debug_str += "\n- s" + std::to_string(g_solver_id) + "-a" + std::to_string(sent_to) + " cancels";
        const auto area_canceled_robot_map_it = g_area_canceled_robots_map.find(sent_to);
        if (area_canceled_robot_map_it != g_area_canceled_robots_map.end())
        {
          for (const int canceled_robot : area_canceled_robot_map_it->second)
          {
            debug_str += " r" + std::to_string(canceled_robot);
          }
        }

        debug_str += "\n- s" + std::to_string(g_solver_id) + "-a" + std::to_string(sent_to) + " rejects";
        const auto area_rejected_robot_map_it = g_area_rejected_robots_map.find(sent_to);
        if (area_rejected_robot_map_it != g_area_rejected_robots_map.end())
        {
          for (const int rejected_robot : area_rejected_robot_map_it->second)
          {
            debug_str += " r" + std::to_string(rejected_robot);
          }
        }

        std::cout << g_solver_name << debug_str << std::endl;
      }
#endif

      if (!req.canceled_robots.empty())
      {
        auto area_rejected_robots_map_req_it = g_area_rejected_robots_map_req.find(sent_to);
        if (area_rejected_robots_map_req_it == g_area_rejected_robots_map_req.end())
        {
          area_rejected_robots_map_req_it = g_area_rejected_robots_map_req.emplace(sent_to, std::vector<int>{}).first;
        }
        area_rejected_robots_map_req_it->second.insert(area_rejected_robots_map_req_it->second.end(), req.canceled_robots.begin(), req.canceled_robots.end());
      }

      if (!req.rejected_robots.empty())
      {
        auto area_canceled_robots_map_req_it = g_area_canceled_robots_map_req.find(sent_to);
        if (area_canceled_robots_map_req_it == g_area_canceled_robots_map_req.end())
        {
          area_canceled_robots_map_req_it = g_area_canceled_robots_map_req.emplace(sent_to, std::vector<int>{}).first;
        }
        area_canceled_robots_map_req_it->second.insert(area_canceled_robots_map_req_it->second.end(), req.rejected_robots.begin(), req.rejected_robots.end());
      }

      res.area_from = sent_to;
      res.area_to = req.area_from;

      const auto area_canceled_robot_map_it = g_area_canceled_robots_map.find(sent_to);
      if (area_canceled_robot_map_it != g_area_canceled_robots_map.end())
      {
        std::copy(area_canceled_robot_map_it->second.begin(), area_canceled_robot_map_it->second.end(), std::back_inserter(res.canceled_robots));
      }

      const auto area_rejected_robot_map_it = g_area_rejected_robots_map.find(sent_to);
      if (area_rejected_robot_map_it != g_area_rejected_robots_map.end())
      {
        std::copy(area_rejected_robot_map_it->second.begin(), area_rejected_robot_map_it->second.end(), std::back_inserter(res.rejected_robots));
      }
    }
      ++g_num_migrate_reject_req_received;
      break;

    case 2:
    {
      const int sent_from = req.area_from;
      const int sent_to = req.area_to;

#ifdef DEBUG_VERBOSE_2
      {
        const int solver_from = g_area_info_map.find(sent_to)->second.neighbor_connection_map.find(sent_from)->second.solver;
        std::string debug_str("-a" + std::to_string(sent_to) + " receives a " + GREEN + "CFM" + RESET +
                              " request from s" + std::to_string(solver_from) + "-a" + std::to_string(sent_from));
        debug_str += "\n- s" + std::to_string(solver_from) + "-a" + std::to_string(sent_from) + " sends";
        for (dmapf::Transit& t : req.transits)
        {
          debug_str += " (r" + std::to_string(t.robot) + ",(" + std::to_string(t.coor_from.x) + "," + std::to_string(t.coor_from.y) + "))";
        }
        debug_str += "\n- s" + std::to_string(g_solver_id) + "-a" + std::to_string(sent_to) + " sends";

        const auto area_neighbor_confirmed_transits_map_it = g_area_neighbor_confirmed_transits_map.find(sent_to);
        if (area_neighbor_confirmed_transits_map_it != g_area_neighbor_confirmed_transits_map.end())
        {
          const auto neighbor_confirmed_transits_map_it = area_neighbor_confirmed_transits_map_it->second.find(sent_from);
          if (neighbor_confirmed_transits_map_it != area_neighbor_confirmed_transits_map_it->second.end())
          {
            for (const auto& robot_transit_pair : neighbor_confirmed_transits_map_it->second)
            {
              debug_str += " (r" + std::to_string(robot_transit_pair.first) + ",(" +
                  std::to_string(robot_transit_pair.second.coor_from.x) + "," + std::to_string(robot_transit_pair.second.coor_from.y) + "))";
            }
          }
        }
        std::cout << g_solver_name << debug_str << std::endl;
      }
#endif

      auto area_confirmed_transits_map_it = g_area_confirmed_transits_map.find(sent_to);
      if (area_confirmed_transits_map_it == g_area_confirmed_transits_map.end())
      {
        area_confirmed_transits_map_it = g_area_confirmed_transits_map.emplace(sent_to, std::vector<std::pair<int, Transit>>{}).first;
      }

      std::vector<std::pair<int, Transit>>& confirmed_transits = area_confirmed_transits_map_it->second;
      for (dmapf::Transit& t : req.transits)
      {
        Transit transit;
        transit.coor_from.x = t.coor_from.x;
        transit.coor_from.y = t.coor_from.y;
        transit.coor_goal.x = t.coor_goal.x;
        transit.coor_goal.y = t.coor_goal.y;
        transit.direction.x = t.direction.x;
        transit.direction.y = t.direction.y;
        transit.next_areas = t.next_areas;
        confirmed_transits.emplace_back(std::pair<int, Transit>(t.robot, std::move(transit)));
      }

      const auto area_neighbor_confirmed_transits_map_it = g_area_neighbor_confirmed_transits_map.find(sent_to);
      if (area_neighbor_confirmed_transits_map_it != g_area_neighbor_confirmed_transits_map.end())
      {
        const auto neighbor_confirmed_transits_map_it = area_neighbor_confirmed_transits_map_it->second.find(sent_from);
        if (neighbor_confirmed_transits_map_it != area_neighbor_confirmed_transits_map_it->second.end())
        {
          for (const auto& robot_transit_pair : neighbor_confirmed_transits_map_it->second)
          {
            dmapf::Transit transit;
            transit.robot = robot_transit_pair.first;
            transit.coor_from.x = robot_transit_pair.second.coor_from.x;
            transit.coor_from.y = robot_transit_pair.second.coor_from.y;
            transit.coor_goal.x = robot_transit_pair.second.coor_goal.x;
            transit.coor_goal.y = robot_transit_pair.second.coor_goal.y;
            transit.direction.x = robot_transit_pair.second.direction.x;
            transit.direction.y = robot_transit_pair.second.direction.y;
            transit.next_areas = robot_transit_pair.second.next_areas;
            res.transits.emplace_back(std::move(transit));
          }
        }
      }
    }
      ++g_num_migrate_confirm_req_received;
      break;

    default:
      std::cout << YELLOW << g_solver_name << "-a" << req.area_to << " receives an unknown migration request" << RESET << std::endl;
  }

#ifdef DEBUG_VERBOSE_3
  {
    const int sent_from = req.area_from;
    const int solver_from = g_area_info_map.find(req.area_to)->second.neighbor_connection_map.find(sent_from)->second.solver;
    switch (req.type)
    {
      case 0:
        std::cout << g_solver_name << "-a" << req.area_to << " returns true to NEG request from s" << solver_from << "-a" << sent_from << std::endl;
        break;

      case 1:
        std::cout << g_solver_name << "-a" << req.area_to << " returns true to REJ request from s" << solver_from << "-a" << sent_from << std::endl;
        break;

      case 2:
        std::cout << g_solver_name << "-a" << req.area_to << " returns true to CFM request from s" << solver_from << "-a" << sent_from << std::endl;
        break;

      default:
        std::cout << YELLOW << g_solver_name << "-a" << req.area_to << " returns true to UNKNOWN request from s" << solver_from << "-a" << sent_from << RESET << std::endl;
        break;
    }
  }
#endif

  return true;
}

int64_t g_time_start = std::numeric_limits<int64_t>::max();
int64_t g_time_end = std::numeric_limits<int64_t>::min();
std::map<int, std::vector<dmapf::Move>> g_round_moves_map;
bool aggregate_service(dmapf::Aggregate::Request& req, dmapf::Aggregate::Response& res)
{
  for (const dmapf::Plan& plan : req.plans)
  {
    auto round_moves_map_it = g_round_moves_map.find(plan.round);
    if (round_moves_map_it == g_round_moves_map.end())
    {
      round_moves_map_it = g_round_moves_map.emplace(plan.round, std::vector<dmapf::Move>{}).first;
    }

    std::vector<dmapf::Move>& moves = round_moves_map_it->second;
    moves.insert(moves.end(), plan.moves.begin(), plan.moves.end());
  }

  if (req.time_start < g_time_start)
  {
    g_time_start = req.time_start;
  }

  if (req.time_end > g_time_end)
  {
    g_time_end = req.time_end;
  }
  ++g_num_aggregate_req_received;

  return true;
}

#if defined(DEBUG_ABSTRACT) || defined(ABSTRACT_UCS) || defined(ABSTRACT_UCS_CENTRAL)
std::vector<std::vector<int>> g_steps_num_robots;
std::vector<int> g_num_nodes;
#endif

#ifdef DEBUG_ABSTRACT
int64_t g_time_abstract_total = std::numeric_limits<int64_t>::min();
#endif

#ifdef ABSTRACT_UCS_CENTRAL
struct SolverAims
{
  int solver;
  std::vector<dmapf::Aim> aims;
};

std::vector<SolverAims> g_solver_aims;
std::vector<dmapf::Next> g_nexts;
#endif

#if defined(DEBUG_ABSTRACT) || defined(ABSTRACT_UCS_CENTRAL)
bool abstract_service(dmapf::Abstract::Request& req, dmapf::Abstract::Response& res)
{
  switch (req.type)
  {
#ifdef ABSTRACT_UCS_CENTRAL
    case 0:
    {
      for (const auto& num_node : req.num_nodes)
      {
        g_num_nodes[num_node.area] = num_node.var;
      }
      g_solver_aims.emplace_back(SolverAims{req.solver, std::move(req.aims)});
    }
    break;

    case 1:
    {
      g_nexts = std::move(req.nexts);
    }
    break;
#else
    case 2:
    {
#ifndef ABSTRACT_UCS
      for (const auto& num_node : req.num_nodes)
      {
        g_num_nodes[num_node.area] = num_node.var;
      }
#endif

      const int num_step_requested = req.steps_num_robots.size();
      if (num_step_requested > 0)
      {
        const int num_step_current = g_steps_num_robots.size();

        // Expand the vector
        for (int i = num_step_current; i < num_step_requested; ++i)
        {
          g_steps_num_robots.emplace_back(g_steps_num_robots[i - 1]);
        }

        // Include the requested steps
        for (int i = 0; i < num_step_requested; ++i)
        {
          for (const auto& num_robot : req.steps_num_robots[i].areas)
          {
            g_steps_num_robots[i][num_robot.area] += num_robot.var;
          }
        }

        // Extend the requested steps to the whole vector
        const auto& last = req.steps_num_robots.back();
        for (int i = num_step_requested; i < num_step_current; ++i)
        {
          for (const auto& num_robot : last.areas)
          {
            g_steps_num_robots[i][num_robot.area] += num_robot.var;
          }
        }
      }

      if (req.time_total > g_time_abstract_total)
      {
        g_time_abstract_total = req.time_total;
      }
    }
    break;
#endif

    default:
      std::cout << YELLOW << g_solver_name << " receives an unknown abstract request" << RESET << std::endl;
  }

  ++g_num_abstract_req_received;
  return true;
}
#endif

#ifdef ABSTRACT_UCS
void areas_callback(const dmapf::Areas& msg)
{
  for (const auto& num_node : msg.areas)
  {
    g_num_nodes[num_node.area] = num_node.var;
  }
  --g_num_areas_msg_expected;
}
#endif

#if !defined(ABSTRACT_ASP) && !defined(ABSTRACT_ASP_ITERABLE)
bool search_abstract_plan(const int area_start, const int area_goal, const std::vector<std::vector<int>>& adj, std::vector<int>& next_areas)
{
  if (area_start == area_goal)
  {
    return true;
  }

  std::vector<AbstractNode> nodes{AbstractNode{area_start, -1}};

#if defined(ABSTRACT_BFS) || defined(ABSTRACT_BFS_RANDOM) || defined(ABSTRACT_RANDOM)
  std::unordered_set<int> visited{area_start};
#else
  std::unordered_set<int> visited;
#endif

#if defined(ABSTRACT_BFS) || defined(ABSTRACT_BFS_RANDOM)
  std::queue<int> queue;
  queue.push(0);
#elif defined(ABSTRACT_RANDOM)
  std::vector<int> queue;
  queue.push_back(0);
#elif defined(ABSTRACT_UCS) || defined(ABSTRACT_UCS_CENTRAL)
  std::priority_queue<AbstractRank> queue;
  queue.push(AbstractRank{0.0, 0, 0});
#endif

  while (!queue.empty())
  {
#if defined(ABSTRACT_BFS) || defined(ABSTRACT_BFS_RANDOM)
    const int i = queue.front();
    queue.pop();
#elif defined(ABSTRACT_RANDOM)
    const int r = rand() % queue.size();
    const int i = queue[r];
    queue[r] = queue.back();
    queue.pop_back();
#elif defined(ABSTRACT_UCS) || defined(ABSTRACT_UCS_CENTRAL)
    const AbstractRank rank = queue.top();
    const int i = rank.index;
    queue.pop();

    if (visited.find(nodes[i].area) != visited.end())
    {
      continue;
    }
    visited.insert(nodes[i].area);
#endif

    // Found the abstract plan
    if (nodes[i].area == area_goal)
    {
      next_areas.push_back(area_goal);

      int p = nodes[i].parent;
      while (p > 0)
      {
        next_areas.push_back(nodes[p].area);
        p = nodes[p].parent;
      }

      return true;
    }

#if defined(ABSTRACT_UCS) || defined(ABSTRACT_UCS_CENTRAL)
    // Expand the vector
    for (int j = g_steps_num_robots.size(); j <= rank.step + 1; ++j)
    {
      g_steps_num_robots.emplace_back(g_steps_num_robots[j - 1]);
    }
#endif

    // Otherwise, generate child nodes
#ifdef ABSTRACT_BFS_RANDOM
    std::vector<int> rand_adj = adj[nodes[i].area];
    while (!rand_adj.empty())
    {
      const int r = rand() % rand_adj.size();
      const int connected_area = rand_adj[r];
      rand_adj[r] = rand_adj.back();
      rand_adj.pop_back();

      if (visited.find(connected_area) == visited.end())
      {
        visited.insert(connected_area);
        nodes.emplace_back(AbstractNode{connected_area, i});
        queue.push(nodes.size() - 1);
      }
    }
#else
    for (const int connected_area : adj[nodes[i].area])
    {
      if (visited.find(connected_area) == visited.end())
      {
#if defined(ABSTRACT_BFS) || defined(ABSTRACT_RANDOM)
        visited.insert(connected_area);
#endif
        nodes.emplace_back(AbstractNode{connected_area, i});
#if defined(ABSTRACT_BFS)
        queue.push(nodes.size() - 1);
#elif defined(ABSTRACT_RANDOM)
        queue.push_back(nodes.size() - 1);
#elif defined(ABSTRACT_UCS) || defined(ABSTRACT_UCS_CENTRAL)
        double congestion = (double)(g_steps_num_robots[rank.step + 1][connected_area] + 1) / g_num_nodes[connected_area];
        queue.emplace(AbstractRank{rank.congestion + congestion, rank.step + 1, (int)nodes.size() - 1});
#endif
      }
    }
#endif
  }

  return false;
}
#endif

void track_callback(const dmapf::Track& msg)
{
  switch (msg.type)
  {
    case 0:
      // Accumulate partial information
      if (msg.active)
      {
        g_active_solvers_set_volatile.insert(msg.solver);

        // Only store the crossing information if it's related to the solver
        if (g_solver_id == msg.solver)
        {
          for (const auto& cross : msg.crosses)
          {
            g_active_solvers_set_volatile.insert(cross.solver_to);  // The receiving solver needs to be active as well
            g_crosses_volatile.emplace_back(Cross {msg.solver, cross.area_from, cross.solver_to, cross.area_to});
          }
        }
        else
        {
          for (const auto& cross : msg.crosses)
          {
            g_active_solvers_set_volatile.insert(cross.solver_to);  // The receiving solver needs to be active as well
            if (g_solver_id == cross.solver_to)
            {
              g_crosses_volatile.emplace_back(Cross {msg.solver, cross.area_from, cross.solver_to, cross.area_to});
            }
          }
        }
      }

      if (g_num_track_msg_expected == 1)  // Reset
      {
        // Set the final resulting variables
        g_active_solvers_set = g_active_solvers_set_volatile;
        g_crosses = g_crosses_volatile;

        // Reset the volatile variables
        g_active_solvers_set_volatile.clear();
        g_crosses_volatile.clear();

        // Reset the counter
        g_num_track_msg_expected = 0;   // Has to be at the end to prevent prematurely accessing of data
      }
      else
      {
        // Decrease the counter
        --g_num_track_msg_expected;
      }
      break;

    case 1:
      --g_num_track_msg_ignored_1;
      break;

    case 2:
      --g_num_track_msg_ignored_2;
      break;

    default:
      std::cout << YELLOW << g_solver_name << " receives an unknown tracking message (type = " << msg.type << ")" << RESET << std::endl;
  }
}

#ifndef SOLVER_ASP
std::string get_instance_str(const AreaInfo& area_info, const std::vector<int>& robot_ids, const Instance& instance)
{
  std::string str = "Avoid locations:";
  for (int in : instance.avoid_locations)
  {
    str += " " + std::to_string(in) +
           "(" + std::to_string(instance.getColCoordinate(in) + area_info.min_node.x) +
           "," + std::to_string(instance.getRowCoordinate(in) + area_info.min_node.y) + ")";
  }

  for (int i = 0; i < instance.num_of_agents; ++i)
  {
    const int s = instance.start_locations[i];
    const int g = instance.goal_locations[i];
    if (g >= 0)
    {
      str += "\nRobot " + std::to_string(robot_ids[i]) +
             ": " + std::to_string(s) +
             "(" + std::to_string(instance.getColCoordinate(s) + area_info.min_node.x) +
             "," + std::to_string(instance.getRowCoordinate(s) + area_info.min_node.y) +
             ") -> " + std::to_string(g) +
             "(" + std::to_string(instance.getColCoordinate(g) + area_info.min_node.x) +
             "," + std::to_string(instance.getRowCoordinate(g) + area_info.min_node.y) + ")";
    }
    else
    {
      str += "\nRobot " + std::to_string(robot_ids[i]) +
             ": " + std::to_string(s) +
             "(" + std::to_string(instance.getColCoordinate(s) + area_info.min_node.x) +
             "," + std::to_string(instance.getRowCoordinate(s) + area_info.min_node.y) +
             ") -> -1";
    }
  }

  str += "\ntype octile\nheight " + std::to_string(instance.num_of_rows) + "\nwidth " + std::to_string(instance.num_of_cols) + "\nmap\n";
  int location = -1;
  for (int i = 0; i < instance.num_of_rows; ++i)
  {
    for (int j = 0; j < instance.num_of_cols; ++j)
    {
      str += instance.my_map[++location] ? "@" : ".";
    }
    str += "\n";
  }

  str += "version 1";
  for (int i = 0; i < instance.num_of_agents; ++i)
  {
    const int s = instance.start_locations[i];
    const int g = instance.goal_locations[i];
    const Coor s_coor(instance.getColCoordinate(s), instance.getRowCoordinate(s));
    Coor g_coor(-1, -1);
    int d = 0;
    if (g >= 0)
    {
      g_coor.x = instance.getColCoordinate(g);
      g_coor.y = instance.getRowCoordinate(g);
      d = std::abs(s_coor.x - g_coor.x) + std::abs(s_coor.y - g_coor.y);
    }

    str += "\n" + std::to_string(robot_ids[i]) +
           "\tmy.map\t" + std::to_string(instance.num_of_cols) +
           "\t" + std::to_string(instance.num_of_rows) +
           "\t" + std::to_string(s_coor.x) + "\t" + std::to_string(s_coor.y) +
           "\t" + std::to_string(g_coor.x) + "\t" + std::to_string(g_coor.y) +
           "\t" + std::to_string(d);
  }

  return str;
}
#endif

#ifdef DEBUG_ABSTRACT
void print_abstract_stats()
{
  double max_congestion = DBL_MIN;
  double min_congestion = DBL_MAX;

  for (int i = 0; i < g_steps_num_robots.size(); ++i)
  {
    std::string debug_str;
    int total_robots = 0;

    double max_congestion_local = DBL_MIN;
    double min_congestion_local = DBL_MAX;

    const std::vector<int>& num_robots = g_steps_num_robots[i];
    for (int area = 1; area < num_robots.size(); ++area)
    {
#ifdef DEBUG_ABSTRACT_VERBOSE
      debug_str += "\n- a" + std::to_string(area) + ": " + std::to_string(num_robots[area]) + " / " + std::to_string(g_num_nodes[area]);
#endif

      total_robots += num_robots[area];

      const double congestion = (double)num_robots[area] / g_num_nodes[area];
      if (congestion > max_congestion_local)
      {
        max_congestion_local = congestion;
        if (max_congestion_local > max_congestion)
        {
          max_congestion = max_congestion_local;
        }
      }
      if (congestion < min_congestion_local)
      {
        min_congestion_local = congestion;
        if (min_congestion_local < min_congestion)
        {
          min_congestion = min_congestion_local;
        }
      }
    }

    std::cout << i << ", " << max_congestion_local << ", " << min_congestion_local << ", " << total_robots << debug_str << std::endl;
  }

  std::cout << "----------------------------------------------------------------" << std::endl
#ifdef ABSTRACT_ASP
            << "Using ABSTRACT_ASP" << std::endl
#endif
#ifdef ABSTRACT_ASP_ITERABLE
            << "Using ABSTRACT_ASP_ITERABLE" << std::endl
#endif
#ifdef ABSTRACT_BFS
            << "Using ABSTRACT_BFS" << std::endl
#endif
#ifdef ABSTRACT_BFS_RANDOM
            << "Using ABSTRACT_BFS_RANDOM" << std::endl
#endif
#ifdef ABSTRACT_RANDOM
            << "Using ABSTRACT_RANDOM" << std::endl
#endif
#ifdef ABSTRACT_UCS
            << "Using ABSTRACT_UCS" << std::endl
#endif
#ifdef ABSTRACT_UCS_CENTRAL
            << "Using ABSTRACT_UCS_CENTRAL" << std::endl
#endif
            << "Time    : " << format_time(g_time_abstract_total) << std::endl
            << "#steps  : " << g_steps_num_robots.size() << std::endl
            << "Congestion" << std::endl
            << "- max   : " << max_congestion << std::endl
            << "- min   : " << min_congestion << std::endl
            << "- range : " << max_congestion - min_congestion << std::endl;
}
#endif

int main(int argc, char **argv)
{
  const int MASTER_SOLVER_ID = 1;
  const std::string SOLVER_NAME_PREFIX("s");

  const std::string AGGREGATE_SERVICE_NAME("aggregate");
  const std::string MIGRATE_SERVICE_NAME("migrate");
#if defined(DEBUG_ABSTRACT) || defined(ABSTRACT_UCS_CENTRAL)
  const std::string ABSTRACT_SERVICE_NAME("abstract");
#endif

  ros::init(argc, argv, "solver", ros::init_options::NoRosout);
  ros::NodeHandle nh;
  ros::Rate rate(RATE);

  // Start the timer
  std::chrono::steady_clock::time_point time_point_start = std::chrono::steady_clock::now();

  // Create asynchronous spinners - important: communication often hangs without it
  ros::AsyncSpinner async_spinner(1);   // The number of threads = 0 means to use the number of processor cores
  async_spinner.start();

  std::srand((unsigned)std::time(nullptr));

  // Set up solver name and ID
  g_solver_name = ros::this_node::getName().substr(ros::this_node::getName().find_last_of("/") + 1, ros::this_node::getName().size());
  g_solver_id = std::stoi(g_solver_name.substr(1, g_solver_name.size()));

  const std::string project_name("dmapf");
  const std::string project_path(ros::package::getPath(project_name));
  if (project_path.empty())
  {
    std::cerr << RED << g_solver_name << "Cannot find project " << project_name << RESET << std::endl;
    return EXIT_FAILURE;
  }
  const std::string abstract_lp(project_path + "/src/abstract.lp");
  const std::string migrate_lp(project_path + "/src/migrate.lp");

#ifdef DEBUG_TIMES
  std::chrono::steady_clock::time_point t1, t2;
#endif

#ifdef DEBUG_TIMES
  t1 = std::chrono::steady_clock::now();
#endif

  // Retrieve parameters
  std::string problem_lp;
  std::string answer_lp;
  std::string links_lp;
  int total_solvers;
  std::string domain_lp;

  if (g_solver_id == MASTER_SOLVER_ID)
  {
    retrieve("problem", problem_lp);
    retrieve("answer", answer_lp);
  }
  retrieve("links", links_lp);
  retrieve("areas", g_total_areas);
  retrieve("solvers", total_solvers);
  retrieve("~domain", domain_lp);

#if defined(DEBUG_ABSTRACT) || defined(ABSTRACT_UCS) || defined(ABSTRACT_UCS_CENTRAL)
  g_steps_num_robots.emplace_back(std::vector<int>(g_total_areas + 1, 0));
  g_num_nodes.resize(g_total_areas + 1, 0);
#endif

  // Set up publishers
#ifdef ABSTRACT_UCS
  ros::Publisher areas_pub = nh.advertise<dmapf::Areas>("areas", total_solvers);
#endif
  ros::Publisher track_pub = nh.advertise<dmapf::Track>("track", total_solvers);

  // Set up subscribers
#ifdef ABSTRACT_UCS
  ros::Subscriber areas_sub = nh.subscribe("areas", total_solvers, areas_callback);
#endif
  ros::Subscriber track_sub = nh.subscribe("track", total_solvers, track_callback);

  // Set up service servers
  ros::ServiceServer migrate_server = nh.advertiseService(g_solver_name + "/" + MIGRATE_SERVICE_NAME, migrate_service);
  ros::ServiceServer aggregate_server;
  if (g_solver_id == MASTER_SOLVER_ID)
  {
    aggregate_server = nh.advertiseService(g_solver_name + "/" + AGGREGATE_SERVICE_NAME, aggregate_service);
  }

#if defined(DEBUG_ABSTRACT) || defined (ABSTRACT_UCS_CENTRAL)
  ros::ServiceServer abstract_server;
#ifndef ABSTRACT_UCS_CENTRAL
  if (g_solver_id == MASTER_SOLVER_ID)
#endif
  {
    abstract_server = nh.advertiseService(g_solver_name + "/" + ABSTRACT_SERVICE_NAME, abstract_service);
  }
#endif

  // Read the content of abstract.lp
  std::string abstract_lp_content;
  {
    std::ifstream src(abstract_lp, std::ios::binary);
    if (!src.is_open())
    {
      std::cerr << RED << g_solver_name << ": Cannot open abstract.lp at " << abstract_lp << RESET << std::endl;
      return EXIT_FAILURE;
    }
    std::stringstream buffer;
    buffer << src.rdbuf();
    abstract_lp_content = buffer.str();
    src.close();
  }

  // Read the content of links.lp
  std::string links_lp_content;
  {
    std::ifstream src(links_lp, std::ios::binary);
    if (!src.is_open())
    {
      std::cerr << RED << g_solver_name << ": Cannot open links.lp at " << links_lp << RESET << std::endl;
      return EXIT_FAILURE;
    }
    std::stringstream buffer;
    buffer << src.rdbuf();
    links_lp_content = buffer.str();
    src.close();
  }

  // Read the content of migrate.lp
  {
    std::ifstream src(migrate_lp, std::ios::binary);
    if (!src.is_open())
    {
      std::cerr << RED << g_solver_name << ": Cannot open migrate.lp at " << migrate_lp << RESET << std::endl;
      return EXIT_FAILURE;
    }
    std::stringstream buffer;
    buffer << src.rdbuf();
    g_migrate_lp_content = buffer.str();
    src.close();
  }

#ifdef SOLVER_ASP
  // Read the content of movement.lp
  std::string movement_lp_content;
  {
    const std::string movement_lp(project_path + "/solvers/ASP/movement.lp");
    std::ifstream src(movement_lp, std::ios::binary);
    if (!src.is_open())
    {
      std::cerr << RED << g_solver_name << ": Cannot open movement.lp at " << movement_lp << RESET << std::endl;
      return EXIT_FAILURE;
    }
    std::stringstream buffer;
    buffer << src.rdbuf();
    movement_lp_content = buffer.str();
    src.close();
  }
#endif

  // Read the content of the domain file
  std::string domain_lp_content;
  {
    std::ifstream src(domain_lp, std::ios::binary);
    if (!src.is_open())
    {
      std::cerr << RED << g_solver_name << ": Cannot open the domain file at " << domain_lp << RESET << std::endl;
      return EXIT_FAILURE;
    }
    std::stringstream buffer;
    buffer << src.rdbuf();
    domain_lp_content = buffer.str();
    src.close();
  }

  // Get domain info
  std::unordered_map<int, int> area_latest_round_map;
  {
#ifdef DEBUG_ABSTRACT
    std::chrono::steady_clock::time_point time_abstract_total{};
#endif
    std::vector<std::string> tokens = tokenizeString(domain_lp_content, "%");
    std::stringstream info_stream(tokens[0]);

    // Get areas info
    std::vector<int> ordered_areas;             // Used to keep the order of areas
    std::vector<int> ordered_connected_areas;   // Used to keep the order of connected areas
    int num_areas;
    info_stream >> num_areas;
    ordered_areas.reserve(num_areas);

#if defined(DEBUG_ABSTRACT) && !defined(ABSTRACT_UCS) && !defined(ABSTRACT_UCS_CENTRAL)
    std::vector<std::vector<int>> steps_num_robots;
    std::vector<dmapf::Area> num_nodes;
    num_nodes.reserve(num_areas);
#endif
#if defined(ABSTRACT_UCS) || defined(ABSTRACT_UCS_CENTRAL)
    dmapf::Areas areas_msg;
    areas_msg.areas.reserve(num_areas);
#endif

    for (int i = 0; i < num_areas; ++i)
    {
      int area;
      info_stream >> area;
      ordered_areas.push_back(area);

      AreaInfo area_info = {};
      area_info.solving_time_per_robot = 0.0;
      info_stream >> area_info.num_nodes;

#if defined(DEBUG_ABSTRACT) && !defined(ABSTRACT_UCS) && !defined(ABSTRACT_UCS_CENTRAL)
      num_nodes.emplace_back(dmapf::Area{});
      auto& num_nodes_back = num_nodes.back();
      num_nodes_back.area = area;
      num_nodes_back.var = area_info.num_nodes;
#endif
#if defined(ABSTRACT_UCS) || defined(ABSTRACT_UCS_CENTRAL)
      areas_msg.areas.emplace_back(dmapf::Area{});
      auto& areas_msg_back = areas_msg.areas.back();
      areas_msg_back.area = area;
      areas_msg_back.var = area_info.num_nodes;

#ifdef ABSTRACT_UCS_CENTRAL
      g_num_nodes[area] = area_info.num_nodes;
#endif
#endif

#ifdef SOLVER_ASP
      // Skip min_node and max_node
      int dummy;
      info_stream >> dummy; info_stream >> dummy; info_stream >> dummy; info_stream >> dummy;
#else
      info_stream >> area_info.min_node.x;
      info_stream >> area_info.min_node.y;
      info_stream >> area_info.max_node.x;
      info_stream >> area_info.max_node.y;
#endif

      int num_connections;
      info_stream >> num_connections;
      ordered_connected_areas.reserve(num_connections);

      for (int j = 0; j < num_connections; ++j)
      {
        int connected_area;
        int num_corners;
        Connection connection = {};
        info_stream >> connected_area;
        info_stream >> connection.solver;
        info_stream >> connection.num_links;
        info_stream >> num_corners;
        for (int k = 0; k < num_corners; ++k)
        {
          Coor corner;
          info_stream >> corner.x;
          info_stream >> corner.y;

          auto corner_neighbors_map_it = area_info.corner_neighbors_map.find(corner);
          if (corner_neighbors_map_it == area_info.corner_neighbors_map.end())
          {
            corner_neighbors_map_it = area_info.corner_neighbors_map.emplace(corner, std::vector<int>{}).first;
          }
          corner_neighbors_map_it->second.push_back(connected_area);
        }
        ordered_connected_areas.push_back(connected_area);
        area_info.neighbor_connection_map.emplace(connected_area, std::move(connection));
      }
      g_area_info_map.emplace(area, std::move(area_info));
    }

#ifdef ABSTRACT_UCS
    // Make sure all solvers have subscribed to Track topic
    wait_for_subscribers(areas_pub, total_solvers);

    // Tell other solvers about the size of areas in this solver
    g_num_areas_msg_expected += total_solvers;   // must use += and before publishing
    areas_pub.publish(areas_msg);

    // Wait until the information of every area is gathered
    while (ros::ok() && g_num_areas_msg_expected > 0)
    {
      rate.sleep();
    }
#endif

    // Get robots info
    int num_areas_with_robot;
    info_stream >> num_areas_with_robot;

#ifdef ABSTRACT_UCS_CENTRAL
    dmapf::Abstract abstract_srv;
    std::unordered_map<int, Transit> robot_transit_map;
    if (g_solver_id == MASTER_SOLVER_ID)
    {
      abstract_srv.request.type = 1;

      // Wait for abstract plan requests from other solvers
      while (ros::ok() && g_num_abstract_req_received < total_solvers - 1)
      {
        rate.sleep();
      }
      g_num_abstract_req_received = 0; // Reset the counter
    }
    else
    {
      abstract_srv.request.type = 0;
      abstract_srv.request.solver = g_solver_id;
      abstract_srv.request.aims.reserve(num_areas_with_robot);
      abstract_srv.request.num_nodes = std::move(areas_msg.areas);
    }
#endif

#if !defined(ABSTRACT_ASP) && !defined(ABSTRACT_ASP_ITERABLE)
    std::vector<std::vector<int>> adj;
#endif

    if (num_areas_with_robot > 0)
    {
#if !defined(ABSTRACT_ASP) && !defined(ABSTRACT_ASP_ITERABLE)
#ifdef ABSTRACT_UCS_CENTRAL
      if (g_solver_id == MASTER_SOLVER_ID)
#endif
      {
        // Build an adjacency list
        adj.resize(g_total_areas + 1, std::vector<int>{});
        std::vector<std::string> tokens(tokenizeString(links_lp_content, "l(,).\n"));
        for (int i = 0; i < tokens.size(); i += 2)
        {
          adj[std::stoi(tokens[i])].push_back(std::stoi(tokens[i + 1]));
        }
      }
#endif

#if defined(DEBUG_ABSTRACT) && !defined(ABSTRACT_UCS) && !defined(ABSTRACT_UCS_CENTRAL)
      steps_num_robots.emplace_back(std::vector<int>(g_total_areas + 1, 0));
#endif

      std::unordered_map<int, Area>& areas_map = g_round_areas_map.emplace(0, std::unordered_map<int, Area>{}).first->second;
      for (int i = 0; i < num_areas_with_robot; ++i)
      {
        int area_start;
        int num_robots;
        info_stream >> area_start;
        info_stream >> num_robots;

        auto areas_map_it = areas_map.emplace(area_start, Area(g_area_info_map.find(area_start)->second)).first;
        area_latest_round_map.emplace(area_start, 0);
        Area& area = areas_map_it->second;
        initialize_area(area);

#ifdef ABSTRACT_UCS_CENTRAL
        if (g_solver_id != MASTER_SOLVER_ID)
        {
          abstract_srv.request.aims.emplace_back(dmapf::Aim{});
          abstract_srv.request.aims.back().area_start = area_start;
        }
#endif

#if defined(ABSTRACT_ASP) || defined(ABSTRACT_ASP_ITERABLE) || defined(ABSTRACT_BFS)
        std::unordered_map<Coor, std::vector<int>, CoorHasher> area_pair_next_areas_map;
        area_pair_next_areas_map.reserve(num_robots);
#endif

        for (int j = 0; j < num_robots; ++j)
        {
          int robot;
          int area_goal;
          Transit transit;
          info_stream >> robot;
          info_stream >> area_goal;
          info_stream >> transit.coor_from.x;
          info_stream >> transit.coor_from.y;
          info_stream >> transit.coor_goal.x;
          info_stream >> transit.coor_goal.y;

#ifdef ABSTRACT_UCS_CENTRAL
          if (g_solver_id != MASTER_SOLVER_ID)
          {
            abstract_srv.request.aims.back().targets.emplace_back(dmapf::Target{});
            abstract_srv.request.aims.back().targets.back().robot = robot;
            abstract_srv.request.aims.back().targets.back().area_goal = area_goal;
            robot_transit_map.emplace(robot, std::move(transit));
            continue;
          }
#endif

#ifdef DEBUG_ABSTRACT
          std::chrono::steady_clock::time_point time_abstract_start = std::chrono::steady_clock::now();
#endif

#if defined(ABSTRACT_ASP) || defined(ABSTRACT_ASP_ITERABLE) || defined(ABSTRACT_BFS)
          // Check if an abstract plan for this area pair has been made
          const Coor area_pair(area_start, area_goal);
          auto area_pair_next_areas_map_it = area_pair_next_areas_map.find(area_pair);
          if (area_pair_next_areas_map_it != area_pair_next_areas_map.end())
          {
            transit.next_areas = area_pair_next_areas_map_it->second;
          }
          else
#endif
          {
#ifdef DEBUG_ABSTRACT
            time_abstract_start = std::chrono::steady_clock::now();
#endif
            // Make an abstract plan for this robot
#if defined(ABSTRACT_ASP) || defined(ABSTRACT_ASP_ITERABLE)
            // Create a control object to make abstract plans for the robot
            auto area_pair_abstract_plan_map_it = g_area_pair_abstract_plan_map.find(area_pair);
            if (area_pair_abstract_plan_map_it == g_area_pair_abstract_plan_map.end())
            {
              area_pair_abstract_plan_map_it = g_area_pair_abstract_plan_map.emplace(area_pair, AbstractPlan{}).first;

              clingo_control_new(g_clingo_argv, sizeof(g_clingo_argv) / sizeof(*g_clingo_argv), NULL, NULL, 0, &area_pair_abstract_plan_map_it->second.ctl);
              clingo_control_add(area_pair_abstract_plan_map_it->second.ctl, "base", NULL, 0,
                                 std::string("r(" + std::to_string(area_start) + ",0).g(" + std::to_string(area_goal) + ").").c_str());
              clingo_control_add(area_pair_abstract_plan_map_it->second.ctl, "base", NULL, 0, links_lp_content.c_str());

              clingo_part_t part_base = {"base", NULL, 0};
              clingo_control_ground(area_pair_abstract_plan_map_it->second.ctl, &part_base, 1, NULL, NULL);

              char const* abstract_plan_param[] = {"i"};
              clingo_control_add(area_pair_abstract_plan_map_it->second.ctl, "p", abstract_plan_param, 1, abstract_lp_content.c_str());

              area_pair_abstract_plan_map_it->second.shortest_plan_length = -1;   // Denote that the abstract plan has not been made yet
            }

            if (!make_abstract_plan(area_pair_abstract_plan_map_it->second, transit.next_areas))
            {
              std::chrono::steady_clock::time_point time_point_end = std::chrono::steady_clock::now();
              std::chrono::milliseconds time_total = std::chrono::duration_cast<std::chrono::milliseconds>(time_point_end - time_point_start);

              std::cerr << RED << g_solver_name << "-a" << area_start << " cannot find an abstract plan for r" << robot << RESET << std::endl;
              std::cout << "No solution" << std::endl
                        << "Total time: "<< format_time(time_total.count()) << std::endl;
              ros_exit();
              return EXIT_FAILURE;
            }
#else
            if (!search_abstract_plan(area_start, area_goal, adj, transit.next_areas))
            {
              std::chrono::steady_clock::time_point time_point_end = std::chrono::steady_clock::now();
              std::chrono::milliseconds time_total = std::chrono::duration_cast<std::chrono::milliseconds>(time_point_end - time_point_start);

              std::cerr << RED << g_solver_name << "-a" << area_start << " cannot find an abstract plan for r" << robot << RESET << std::endl;
              std::cout << "No solution" << std::endl
                        << "Total time: "<< format_time(time_total.count()) << std::endl;
              ros_exit();
              return EXIT_FAILURE;
            }
#endif
          }

#ifdef DEBUG_ABSTRACT
          time_abstract_total += std::chrono::steady_clock::now() - time_abstract_start;
#endif

#if defined(ABSTRACT_UCS) || defined(ABSTRACT_UCS_CENTRAL)
          std::vector<std::vector<int>>& steps_num_robots_updated = g_steps_num_robots;
#elif defined(DEBUG_ABSTRACT)
          std::vector<std::vector<int>>& steps_num_robots_updated = steps_num_robots;
#endif

#if defined(DEBUG_ABSTRACT) || defined(ABSTRACT_UCS) || defined(ABSTRACT_UCS_CENTRAL)
          const int num_step_current = steps_num_robots_updated.size();
          const int num_step_new = transit.next_areas.size();

          // Expand the vector
          for (int k = num_step_current; k < num_step_new + 1; ++k)
          {
            steps_num_robots_updated.emplace_back(steps_num_robots_updated[k - 1]);
          }

          // Update the vector
          ++steps_num_robots_updated[0][area_start];

          int k = 1;
          for (auto next_areas_rit = transit.next_areas.rbegin(); next_areas_rit != transit.next_areas.rend(); ++next_areas_rit, ++k)
          {
            ++steps_num_robots_updated[k][*next_areas_rit];
          }

          for (; k < num_step_current; ++k)
          {
            ++steps_num_robots_updated[k][area_goal];
          }
#endif

#if defined(ABSTRACT_ASP) || defined(ABSTRACT_ASP_ITERABLE) || defined(ABSTRACT_BFS)
          area_pair_next_areas_map.emplace(area_pair, transit.next_areas);
#endif

          area.add_robot(robot, std::move(transit));
        }
      }
    }

#ifdef ABSTRACT_UCS_CENTRAL
    if (g_solver_id == MASTER_SOLVER_ID)
    {
      if (adj.empty())
      {
        // Build an adjacency list if it has not been built
        adj.resize(g_total_areas + 1, std::vector<int>{});
        std::vector<std::string> tokens(tokenizeString(links_lp_content, "l(,).\n"));
        for (int i = 0; i < tokens.size(); i += 2)
        {
          adj[std::stoi(tokens[i])].push_back(std::stoi(tokens[i + 1]));
        }
      }

      // Make abstract plans for requested solvers
      for (const auto& solver_aims : g_solver_aims)
      {
        abstract_srv.request.nexts.clear();
        abstract_srv.request.nexts.reserve(solver_aims.aims.size());
        for (const auto& aim : solver_aims.aims)
        {
          abstract_srv.request.nexts.emplace_back(dmapf::Next{});
          auto& next = abstract_srv.request.nexts.back();
          next.area_start = aim.area_start;

          for (const auto& target : aim.targets)
          {
            next.steps.emplace_back(dmapf::Step{});
            auto& step = next.steps.back();
            step.robot = target.robot;

#ifdef DEBUG_ABSTRACT
            std::chrono::steady_clock::time_point time_abstract_start = std::chrono::steady_clock::now();
#endif
            if (!search_abstract_plan(aim.area_start, target.area_goal, adj, step.areas))
            {
              std::chrono::steady_clock::time_point time_point_end = std::chrono::steady_clock::now();
              std::chrono::milliseconds time_total = std::chrono::duration_cast<std::chrono::milliseconds>(time_point_end - time_point_start);

              std::cerr << RED << solver_aims.solver << "-a" << aim.area_start << " cannot find an abstract plan for r" << target.robot << RESET << std::endl;
              std::cout << "No solution" << std::endl
                        << "Total time: "<< format_time(time_total.count()) << std::endl;
              ros_exit();
              return EXIT_FAILURE;
            }

#ifdef DEBUG_ABSTRACT
            time_abstract_total += std::chrono::steady_clock::now() - time_abstract_start;
#endif

            const int num_step_current = g_steps_num_robots.size();
            const int num_step_new = step.areas.size();

            // Expand the vector
            for (int i = num_step_current; i < num_step_new + 1; ++i)
            {
              g_steps_num_robots.emplace_back(g_steps_num_robots[i - 1]);
            }

            // Update the vector
            ++g_steps_num_robots[0][aim.area_start];

            int i = 1;
            for (auto next_areas_rit = step.areas.rbegin(); next_areas_rit != step.areas.rend(); ++next_areas_rit, ++i)
            {
              ++g_steps_num_robots[i][*next_areas_rit];
            }

            for (; i < num_step_current; ++i)
            {
              ++g_steps_num_robots[i][target.area_goal];
            }
          }
        }

        const std::string SERVICE_NAME(SOLVER_NAME_PREFIX + std::to_string(solver_aims.solver) + "/" + ABSTRACT_SERVICE_NAME);
        ros::service::waitForService(SERVICE_NAME);
        if (!ros::service::call(SERVICE_NAME, abstract_srv))
        {
          std::cerr << RED << g_solver_name << ": Error in calling service " << SERVICE_NAME << RESET << std::endl;
          ros_exit();
        }
      }
    }
    else
    {
      const std::string SERVICE_NAME(SOLVER_NAME_PREFIX + std::to_string(MASTER_SOLVER_ID) + "/" + ABSTRACT_SERVICE_NAME);
      ros::service::waitForService(SERVICE_NAME);
      if (!ros::service::call(SERVICE_NAME, abstract_srv))
      {
        std::cerr << RED << g_solver_name << ": Error in calling service " << SERVICE_NAME << RESET << std::endl;
        ros_exit();
      }

      // Wait for abstract plans from the master
      while (ros::ok() && g_num_abstract_req_received < 1)
      {
        rate.sleep();
      }
      g_num_abstract_req_received = 0; // Reset the counter

      // Save the abstract plans made by the master
      for (const auto& next : g_nexts)
      {
        auto areas_map_it = g_round_areas_map.find(0)->second.find(next.area_start);
        Area& area = areas_map_it->second;

        for (const auto& step : next.steps)
        {
          Transit& transit = robot_transit_map.find(step.robot)->second;
          transit.next_areas = std::move(step.areas);
          area.add_robot(step.robot, std::move(transit));
        }
      }
    }
#endif

#ifdef DEBUG_VERBOSE_2
    {
      const auto round_areas_map_it = g_round_areas_map.find(0);
      if (round_areas_map_it == g_round_areas_map.end())
      {
#ifdef DEBUG_VERBOSE_3
        std::cout << g_solver_name << " does not have any area with a robot" << std::endl;
#endif
      }
      else
      {
        std::string debug_str;
        for (const auto& area : round_areas_map_it->second)
        {
          debug_str += g_solver_name + "-a" + std::to_string(area.first) + " abstract plans";
          for (const auto& robot_transit_map_pair : area.second.robot_transit_map)
          {
            debug_str += "\n- r" + std::to_string(robot_transit_map_pair.first) + ":";

#ifdef DEBUG_VERBOSE_3
            debug_str += " (" + std::to_string(robot_transit_map_pair.second.coor_from.x) +
                         "," + std::to_string(robot_transit_map_pair.second.coor_from.y) +
                         ") -> (" + std::to_string(robot_transit_map_pair.second.coor_goal.x) +
                         "," + std::to_string(robot_transit_map_pair.second.coor_goal.y) + "):";
#endif

            if (robot_transit_map_pair.second.next_areas.empty())
            {
              debug_str += " local";
            }
            else
            {
              for (const auto next_area : robot_transit_map_pair.second.next_areas)
              {
                debug_str += " a" + std::to_string(next_area);
              }
            }
          }
        }
        std::cout << debug_str << std::endl;
      }
    }
#endif

#ifdef DEBUG_ABSTRACT
    g_num_track_msg_ignored_1 += 1;   // must use += and before publishing

#ifdef ABSTRACT_UCS_CENTRAL
    if (g_solver_id == MASTER_SOLVER_ID)
    {
      g_time_abstract_total = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::time_point_cast<std::chrono::milliseconds>(time_abstract_total).time_since_epoch()).count();

      print_abstract_stats();

      // Notify the other solvers that the maximum abstract planning time has been determined
      dmapf::Track track_msg;
      track_msg.type = 1;
      track_pub.publish(track_msg);
    }
#else
    if (g_solver_id == MASTER_SOLVER_ID)
    {
      // Wait for abstract planning times from other solvers
      while (ros::ok() && g_num_abstract_req_received < total_solvers - 1)
      {
        rate.sleep();
      }
      g_num_abstract_req_received = 0; // Reset the counter

#ifndef ABSTRACT_UCS
      for (const auto& num_node : num_nodes)
      {
        g_num_nodes[num_node.area] = num_node.var;
      }

      const int num_step_own = steps_num_robots.size();
      if (num_step_own > 0)
      {
        const int num_step_current = g_steps_num_robots.size();

        for (int i = num_step_current; i < num_step_own; ++i)
        {
          g_steps_num_robots.emplace_back(g_steps_num_robots[i - 1]);
        }

        // Include its own steps
        for (int i = 0; i < num_step_own; ++i)
        {
          for (int area = 1; area < g_total_areas + 1; ++area)
          {
            g_steps_num_robots[i][area] += steps_num_robots[i][area];
          }
        }

        // Extend its own steps to the whole vector
        const auto& last = steps_num_robots.back();
        for (int i = num_step_own; i < num_step_current; ++i)
        {
          for (int area = 1; area < g_total_areas + 1; ++area)
          {
            g_steps_num_robots[i][area] += last[area];
          }
        }
      }
#endif

      // Get the maximum abstract planning time from all solvers
      int64_t time_total = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::time_point_cast<std::chrono::milliseconds>(time_abstract_total).time_since_epoch()).count();
      if (time_total > g_time_abstract_total)
      {
        g_time_abstract_total = time_total;
      }

      print_abstract_stats();

      // Notify the other solvers that the maximum abstract planning time has been determined
      dmapf::Track track_msg;
      track_msg.type = 1;
      track_pub.publish(track_msg);
    }
    else
    {
      // Send the plan to the master solver
      dmapf::Abstract abstract_srv;
      abstract_srv.request.type = 2;

#ifdef ABSTRACT_UCS
      abstract_srv.request.steps_num_robots.reserve(g_steps_num_robots.size());
      for (const auto& num_robots : g_steps_num_robots)
#else
      abstract_srv.request.num_nodes = std::move(num_nodes);
      abstract_srv.request.steps_num_robots.reserve(steps_num_robots.size());
      for (const auto& num_robots : steps_num_robots)
#endif
      {
        abstract_srv.request.steps_num_robots.emplace_back(dmapf::Areas{});
        auto& num_robots_req = abstract_srv.request.steps_num_robots.back();
        for (int area = 1; area < num_robots.size(); ++area)
        {
          if (num_robots[area] > 0)
          {
            num_robots_req.areas.emplace_back(dmapf::Area{});
            auto& num_robots_req_areas_back = num_robots_req.areas.back();
            num_robots_req_areas_back.area = area;
            num_robots_req_areas_back.var = num_robots[area];
          }
        }
      }

      abstract_srv.request.time_total = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::time_point_cast<std::chrono::milliseconds>(time_abstract_total).time_since_epoch()).count();

      const std::string SERVICE_NAME(SOLVER_NAME_PREFIX + std::to_string(MASTER_SOLVER_ID) + "/" + ABSTRACT_SERVICE_NAME);
      ros::service::waitForService(SERVICE_NAME);
      if (!ros::service::call(SERVICE_NAME, abstract_srv))
      {
        std::cerr << RED << g_solver_name << ": Error in calling service " << SERVICE_NAME << RESET << std::endl;
        ros_exit();
      }
    }
#endif

    // Wait until the result of abstract planning is shown
    while (ros::ok() && g_num_track_msg_ignored_1 > 0)
    {
      rate.sleep();
    }

    ros_exit();
    return EXIT_SUCCESS;
#endif

#ifdef ABSTRACT_UCS_CENTRAL
    abstract_server.shutdown();

    g_solver_aims.clear();
    g_nexts.clear();
#endif

#ifdef ABSTRACT_UCS
    areas_pub.shutdown();
    areas_sub.shutdown();
#endif

#if defined(ABSTRACT_UCS) || defined(ABSTRACT_UCS_CENTRAL)
    g_steps_num_robots.clear();
    g_num_nodes.clear();
#endif

    // Get areas atoms
    int connected_areas_index = -1;
    int token_index = 0;
    for (const int area : ordered_areas)
    {
      AreaInfo& area_info = g_area_info_map.find(area)->second;

#ifdef SOLVER_ASP
      area_info.node_lp = trim(tokens[++token_index]);
#else
      area_info.instance.num_of_cols = area_info.max_node.x - area_info.min_node.x + 1;
      area_info.instance.num_of_rows = area_info.max_node.y - area_info.min_node.y + 1;
      area_info.instance.map_size = area_info.instance.num_of_cols * area_info.instance.num_of_rows;
      area_info.instance.my_map.resize(area_info.instance.map_size, true);

      std::vector<std::string> node_tokens(tokenizeString(tokens[++token_index], "i(,).\n"));
      for (int i = 0; i < node_tokens.size(); i += 2)
      {
        const int col = std::stoi(node_tokens[i]) - area_info.min_node.x;
        const int row = std::stoi(node_tokens[i + 1]) - area_info.min_node.y;
        area_info.instance.my_map[area_info.instance.linearizeCoordinate(row, col)] = false;
      }
#endif

#ifdef SOLVER_ASP
      area_info.nextto_lp = trim(tokens[++token_index]);
      if (area_info.nextto_lp[0] == '-')
      {
        area_info.nextto_lp.clear();
      }
#else
      ++token_index;
#endif

      const int num_connections = area_info.neighbor_connection_map.size();
      for (int i = 0; i < num_connections; ++i)
      {
        area_info.neighbor_connection_map.find(ordered_connected_areas[++connected_areas_index])->second.link_lp = trim(tokens[++token_index]);
      }
    }

#ifdef DEBUG_VERBOSE_3
    {
      std::string debug_str;
      for (const auto& area_info_pair : g_area_info_map)
      {
#ifdef SOLVER_ASP
        debug_str += std::to_string(area_info_pair.first) + " " + std::to_string(area_info_pair.second.num_nodes) + " " +
                     std::to_string(area_info_pair.second.neighbor_connection_map.size()) + "\n" + area_info_pair.second.node_lp + "\n" + area_info_pair.second.nextto_lp;
#else
        debug_str += std::to_string(area_info_pair.first) + " " + std::to_string(area_info_pair.second.num_nodes) + " " +
                             std::to_string(area_info_pair.second.neighbor_connection_map.size());
#endif
        for (const auto& neighbor_connection_pair : area_info_pair.second.neighbor_connection_map)
        {
          debug_str += "\n" + std::to_string(neighbor_connection_pair.first) + " " + std::to_string(neighbor_connection_pair.second.solver) + " " +
                       std::to_string(neighbor_connection_pair.second.num_links);
          for (const auto& corner_neighbors_pair : area_info_pair.second.corner_neighbors_map)
          {
            for (const int neighbor : corner_neighbors_pair.second)
            {
              if (neighbor == neighbor_connection_pair.first)
              {
                debug_str += " (" + std::to_string(corner_neighbors_pair.first.x) + "," + std::to_string(corner_neighbors_pair.first.y) + ")";
              }
            }
          }
          debug_str += "\n" + neighbor_connection_pair.second.link_lp;
        }
      }
      std::cout << debug_str << std::endl;
    }
#endif
  }

#ifdef DEBUG_TIMES
  t2 = std::chrono::steady_clock::now();
  std::cout << MAGENTA << "Time initializing = "
            << format_time(std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()) << RESET << std::endl;
#endif

#ifdef DEBUG_TIMES
  t1 = std::chrono::steady_clock::now();
#endif

  // Make sure all solvers have subscribed to Track topic
  wait_for_subscribers(track_pub, total_solvers);

#ifdef DEBUG_TIMES
  t2 = std::chrono::steady_clock::now();
  std::cout << MAGENTA << "Time waiting for subscribers = "
            << format_time(std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()) << RESET << std::endl;
#endif

  // A barrier to determine a set of active solvers and their task
  {
    dmapf::Track track_msg;
    track_msg.type = 0;

    const auto round_areas_map_it = g_round_areas_map.find(0);
    if (round_areas_map_it == g_round_areas_map.end())
    {
      track_msg.active = false;
    }
    else
    {
      track_msg.active = true;
      track_msg.solver = g_solver_id;
      for (const auto& area : round_areas_map_it->second)
      {
        const AreaInfo& area_info = area.second.area_info;
        for (const auto& neighbor_tier_robots_pair : area.second.neighbor_tier_robots_map)
        {
          dmapf::Cross cross;
          cross.area_from = area.first;
          cross.area_to = neighbor_tier_robots_pair.first;
          cross.solver_to = area_info.neighbor_connection_map.find(neighbor_tier_robots_pair.first)->second.solver;
          track_msg.crosses.emplace_back(std::move(cross));
        }
      }
    }

    g_num_track_msg_expected += total_solvers;   // must use += and before publishing
    track_pub.publish(track_msg);

    while (ros::ok() && g_num_track_msg_expected > 0)   // must use > 0
    {
      rate.sleep();
    }
  }

  if (g_active_solvers_set.empty())
  {
    std::cout << YELLOW << g_solver_name << ": There is no robot" << RESET << std::endl;
    return EXIT_SUCCESS;
  }

  // Construct the initial task and add any missing areas that will be migrating into
  Task task{};
  for (const auto& cross : g_crosses)
  {
    task.add_task(cross);

    if (g_solver_id != cross.solver_from || cross.solver_from == cross.solver_to)   // Receiver needs to make sure the area has been initialized
    {
      // Create the receiving area if it has not been created
      auto round_areas_map_it = g_round_areas_map.find(0);
      if (round_areas_map_it == g_round_areas_map.end())
      {
        round_areas_map_it = g_round_areas_map.emplace(0, std::unordered_map<int, Area>{}).first;
      }
      std::unordered_map<int, Area>& areas_map = round_areas_map_it->second;

      if (areas_map.find(cross.area_to) == areas_map.end())
      {
        auto areas_map_it = areas_map.find(cross.area_to);
        if (areas_map_it == areas_map.end())
        {
          areas_map_it = areas_map.emplace(cross.area_to, Area(g_area_info_map.find(cross.area_to)->second)).first;
        }
        Area& area = areas_map_it->second;
        area_latest_round_map.emplace(cross.area_to, 0);

        initialize_area(area);
      }
    }
  }

  // Another barrier to ensure that all the active solvers have the areas ready
  {
    dmapf::Track track_msg;
    track_msg.type = 1;

    g_num_track_msg_ignored_1 += g_active_solvers_set.size();   // must use += and before publishing
    if (g_active_solvers_set.find(g_solver_id) != g_active_solvers_set.end())
    {
      track_pub.publish(track_msg);
    }

    while (ros::ok() && g_num_track_msg_ignored_1 > 0)   // must use > 0
    {
      rate.sleep();
    }
  }

  // Interleaved planning
  while (ros::ok())
  {
#ifdef DEBUG_VERBOSE_1
    {
      std::string debug_str("Round " + std::to_string(g_current_round));

#ifdef DEBUG_VERBOSE_2
      const auto round_areas_map_it = g_round_areas_map.find(g_current_round);
      if (round_areas_map_it != g_round_areas_map.end())
      {
        for (const auto& area_pair : round_areas_map_it->second)
        {
          debug_str += "\n- a" + std::to_string(area_pair.first) + ":";

          const Area& area = area_pair.second;
          for (const auto& robot_transit_pair : area.robot_transit_map)
          {
            debug_str += " r" + std::to_string(robot_transit_pair.first);
          }
        }
      }

      debug_str += "\n- active_solvers:";
      for (const int solver_id : g_active_solvers_set)
      {
        debug_str += " s" + std::to_string(solver_id);
      }

      debug_str += "\n- send_areas:";
      for (const auto& send_areas_pair : task.send_areas_map)
      {
        for (const auto& neighbor_solver_pair : send_areas_pair.second)
        {
          debug_str += " (a" + std::to_string(send_areas_pair.first) +
                       ",s" + std::to_string(neighbor_solver_pair.second) + "-a" + std::to_string(neighbor_solver_pair.first) + ")";
        }
      }

      debug_str += "\n- recv_areas:";
      for (const auto& recv_areas_pair : task.recv_areas_map)
      {
        const auto& neighbor_connection_map = round_areas_map_it->second.find(recv_areas_pair.first)->second.area_info.neighbor_connection_map;
        for (const int neighbor : recv_areas_pair.second)
        {
          debug_str += " (s" + std::to_string(neighbor_connection_map.find(neighbor)->second.solver) + "-a" + std::to_string(neighbor) +
                       ",a" + std::to_string(recv_areas_pair.first) + ")";
        }
      }
#endif
      std::cout << g_solver_name << ": " << debug_str  << std::endl;
    }
#else
    if (g_solver_id == MASTER_SOLVER_ID)
    {
      std::cout << "Round " << g_current_round << std::endl;
    }
#endif

    if (g_active_solvers_set.find(g_solver_id) != g_active_solvers_set.end())
    {
      std::unordered_map<int, std::vector<Migrate>> area_emigrates_map;   // Area -> [(robot, from, distance)]
      std::unordered_map<int, std::vector<Migrate>> area_immigrates_map;  // Area -> Area -> [(robot, to, distance)]

#ifdef DEBUG_VERBOSE_3
      std::cout << g_solver_name << ": Phase 1 at round " << g_current_round << std::endl;
#endif

      /* Phase 1: Negotiate migration with higher-ranked neighbors */
      for (const auto& area_from_to_pair : task.send_areas_map)
      {
        const int area_from = area_from_to_pair.first;
        for (const auto& area_to_solver_pair : area_from_to_pair.second)
        {
          const int area_to = area_to_solver_pair.first;
          const int solver_to = area_to_solver_pair.second;
          const Area& area = g_round_areas_map.find(g_current_round)->second.find(area_from)->second;
          const AreaInfo& area_info = area.area_info;

          // Construct the migration request message
          dmapf::Migrate::Request migrate_req;
          migrate_req.type = 0;
          migrate_req.area_from = area_from;
          migrate_req.area_to = area_to;

          const auto neighbor_tier_robots_map_it = area.neighbor_tier_robots_map.find(area_to);
          if (neighbor_tier_robots_map_it != area.neighbor_tier_robots_map.end()) // If there are robots that want to migrate
          {
            const Connection& connection = area_info.neighbor_connection_map.find(area_to)->second;
            int num_robots = 0;
            for (const auto& tier_robots_pair : neighbor_tier_robots_map_it->second)
            {
              migrate_req.tiers.emplace_back(dmapf::Tier{});
              dmapf::Tier& tier = migrate_req.tiers.back();
              tier.rank = tier_robots_pair.first;

              for (const int robot : tier_robots_pair.second)
              {
                const Coor& coor = area.robot_transit_map.find(robot)->second.coor_from;
                dmapf::Position position;
                position.robot = robot;
                position.coor.x = coor.x;
                position.coor.y = coor.y;
                tier.positions.emplace_back(std::move(position));
              }

              // Break early if the number of robots in the message is already more than the number of links between the areas
              num_robots += tier_robots_pair.second.size();
              if (num_robots >= connection.num_links)
              {
                break;
              }
            }
          }

          // Send the migrate request for negotiation
          dmapf::Migrate::Response migrate_res;
          const std::string service_name(SOLVER_NAME_PREFIX + std::to_string(solver_to) + "/" + MIGRATE_SERVICE_NAME);
          ros::service::waitForService(service_name);
          if (!ros::service::call(service_name, migrate_req, migrate_res))
          {
            std::cerr << RED << g_solver_name << "-a" << area_from << ": Error in calling service " << service_name
                      << " in phase 1 at round " << g_current_round << RESET << std::endl;
            ros_exit();
            return EXIT_FAILURE;
          }

          // Analyze the returned results from the negotiation
          if (!migrate_res.dists.empty())
          {
            auto area_emigrates_map_it = area_emigrates_map.find(area_from);
            if (area_emigrates_map_it == area_emigrates_map.end())
            {
              area_emigrates_map_it = area_emigrates_map.emplace(area_from, std::vector<Migrate>{}).first;
            }
            std::vector<Migrate>& emigrates = area_emigrates_map_it->second;

            auto area_immigrates_map_it = area_immigrates_map.find(area_from);
            if (area_immigrates_map_it == area_immigrates_map.end())
            {
              area_immigrates_map_it = area_immigrates_map.emplace(area_from, std::vector<Migrate>{}).first;
            }
            std::vector<Migrate>& immigrates = area_immigrates_map_it->second;

            for (const dmapf::Dist& dist : migrate_res.dists)
            {
              // If it's an existing robot, then it's going out
              if (area.robot_transit_map.find(dist.robot) != area.robot_transit_map.end())
              {
                emigrates.emplace_back(Migrate{dist.robot, Coor(dist.coor_from.x, dist.coor_from.y), Coor(dist.coor_to.x, dist.coor_to.y), dist.rank});
              }
              else  // Otherwise the robot is coming in
              {
                immigrates.emplace_back(Migrate{dist.robot, Coor(dist.coor_from.x, dist.coor_from.y), Coor(dist.coor_to.x, dist.coor_to.y), dist.rank});
              }
            }
          }
        }
      }

      // Wait until all negotiation requests from lower solvers have been received
      while (ros::ok() && g_num_migrate_negotiate_req_received < task.num_recv_areas)
      {
        rate.sleep();
      }
      g_num_migrate_negotiate_req_received = 0; // Reset the counter

#ifdef DEBUG_VERBOSE_3
      std::cout << g_solver_name << ": Phase 2 at round " << g_current_round << std::endl;
#endif

      /* Phase 2: Reject previously accepted robots from higher-ranked neighbors if they conflict with the accepted robots from lower-ranked neighbors */
      // Check for conflicted "out" border nodes
      // area -> border_from -> (robot, border_to, rank)
      std::unordered_map<int, std::unordered_map<Coor, Emigrate, CoorHasher>> area_border_emigrate_map;
      for (const auto& area_emigrates_pair : g_area_emigrates_map)
      {
        auto area_border_emigrate_it = area_border_emigrate_map.find(area_emigrates_pair.first);
        if (area_border_emigrate_it == area_border_emigrate_map.end())
        {
          area_border_emigrate_it = area_border_emigrate_map.emplace(area_emigrates_pair.first, std::unordered_map<Coor, Emigrate, CoorHasher>{}).first;
        }
        for(const auto& emigrate : area_emigrates_pair.second)
        {
          auto border_emigrate_map_it = area_border_emigrate_it->second.find(emigrate.border_from);
          if (border_emigrate_map_it == area_border_emigrate_it->second.end())
          {
            area_border_emigrate_it->second.emplace(emigrate.border_from, Emigrate{emigrate.robot, emigrate.border_to, emigrate.rank});
          }
          else  // There should be no conflict here
          {
            std::cout << YELLOW << g_solver_name << "-a" << area_emigrates_pair.first
                      << " has a conflict in emigrating robots from lower-ranked solvers" << RESET << std::endl;

            auto area_canceled_robots_map_it = g_area_canceled_robots_map.find(area_emigrates_pair.first);
            if (area_canceled_robots_map_it == g_area_canceled_robots_map.end())
            {
              area_canceled_robots_map_it = g_area_canceled_robots_map.emplace(area_emigrates_pair.first, std::unordered_set<int>{}).first;
            }

            if (emigrate.rank > border_emigrate_map_it->second.rank)
            {
              area_canceled_robots_map_it->second.insert(border_emigrate_map_it->second.robot);
              border_emigrate_map_it->second.robot = emigrate.robot;
              border_emigrate_map_it->second.border_to = emigrate.border_to;
              border_emigrate_map_it->second.rank = emigrate.rank;
            }
            else
            {
              area_canceled_robots_map_it->second.insert(emigrate.robot);
            }
          }
        }
      }
      for (const auto& area_emigrates_pair : area_emigrates_map)
      {
        auto area_border_emigrate_it = area_border_emigrate_map.find(area_emigrates_pair.first);
        if (area_border_emigrate_it == area_border_emigrate_map.end())
        {
          area_border_emigrate_it = area_border_emigrate_map.emplace(area_emigrates_pair.first, std::unordered_map<Coor, Emigrate, CoorHasher>{}).first;
        }
        for (const auto& emigrate : area_emigrates_pair.second)
        {
          auto border_emigrate_map_it = area_border_emigrate_it->second.find(emigrate.border_from);
          if (border_emigrate_map_it == area_border_emigrate_it->second.end())
          {
            area_border_emigrate_it->second.emplace(emigrate.border_from, Emigrate{emigrate.robot, emigrate.border_to, emigrate.rank});
          }
          else  // There can be conflicts here
          {
            auto area_canceled_robots_map_it = g_area_canceled_robots_map.find(area_emigrates_pair.first);
            if (area_canceled_robots_map_it == g_area_canceled_robots_map.end())
            {
              area_canceled_robots_map_it = g_area_canceled_robots_map.emplace(area_emigrates_pair.first, std::unordered_set<int>{}).first;
            }

            if (emigrate.rank > border_emigrate_map_it->second.rank)
            {
              area_canceled_robots_map_it->second.insert(border_emigrate_map_it->second.robot);
              border_emigrate_map_it->second.robot = emigrate.robot;
              border_emigrate_map_it->second.border_to = emigrate.border_to;
              border_emigrate_map_it->second.rank = emigrate.rank;
            }
            else
            {
              area_canceled_robots_map_it->second.insert(emigrate.robot);
            }
          }
        }
      }

      // Check for conflicted "in" border nodes
      // area -> border_to -> (robot, rank)
      std::unordered_map<int, std::unordered_map<Coor, Immigrate, CoorHasher>> area_border_immigrate_map;
      for (const auto& area_immigrates_pair : g_area_immigrates_map)
      {
        auto area_border_immigrate_it = area_border_immigrate_map.find(area_immigrates_pair.first);
        if (area_border_immigrate_it == area_border_immigrate_map.end())
        {
          area_border_immigrate_it = area_border_immigrate_map.emplace(area_immigrates_pair.first, std::unordered_map<Coor, Immigrate, CoorHasher>{}).first;
        }
        for(const auto& immigrate : area_immigrates_pair.second)
        {
          auto border_immigrate_map_it = area_border_immigrate_it->second.find(immigrate.border_to);
          if (border_immigrate_map_it == area_border_immigrate_it->second.end())
          {
            area_border_immigrate_it->second.emplace(immigrate.border_to, Immigrate{immigrate.robot, immigrate.rank});
          }
          else  // There should be no conflict here
          {
            std::cout << YELLOW << g_solver_name << "-a" << area_immigrates_pair.first
                      << " has a conflict in immigrating robots from lower-ranked solvers" << RESET << std::endl;

            auto area_rejected_robots_map_it = g_area_rejected_robots_map.find(area_immigrates_pair.first);
            if (area_rejected_robots_map_it == g_area_rejected_robots_map.end())
            {
              area_rejected_robots_map_it = g_area_rejected_robots_map.emplace(area_immigrates_pair.first, std::unordered_set<int>{}).first;
            }

            if (immigrate.rank > border_immigrate_map_it->second.rank)
            {
              area_rejected_robots_map_it->second.insert(border_immigrate_map_it->second.robot);
              border_immigrate_map_it->second.robot = immigrate.robot;
              border_immigrate_map_it->second.rank = immigrate.rank;
            }
            else
            {
              area_rejected_robots_map_it->second.insert(immigrate.robot);
            }
          }
        }
      }
      for (const auto& area_immigrates_pair : area_immigrates_map)
      {
        auto area_border_immigrate_it = area_border_immigrate_map.find(area_immigrates_pair.first);
        if (area_border_immigrate_it == area_border_immigrate_map.end())
        {
          area_border_immigrate_it = area_border_immigrate_map.emplace(area_immigrates_pair.first, std::unordered_map<Coor, Immigrate, CoorHasher>{}).first;
        }
        for (const auto& immigrate : area_immigrates_pair.second)
        {
          auto border_immigrate_map_it = area_border_immigrate_it->second.find(immigrate.border_to);
          if (border_immigrate_map_it == area_border_immigrate_it->second.end())
          {
            area_border_immigrate_it->second.emplace(immigrate.border_to, Immigrate{immigrate.robot, immigrate.rank});
          }
          else  // There can be conflicts here
          {
            auto area_rejected_robots_map_it = g_area_rejected_robots_map.find(area_immigrates_pair.first);
            if (area_rejected_robots_map_it == g_area_rejected_robots_map.end())
            {
              area_rejected_robots_map_it = g_area_rejected_robots_map.emplace(area_immigrates_pair.first, std::unordered_set<int>{}).first;
            }

            if (immigrate.rank > border_immigrate_map_it->second.rank)
            {
              area_rejected_robots_map_it->second.insert(border_immigrate_map_it->second.robot);
              border_immigrate_map_it->second.robot = immigrate.robot;
              border_immigrate_map_it->second.rank = immigrate.rank;
            }
            else
            {
              area_rejected_robots_map_it->second.insert(immigrate.robot);
            }
          }
        }
      }

      // Regulate the number of robots in the areas
      for (const auto& area : g_round_areas_map.find(g_current_round)->second)
      {
        const int n = area.second.area_info.num_nodes;
        const int r = area.second.robot_transit_map.size();

        // We can only do something if there are robots coming in
        const auto area_border_immigrate_map_it = area_border_immigrate_map.find(area.first);
        if (area_border_immigrate_map_it != area_border_immigrate_map.end())
        {
          int i = area_border_immigrate_map_it->second.size();

          // Choose more incoming robots to reject if they overrun the minimum free space required
          const int num_nodes_overrun = MIN_NUM_FREE_NODES - (n - r - i);
          if (num_nodes_overrun > 0)
          {
            if (i > num_nodes_overrun)  // Choose robots with longer remaining abstract steps to migrate
            {
              // Copy the set of incoming robots into a vector
              std::vector<Immigrate> immigrates;
              immigrates.reserve(area_border_immigrate_map_it->second.size());
              for (const auto& border_immigrate_pair : area_border_immigrate_map_it->second)
              {
                immigrates.emplace_back(Immigrate{border_immigrate_pair.second.robot, border_immigrate_pair.second.rank});
              }

              // Sort the incoming robots with the shortest number of remaining abstract steps first
              std::sort(immigrates.begin(), immigrates.end(), Immigrate::compare_ascend);

              // Add the excess robot to the set of rejected robots
              auto area_rejected_robots_map_it = g_area_rejected_robots_map.find(area.first);
              if (area_rejected_robots_map_it == g_area_rejected_robots_map.end())
              {
                area_rejected_robots_map_it = g_area_rejected_robots_map.emplace(area.first, std::unordered_set<int>{}).first;
              }
              for (int j = 0; j < num_nodes_overrun; ++j)
              {
                area_rejected_robots_map_it->second.insert(immigrates[j].robot);
              }
            }
            else if (i == num_nodes_overrun)  // Reject all incoming robots - this may cause an infinite loop
            {
              auto area_rejected_robots_map_it = g_area_rejected_robots_map.find(area.first);
              if (area_rejected_robots_map_it == g_area_rejected_robots_map.end())
              {
                area_rejected_robots_map_it = g_area_rejected_robots_map.emplace(area.first, std::unordered_set<int>{}).first;
              }
              area_rejected_robots_map_it->second.reserve(i);
              for (const auto& border_immigrate_pair : area_border_immigrate_map_it->second)
              {
                area_rejected_robots_map_it->second.insert(border_immigrate_pair.second.robot);
              }
              area_border_immigrate_map_it->second.clear();
            }
            else  // Cannot resolve the overrun
            {
              std::cerr << RED << g_solver_name << "-a" << area.first << " is overrun"
                        << " - n = " << n << ", r = " << r << ", i = " << i << ", overruns = " << num_nodes_overrun << std::endl
                        << "Consider increasing the size of areas" << RESET << std::endl;
              ros_exit();
              return EXIT_FAILURE;
            }
          }
        }
      }

      // A barrier to ensure that all the active solvers have finished deciding which robots to reject
      {
        dmapf::Track track_msg;
        track_msg.type = 2;

        g_num_track_msg_ignored_2 += g_active_solvers_set.size();   // must use += and before publishing
        track_pub.publish(track_msg);

        while (ros::ok() && g_num_track_msg_ignored_2 > 0)   // must use > 0
        {
          rate.sleep();
        }
      }

      // Send the migrate request for canceled/rejected robots
      std::unordered_map<int, std::vector<int>> area_canceled_robots_map_res;
      std::unordered_map<int, std::vector<int>> area_rejected_robots_map_res;
      for (const auto& area_from_to_pair : task.send_areas_map)
      {
        const int area_from = area_from_to_pair.first;
        for (const auto& area_to_solver_pair : area_from_to_pair.second)
        {
          const int area_to = area_to_solver_pair.first;
          const int solver_to = area_to_solver_pair.second;

          // Construct the migration request message
          dmapf::Migrate::Request migrate_req;
          migrate_req.type = 1;
          migrate_req.area_from = area_from;
          migrate_req.area_to = area_to;

          const auto area_canceled_robots_map_it = g_area_canceled_robots_map.find(area_from);
          if (area_canceled_robots_map_it != g_area_canceled_robots_map.end())
          {
#ifdef DEBUG_VERBOSE_3
            std::string msg;
            for (const int canceled_robot : area_canceled_robots_map_it->second)
            {
              msg += " " + std::to_string(canceled_robot);
            }
            std::cout << MAGENTA << g_solver_name + "-a" + std::to_string(area_from) + " requests to cancel robots" << msg << RESET << std::endl;
#endif

            std::copy(area_canceled_robots_map_it->second.begin(), area_canceled_robots_map_it->second.end(), std::back_inserter(migrate_req.canceled_robots));
          }

          const auto area_rejected_robots_map_it = g_area_rejected_robots_map.find(area_from);
          if (area_rejected_robots_map_it != g_area_rejected_robots_map.end())
          {
#ifdef DEBUG_VERBOSE_3
            std::string msg;
            for (const int rejected_robot : area_rejected_robots_map_it->second)
            {
              msg += " " + std::to_string(rejected_robot);
            }
            std::cout << CYAN << g_solver_name + "-a" + std::to_string(area_from) + " requests to reject robots" << msg << RESET << std::endl;
#endif

            std::copy(area_rejected_robots_map_it->second.begin(), area_rejected_robots_map_it->second.end(), std::back_inserter(migrate_req.rejected_robots));
          }

          // Send the migrate request for rejection
          dmapf::Migrate::Response migrate_res;
          const std::string service_name(SOLVER_NAME_PREFIX + std::to_string(solver_to) + "/" + MIGRATE_SERVICE_NAME);
          ros::service::waitForService(service_name);
          if (!ros::service::call(service_name, migrate_req, migrate_res))
          {
            std::cerr << RED << g_solver_name << ": Error in calling service " << service_name << " from area " << area_from
                      << " in phase 2 at round " << g_current_round << RESET << std::endl;
            ros_exit();
            return EXIT_FAILURE;
          }

          // Save the response
          auto area_canceled_robots_map_res_it = area_canceled_robots_map_res.find(area_from);
          if (area_canceled_robots_map_res_it == area_canceled_robots_map_res.end())
          {
            area_canceled_robots_map_res_it = area_canceled_robots_map_res.emplace(area_from, std::vector<int>{}).first;
          }
          area_canceled_robots_map_res_it->second.insert(area_canceled_robots_map_res_it->second.end(),
                                                         migrate_res.rejected_robots.begin(), migrate_res.rejected_robots.end());

          auto area_rejected_robots_map_res_it = area_rejected_robots_map_res.find(area_from);
          if (area_rejected_robots_map_res_it == area_rejected_robots_map_res.end())
          {
            area_rejected_robots_map_res_it = area_rejected_robots_map_res.emplace(area_from, std::vector<int>{}).first;
          }
          area_rejected_robots_map_res_it->second.insert(area_rejected_robots_map_res_it->second.end(),
                                                         migrate_res.canceled_robots.begin(), migrate_res.canceled_robots.end());
        }
      }

      // Wait until all rejection requests from lower solvers have been received
      while (ros::ok() && g_num_migrate_reject_req_received < task.num_recv_areas)
      {
        rate.sleep();
      }
      g_num_migrate_reject_req_received = 0; // Reset the counter

      // Combine the canceled robots into one set
      for (const auto& area_canceled_robots_map_req_pair : g_area_canceled_robots_map_req)
      {
        auto area_canceled_robots_map_it = g_area_canceled_robots_map.find(area_canceled_robots_map_req_pair.first);
        if (area_canceled_robots_map_it == g_area_canceled_robots_map.end())
        {
          area_canceled_robots_map_it = g_area_canceled_robots_map.emplace(area_canceled_robots_map_req_pair.first, std::unordered_set<int>{}).first;
        }
        std::copy(area_canceled_robots_map_req_pair.second.begin(), area_canceled_robots_map_req_pair.second.end(),
                  std::inserter(area_canceled_robots_map_it->second, area_canceled_robots_map_it->second.begin()));
      }
      for (const auto& area_canceled_robots_map_res_pair : area_canceled_robots_map_res)
      {
        auto area_canceled_robots_map_it = g_area_canceled_robots_map.find(area_canceled_robots_map_res_pair.first);
        if (area_canceled_robots_map_it == g_area_canceled_robots_map.end())
        {
          area_canceled_robots_map_it = g_area_canceled_robots_map.emplace(area_canceled_robots_map_res_pair.first, std::unordered_set<int>{}).first;
        }
        std::copy(area_canceled_robots_map_res_pair.second.begin(), area_canceled_robots_map_res_pair.second.end(),
                  std::inserter(area_canceled_robots_map_it->second, area_canceled_robots_map_it->second.begin()));
      }

      // Combine the rejected robots into one set
      for (const auto& area_rejected_robots_map_req_pair : g_area_rejected_robots_map_req)
      {
        auto area_rejected_robots_map_it = g_area_rejected_robots_map.find(area_rejected_robots_map_req_pair.first);
        if (area_rejected_robots_map_it == g_area_rejected_robots_map.end())
        {
          area_rejected_robots_map_it = g_area_rejected_robots_map.emplace(area_rejected_robots_map_req_pair.first, std::unordered_set<int>{}).first;
        }
        std::copy(area_rejected_robots_map_req_pair.second.begin(), area_rejected_robots_map_req_pair.second.end(),
                  std::inserter(area_rejected_robots_map_it->second, area_rejected_robots_map_it->second.begin()));
      }
      for (const auto& area_rejected_robots_map_res_pair : area_rejected_robots_map_res)
      {
        auto area_rejected_robots_map_it = g_area_rejected_robots_map.find(area_rejected_robots_map_res_pair.first);
        if (area_rejected_robots_map_it == g_area_rejected_robots_map.end())
        {
          area_rejected_robots_map_it = g_area_rejected_robots_map.emplace(area_rejected_robots_map_res_pair.first, std::unordered_set<int>{}).first;
        }
        std::copy(area_rejected_robots_map_res_pair.second.begin(), area_rejected_robots_map_res_pair.second.end(),
                  std::inserter(area_rejected_robots_map_it->second, area_rejected_robots_map_it->second.begin()));
      }


#ifdef DEBUG_VERBOSE_3
      std::cout << g_solver_name << " is making movement plans at round " << g_current_round << std::endl;
#endif

      /* Make Movement Plan */
      for (auto& area : g_round_areas_map.find(g_current_round)->second)
      {
        if (area.second.robot_transit_map.empty())  // Skip this area if there is no robot
        {
#ifdef DEBUG_VERBOSE_2
          std::cout << YELLOW << g_solver_name << "-a" << area.first << " contains no robot at round "
                    << g_current_round << RESET << std::endl;
#endif
          continue;
        }

        AreaInfo& area_info = area.second.area_info;
#ifdef SOLVER_ASP
        std::string outsider;
        std::string init;
        std::string goal;
        std::string in;
#else
        Instance& instance = area_info.instance;
        instance.num_of_agents = area.second.robot_transit_map.size();
        instance.start_locations.resize(instance.num_of_agents);
        instance.goal_locations.clear();
        instance.goal_locations.resize(instance.num_of_agents, -1);
        instance.avoid_locations.clear();

        int agent_id = -1;
        std::vector<int> robot_ids(instance.num_of_agents);
        std::unordered_map<int, int> robot_agent_id_map(instance.num_of_agents);
#endif

        /* Initialize avoid locations */
        bool has_incoming_robots = false;
        const auto area_border_immigrate_map_it = area_border_immigrate_map.find(area.first);
        if (area_border_immigrate_map_it != area_border_immigrate_map.end())
        {
          const auto area_rejected_robots_it = g_area_rejected_robots_map.find(area.first);
          if (area_rejected_robots_it == g_area_rejected_robots_map.end())
          {
            if (!area_border_immigrate_map_it->second.empty())
            {
              has_incoming_robots = true;
              for (const auto& border_immigrate_pair : area_border_immigrate_map_it->second)
              {
#ifdef SOLVER_ASP
                in += "c((" + std::to_string(border_immigrate_pair.first.x) + "," + std::to_string(border_immigrate_pair.first.y) + ")).";
#else
                const int avoid_location = instance.linearizeCoordinate(border_immigrate_pair.first.y - area_info.min_node.y,
                                                                        border_immigrate_pair.first.x - area_info.min_node.x);
                instance.avoid_locations.insert(avoid_location);
#endif
              }
            }
          }
          else  // Do not include rejected robot in the constraint for incoming robots
          {
            for (const auto& border_immigrate_pair : area_border_immigrate_map_it->second)
            {
              if (area_rejected_robots_it->second.find(border_immigrate_pair.second.robot) == area_rejected_robots_it->second.end())
              {
                has_incoming_robots = true;
#ifdef SOLVER_ASP
                in += "c((" + std::to_string(border_immigrate_pair.first.x) + "," + std::to_string(border_immigrate_pair.first.y) + ")).";
#else
                const int avoid_location = instance.linearizeCoordinate(border_immigrate_pair.first.y - area_info.min_node.y,
                                                                        border_immigrate_pair.first.x - area_info.min_node.x);
                instance.avoid_locations.insert(avoid_location);
#endif
              }
#ifdef DEBUG_VERBOSE_3
              else
              {
                std::cout << MAGENTA << g_solver_name << "-a" << area.first << " rejects robot " << border_immigrate_pair.second.robot << RESET << std::endl;
              }
#endif
            }
          }
        }

        /* Initialize goal locations */
        bool last_round = false;
        std::vector<Migrate> emigrates;
        const auto area_border_emigrate_map_it = area_border_emigrate_map.find(area.first);

        // There is some robot that wants to go out
        if (area_border_emigrate_map_it != area_border_emigrate_map.end() && !area_border_emigrate_map_it->second.empty())
        {
          const auto area_canceled_robots_it = g_area_canceled_robots_map.find(area.first);
          if (area_canceled_robots_it == g_area_canceled_robots_map.end())
          {
            emigrates.reserve(area_border_emigrate_map_it->second.size());
            for (const auto& border_emigrate_pair : area_border_emigrate_map_it->second)
            {
              emigrates.emplace_back(Migrate{border_emigrate_pair.second.robot,
                                             border_emigrate_pair.first,
                                             border_emigrate_pair.second.border_to,
                                             border_emigrate_pair.second.rank});
            }
          }
          else  // Do not include canceled robots in the list of outgoing robots
          {
            for (const auto& border_emigrate_pair : area_border_emigrate_map_it->second)
            {
              if (area_canceled_robots_it->second.find(border_emigrate_pair.second.robot) == area_canceled_robots_it->second.end())
              {
                emigrates.emplace_back(Migrate{border_emigrate_pair.second.robot,
                                               border_emigrate_pair.first,
                                               border_emigrate_pair.second.border_to,
                                               border_emigrate_pair.second.rank});
              }
#ifdef DEBUG_VERBOSE_3
              else
              {
                std::cout << CYAN << g_solver_name << "-a" << area.first << " cancels robot " << border_emigrate_pair.second.robot << RESET << std::endl;
              }
#endif
            }
          }

          // Skip this area if there is no assigned robot left after rejection
          // - Cannot skip because there may be robots that need to migrate in immediately
//          if (emigrates.empty())
//          {
//            continue;
//          }
        }
        else if (!has_incoming_robots && area.second.neighbor_tier_robots_map.empty())
        {
          last_round = true;  // This may be the last round of this area
        }

        if (last_round)
        {
          for (const auto& robot_transit_pair : area.second.robot_transit_map)
          {
#ifdef SOLVER_ASP
            goal += "g(r(" + std::to_string(robot_transit_pair.first) + "),(" +
                std::to_string(robot_transit_pair.second.coor_goal.x) + "," + std::to_string(robot_transit_pair.second.coor_goal.y) + ")).";
#else
            ++agent_id;
            robot_ids[agent_id] = robot_transit_pair.first;
            robot_agent_id_map.emplace(robot_transit_pair.first, agent_id);
            instance.goal_locations[agent_id] = instance.linearizeCoordinate(robot_transit_pair.second.coor_goal.y - area_info.min_node.y,
                                                                             robot_transit_pair.second.coor_goal.x - area_info.min_node.x);
#endif
          }
        }
        else
        {
          for (const auto& emigrate : emigrates)
          {
#ifdef SOLVER_ASP
            goal += "g(r(" + std::to_string(emigrate.robot) + "),(" +
                std::to_string(emigrate.border_from.x) + "," + std::to_string(emigrate.border_from.y) + ")).";
#else
            ++agent_id;
            robot_ids[agent_id] = emigrate.robot;
            robot_agent_id_map.emplace(emigrate.robot, agent_id);
            instance.goal_locations[agent_id] = instance.linearizeCoordinate(emigrate.border_from.y - area_info.min_node.y,
                                                                             emigrate.border_from.x - area_info.min_node.x);
#endif
          }

#ifndef SOLVER_ASP
          if (emigrates.size() < instance.num_of_agents)
          {
            for (const auto& robot_transit_pair : area.second.robot_transit_map)
            {
              if (robot_agent_id_map.find(robot_transit_pair.first) == robot_agent_id_map.end())
              {
                ++agent_id;
                robot_ids[agent_id] = robot_transit_pair.first;
                robot_agent_id_map.emplace(robot_transit_pair.first, agent_id);
//                instance.goal_locations[agent_id] = -1;   // Unnecessary because it was already initialized to -1
              }
            }
          }
#endif
        }

        /* Initialize start locations */
        int first_timestep = 0;
        for (const auto& robot_transit_pair : area.second.robot_transit_map)
        {
#ifdef SOLVER_ASP
          init += "r(" + std::to_string(robot_transit_pair.first) + ").";
#else
          instance.start_locations[robot_agent_id_map.find(robot_transit_pair.first)->second] =
              instance.linearizeCoordinate(robot_transit_pair.second.coor_from.y - area_info.min_node.y,
                                           robot_transit_pair.second.coor_from.x - area_info.min_node.x);
#endif

          if (robot_transit_pair.second.direction.is_move())
          {
#ifdef SOLVER_ASP
            init += "a(" + std::to_string(robot_transit_pair.first) + ",(" +
                    std::to_string(robot_transit_pair.second.coor_from.x) + "," + std::to_string(robot_transit_pair.second.coor_from.y) + "),1).";
#endif
            first_timestep = 1;   // Start making a movement plan for all robots at time step 1

            // Force the robot to migrate in at the first time step
            auto robot_moves_last_pair_map_it = area.second.robot_moves_last_pair_map.find(robot_transit_pair.first);
            if (robot_moves_last_pair_map_it == area.second.robot_moves_last_pair_map.end())
            {
              robot_moves_last_pair_map_it = area.second.robot_moves_last_pair_map.emplace
                  (robot_transit_pair.first, std::pair<std::vector<Move>, Coor>{{}, robot_transit_pair.second.coor_from}).first;
            }
            std::pair<std::vector<Move>, Coor>& moves_last_pair = robot_moves_last_pair_map_it->second;
            moves_last_pair.first.emplace_back(Move{robot_transit_pair.second.direction, 1});

#ifdef DEBUG_VERBOSE_1
#ifdef SOLVER_ASP
            outsider += "a(" + std::to_string(robot_transit_pair.first) + ",(" +
                        std::to_string(robot_transit_pair.second.coor_from.x - robot_transit_pair.second.direction.x) + "," +
                        std::to_string(robot_transit_pair.second.coor_from.y - robot_transit_pair.second.direction.y) + "),0).";
            outsider += "m(" + std::to_string(robot_transit_pair.first) + ",(" +
                        std::to_string(robot_transit_pair.second.direction.x) + "," +
                        std::to_string(robot_transit_pair.second.direction.y) + "),1).";
#endif
#endif
          }
          else
          {
#ifdef SOLVER_ASP
            init += "a(" + std::to_string(robot_transit_pair.first) + ",(" +
                    std::to_string(robot_transit_pair.second.coor_from.x) + "," + std::to_string(robot_transit_pair.second.coor_from.y) + "),0).";
#endif
          }
        }

        if (!has_incoming_robots && !last_round && emigrates.empty())
        {
#ifdef DEBUG_VERBOSE_1
          std::cout << g_solver_name << "-a" << area.first << " skips movement planing" << std::endl;
#endif
          continue;
        }

#ifndef SOLVER_ASP
#ifdef DEBUG_VERBOSE_1
        std::cout << g_solver_name << "-a" << area.first << " plans for" << std::endl
                  << get_instance_str(area_info, robot_ids, instance) << std::endl;
#endif
#endif

#ifdef SOLVER_ASP
        std::vector<int> goal_indices;
#endif
        bool sorted = false;
        double timeout = (area_info.solving_time_per_robot < MIN_SOLVING_TIME_PER_ROBOT || last_round) ? TIMEOUT_MAX :
            area_info.solving_time_per_robot * area.second.robot_transit_map.size() * TIMEOUT_TOLERANCE_FACTOR;
        while (true)
        {
          const clock_t time_start = clock();

#ifdef SOLVER_ASP
#ifdef DEBUG_VERBOSE_1
          std::cout << g_solver_name << "-a" << area.first << " plans for "
                    << init << in << goal << "%" << outsider << std::endl;
#endif

          // Create a new control object for making movement plan
          clingo_control_t* ctl;
          clingo_control_new(g_clingo_argv, sizeof(g_clingo_argv) / sizeof(*g_clingo_argv), NULL, NULL, 0, &ctl);
          clingo_control_add(ctl, "base", NULL, 0, init.c_str());
          clingo_control_add(ctl, "base", NULL, 0, in.c_str());
          clingo_control_add(ctl, "base", NULL, 0, goal.c_str());
          clingo_control_add(ctl, "base", NULL, 0, area_info.node_lp.c_str());
          clingo_control_add(ctl, "base", NULL, 0, area_info.nextto_lp.c_str());

          clingo_part_t part_base = {"base", NULL, 0};
          clingo_control_ground(ctl, &part_base, 1, NULL, NULL);

          // Add the movement plan program
          char const* movement_plan_param[] = {"t"};
          clingo_control_add(ctl, "p", movement_plan_param, 1, movement_lp_content.c_str());

          int t = first_timestep;
          double remaining_time = timeout - (double)(clock() - time_start) / CLOCKS_PER_SEC;
          while (remaining_time > 0.0)
          {
#ifdef DEBUG_VERBOSE_3
            std::cout << g_solver_name << "-a" << area.first << " t = " << t << std::endl;
#endif
            clingo_symbol_t sym[2];
            clingo_part_t part_plan = {"p", sym, 1};

            clingo_symbol_create_number(t, &sym[0]);
            clingo_control_ground(ctl, &part_plan, 1, NULL, NULL);

            // Set external query
            clingo_literal_t atom;
            clingo_symbol_create_function("q", sym, 1, true, &sym[1]);
            get_literal(ctl, sym[1], &atom);
            clingo_control_assign_external(ctl, atom, clingo_truth_value_true);

            // Solve
            clingo_solve_result_bitset_t ret;
            std::string model;
            if (solve(ctl, &ret, remaining_time, model) && (ret & clingo_solve_result_satisfiable))
            {
              const double solving_time = (double)(clock() - time_start) / CLOCKS_PER_SEC;
              if (last_round || !emigrates.empty())
              {
                // area_info.solving_time_per_robot = solving_time / std::pow(area.second.robot_transit_map.size(), PER_ROBOT_FACTOR);
                area_info.solving_time_per_robot = solving_time / area.second.robot_transit_map.size();
              }
              else
              {
                area_info.solving_time_per_robot = (TIMEOUT_PENALTY_FACTOR * timeout) /
                                                   (area.second.robot_transit_map.size() * TIMEOUT_TOLERANCE_FACTOR);
              }
              if (area_info.solving_time_per_robot < MIN_SOLVING_TIME_PER_ROBOT)
              {
                area_info.solving_time_per_robot = MIN_SOLVING_TIME_PER_ROBOT;
              }

#ifdef DEBUG_VERBOSE_1
              std::cout << GREEN << g_solver_name << "-a" << area.first << " found a plan"
#ifdef DEBUG_VERBOSE_2
                        << " " << model
#endif
                        << RESET << std::endl;
#endif

              // Mark the robots that have successfully migrated
              if (!last_round)
              {
                auto area_neighbor_confirmed_transits_map_it = g_area_neighbor_confirmed_transits_map.find(area.first);
                if (area_neighbor_confirmed_transits_map_it == g_area_neighbor_confirmed_transits_map.end())
                {
                  area_neighbor_confirmed_transits_map_it =
                      g_area_neighbor_confirmed_transits_map.emplace(area.first, std::unordered_map<int, std::vector<std::pair<int, Transit>>>{}).first;
                }
                auto& neighbor_confirmed_transits_map = area_neighbor_confirmed_transits_map_it->second;

                for (const auto& emigrate : emigrates)
                {
                  area.second.out_robots.insert(emigrate.robot);

                  // Set the transit information of them
                  const Transit& transit = area.second.robot_transit_map.find(emigrate.robot)->second;
                  const int next_area = transit.next_areas.back();
                  Transit next_transit = transit;
                  next_transit.coor_from = emigrate.border_to;  // Start in a local node at time step 1 (not 0)
                  next_transit.direction.x = emigrate.border_to.x - emigrate.border_from.x;
                  next_transit.direction.y = emigrate.border_to.y - emigrate.border_from.y;
                  next_transit.next_areas.pop_back();

                  auto neighbor_confirmed_transits_map_it = neighbor_confirmed_transits_map.find(next_area);
                  if (neighbor_confirmed_transits_map_it == neighbor_confirmed_transits_map.end())
                  {
                    neighbor_confirmed_transits_map_it = neighbor_confirmed_transits_map.emplace(next_area, std::vector<std::pair<int, Transit>>{}).first;
                  }
                  neighbor_confirmed_transits_map_it->second.emplace_back(emigrate.robot, std::move(next_transit));
                }
              }

              // Save the solution
              std::vector<std::string> tokens(tokenizeString(model, "m(,)"));
              for (int i = 0; i < tokens.size(); i += 4)
              {
                const int robot = std::stoi(tokens[i]);
                const Coor direction(std::stoi(tokens[i + 1]), std::stoi(tokens[i + 2]));
                const int timestep = std::stoi(tokens[i + 3]);

                auto robot_moves_last_pair_map_it = area.second.robot_moves_last_pair_map.find(robot);
                if (robot_moves_last_pair_map_it == area.second.robot_moves_last_pair_map.end())
                {
                  robot_moves_last_pair_map_it = area.second.robot_moves_last_pair_map.emplace(
                      robot, std::pair<std::vector<Move>, Coor>{{}, area.second.robot_transit_map.find(robot)->second.coor_from}).first;
                }
                std::pair<std::vector<Move>, Coor>& moves_last_pair = robot_moves_last_pair_map_it->second;
                moves_last_pair.first.emplace_back(Move{direction, timestep});
                moves_last_pair.second.add(direction);
              }

              break;
            }

            remaining_time = timeout - (double)(clock() - time_start) / CLOCKS_PER_SEC;
            clingo_control_release_external(ctl, atom);
            ++t;
          }

          clingo_control_free(ctl);
#endif

#ifdef SOLVER_CBSH2_RTC
          CBS solver(instance, false, 0);
          solver.setPrioritizeConflicts(true);
          solver.setDisjointSplitting(false);
          solver.setBypass(true);
          solver.setRectangleReasoning(rectangle_strategy::GR);
          solver.setCorridorReasoning(corridor_strategy::GC);
          solver.setHeuristicType(heuristics_type::WDG);
          solver.setTargetReasoning(true);
          solver.setMutexReasoning(false);
          solver.setSavingStats(false);
          solver.setNodeLimit(MAX_NODES);

          int min_f_val = 0;
          for (int i = 0; i < RUNS; ++i)
          {
            solver.clear();
            solver.solve(timeout, min_f_val);
            if (solver.solution_found)
              break;
            min_f_val = (int)solver.min_f_val;
            solver.randomRoot = true;
          }
#endif

#ifdef SOLVER_EECBS
          ECBS solver(instance, true, 0);
          solver.setPrioritizeConflicts(true);
          solver.setDisjointSplitting(false);
          solver.setBypass(true);
          solver.setRectangleReasoning(true);
          solver.setCorridorReasoning(true);
          solver.setHeuristicType(heuristics_type::WDG, heuristics_type::GLOBAL);
          solver.setTargetReasoning(true);
          solver.setMutexReasoning(false);
          solver.setConflictSelectionRule(conflict_selection::EARLIEST);
          solver.setNodeSelectionRule(node_selection::NODE_CONFLICTPAIRS);
          solver.setSavingStats(false);
          solver.setHighLevelSolver(high_level_solver_type::EES, 5.0);

          int lowerbound = 0;
          for (int i = 0; i < RUNS; ++i)
          {
            solver.clear();
            solver.solve(timeout, lowerbound);
            if (solver.solution_found)
              break;
            lowerbound = solver.getLowerBound();
            solver.randomRoot = true;
          }
#endif

#ifdef SOLVER_PBS
          PBS solver(instance, true, 0);
          solver.solve(timeout);
#endif

          // Cannot find a movement plan
#ifdef SOLVER_ASP
          if (remaining_time <= 0.0)
#else
          if (!solver.solution_found)
#endif
          {
            // Cannot relax the problem further
            if (last_round || emigrates.empty())
            {
              std::chrono::steady_clock::time_point time_point_end = std::chrono::steady_clock::now();
              std::chrono::milliseconds time_total = std::chrono::duration_cast<std::chrono::milliseconds>(time_point_end - time_point_start);

              std::cout << RED << g_solver_name << "-a" << area.first << " cannot make a movement plan for" << RESET << std::endl
#ifdef SOLVER_ASP
                        << init << in << goal << area_info.node_lp << area_info.nextto_lp << "%" << outsider << std::endl;
#else
                        << get_instance_str(area_info, robot_ids, instance) << std::endl;
              solver.clearSearchEngines();
#endif
              std::cout << "No solution" << std::endl
                        << "Time: " << format_time(time_total.count()) << std::endl;
              ros_exit();
              return EXIT_FAILURE;
            }

            // Relax the goal
            std::cout << YELLOW << g_solver_name << "-a" << area.first << " relaxes a goal" << RESET << std::endl;

            timeout *= TIMEOUT_PENALTY_FACTOR;

#ifdef SOLVER_ASP
            if (sorted)
            {
              emigrates.pop_back();
              goal_indices.pop_back();
              goal = goal.substr(0, goal_indices.back());
            }
            else  // Sort the assigned goals if they have not been sorted
            {
              sorted = true;

              // Sort the outgoing robot assignment based on their distance to border
              std::sort(emigrates.begin(), emigrates.end(), Migrate::compare_descend);

              goal.clear();
              emigrates.pop_back();
              goal_indices.reserve(emigrates.size());
              for (const auto& emigrate : emigrates)
              {
                goal += "g(r(" + std::to_string(emigrate.robot) + "),(" +
                    std::to_string(emigrate.border_from.x) + "," + std::to_string(emigrate.border_from.y) + ")).";
                goal_indices.push_back(goal.size());
              }
            }
#else
            if (!sorted)  // Sort the assigned goals if they have not been sorted
            {
              sorted = true;

              // Sort the outgoing robot assignment based on their distance to border
              std::sort(emigrates.begin(), emigrates.end(), Migrate::compare_descend);
            }
            instance.goal_locations[robot_agent_id_map.find(emigrates.back().robot)->second] = -1;
            emigrates.pop_back();
#endif
          }
          else
          {
#ifndef SOLVER_ASP
#ifdef DEBUG_VERBOSE_1
            std::cout << GREEN << g_solver_name << "-a" << area.first << " found a plan" << RESET << std::endl;
#endif
            const double solving_time = (double)(clock() - time_start) / CLOCKS_PER_SEC;
            if (last_round || !emigrates.empty())
            {
              // area_info.solving_time_per_robot = solving_time / std::pow(area.second.robot_transit_map.size(), PER_ROBOT_FACTOR);
              area_info.solving_time_per_robot = solving_time / area.second.robot_transit_map.size();
            }
            else
            {
              area_info.solving_time_per_robot = (TIMEOUT_PENALTY_FACTOR * timeout) /
                                                 (area.second.robot_transit_map.size() * TIMEOUT_TOLERANCE_FACTOR);
            }
            if (area_info.solving_time_per_robot < MIN_SOLVING_TIME_PER_ROBOT)
            {
              area_info.solving_time_per_robot = MIN_SOLVING_TIME_PER_ROBOT;
            }

            // Mark the robots that have successfully migrated
            if (!last_round)
            {
              auto area_neighbor_confirmed_transits_map_it = g_area_neighbor_confirmed_transits_map.find(area.first);
              if (area_neighbor_confirmed_transits_map_it == g_area_neighbor_confirmed_transits_map.end())
              {
                area_neighbor_confirmed_transits_map_it =
                    g_area_neighbor_confirmed_transits_map.emplace(area.first, std::unordered_map<int, std::vector<std::pair<int, Transit>>>{}).first;
              }
              auto& neighbor_confirmed_transits_map = area_neighbor_confirmed_transits_map_it->second;

              for (const auto& emigrate : emigrates)
              {
                area.second.out_robots.insert(emigrate.robot);

                // Set the transit information of them
                const Transit& transit = area.second.robot_transit_map.find(emigrate.robot)->second;
                const int next_area = transit.next_areas.back();
                Transit next_transit = transit;
                next_transit.coor_from = emigrate.border_to;  // Start in a local node at time step 1 (not 0)
                next_transit.direction.x = emigrate.border_to.x - emigrate.border_from.x;
                next_transit.direction.y = emigrate.border_to.y - emigrate.border_from.y;
                next_transit.next_areas.pop_back();

                auto neighbor_confirmed_transits_map_it = neighbor_confirmed_transits_map.find(next_area);
                if (neighbor_confirmed_transits_map_it == neighbor_confirmed_transits_map.end())
                {
                  neighbor_confirmed_transits_map_it = neighbor_confirmed_transits_map.emplace(next_area, std::vector<std::pair<int, Transit>>{}).first;
                }
                neighbor_confirmed_transits_map_it->second.emplace_back(emigrate.robot, std::move(next_transit));
              }
            }

            // Save the solution
            for (int agent_id = 0; agent_id < instance.num_of_agents; ++agent_id)
            {
              const int robot = robot_ids[agent_id];
              int prev_location = solver.paths[agent_id]->front().location;

              auto robot_moves_last_pair_map_it = area.second.robot_moves_last_pair_map.find(robot);
              if (robot_moves_last_pair_map_it == area.second.robot_moves_last_pair_map.end())
              {
                robot_moves_last_pair_map_it = area.second.robot_moves_last_pair_map.emplace(
                    robot, std::pair<std::vector<Move>, Coor>{{}, area.second.robot_transit_map.find(robot)->second.coor_from}).first;
              }
              std::pair<std::vector<Move>, Coor>& moves_last_pair = robot_moves_last_pair_map_it->second;

              for (int i = 1; i < solver.paths[agent_id]->size(); ++i)
              {
                int location = (*solver.paths[agent_id])[i].location;
                const Coor direction(instance.getColCoordinate(location) - instance.getColCoordinate(prev_location),
                                     instance.getRowCoordinate(location) - instance.getRowCoordinate(prev_location));
                const int timestep = i + first_timestep;

                moves_last_pair.first.emplace_back(Move{direction, timestep});
                moves_last_pair.second.add(direction);
                prev_location = location;
              }
            }

            solver.clearSearchEngines();
#endif
            break;
          }
        }
      }

      // A barrier to ensure that all the active solvers have finished making the movement plan
      {
        dmapf::Track track_msg;
        track_msg.type = 1;

        g_num_track_msg_ignored_1 += g_active_solvers_set.size();   // must use += and before publishing
        track_pub.publish(track_msg);

        while (ros::ok() && g_num_track_msg_ignored_1 > 0)   // must use > 0
        {
          rate.sleep();
        }
      }

#ifdef DEBUG_VERBOSE_3
      std::cout << g_solver_name << ": Phase 3 at round " << g_current_round << std::endl;
#endif

      /* Phase 3: Confirm with higher-ranked neighbors the robots that can actually migrate */
      std::unordered_map<int, std::vector<std::pair<int, Transit>>> area_confirmed_transits_map;
      for (const auto& area_from_to_pair : task.send_areas_map)
      {
        const int area_from = area_from_to_pair.first;
        const auto area_neighbor_confirmed_transits_map_it = g_area_neighbor_confirmed_transits_map.find(area_from);

        for (const auto& area_to_solver_pair : area_from_to_pair.second)
        {
          const int area_to = area_to_solver_pair.first;
          const int solver_to = area_to_solver_pair.second;

          // Construct the migration request message
          dmapf::Migrate::Request migrate_req;
          migrate_req.type = 2;
          migrate_req.area_from = area_from;
          migrate_req.area_to = area_to;

          if (area_neighbor_confirmed_transits_map_it != g_area_neighbor_confirmed_transits_map.end())
          {
            const auto neighbor_confirmed_transits_map_it = area_neighbor_confirmed_transits_map_it->second.find(area_to);
            if (neighbor_confirmed_transits_map_it != area_neighbor_confirmed_transits_map_it->second.end())
            {
              for (const auto& robot_transit_pair : neighbor_confirmed_transits_map_it->second)
              {
                dmapf::Transit transit;
                transit.robot = robot_transit_pair.first;
                transit.coor_from.x = robot_transit_pair.second.coor_from.x;
                transit.coor_from.y = robot_transit_pair.second.coor_from.y;
                transit.coor_goal.x = robot_transit_pair.second.coor_goal.x;
                transit.coor_goal.y = robot_transit_pair.second.coor_goal.y;
                transit.direction.x = robot_transit_pair.second.direction.x;
                transit.direction.y = robot_transit_pair.second.direction.y;
                transit.next_areas = robot_transit_pair.second.next_areas;
                migrate_req.transits.emplace_back(std::move(transit));
              }
            }
          }

          // Send the migrate request for confirmation
          dmapf::Migrate::Response migrate_res;
          const std::string service_name(SOLVER_NAME_PREFIX + std::to_string(solver_to) + "/" + MIGRATE_SERVICE_NAME);
          ros::service::waitForService(service_name);
          if (!ros::service::call(service_name, migrate_req, migrate_res))
          {
            std::cerr << RED << g_solver_name << ": Error in calling service " << service_name << " from area " << area_from
                      << " in phase 3 at round " << g_current_round << RESET << std::endl;
            ros_exit();
            return EXIT_FAILURE;
          }

          auto area_confirmed_transits_map_it = area_confirmed_transits_map.find(area_from);
          if (area_confirmed_transits_map_it == area_confirmed_transits_map.end())
          {
            area_confirmed_transits_map_it = area_confirmed_transits_map.emplace(area_from, std::vector<std::pair<int, Transit>>{}).first;
          }
          std::vector<std::pair<int, Transit>>& robot_transit_pairs = area_confirmed_transits_map_it->second;
          for (const dmapf::Transit& t : migrate_res.transits)
          {
            Transit transit;
            transit.coor_from.x = t.coor_from.x;
            transit.coor_from.y = t.coor_from.y;
            transit.coor_goal.x = t.coor_goal.x;
            transit.coor_goal.y = t.coor_goal.y;
            transit.direction.x = t.direction.x;
            transit.direction.y = t.direction.y;
            transit.next_areas = t.next_areas;
            robot_transit_pairs.emplace_back(std::pair<int, Transit>(t.robot, std::move(transit)));
          }
        }
      }

      // Wait until all confirmation requests from lower solvers have been received
      while (ros::ok() && g_num_migrate_confirm_req_received < task.num_recv_areas)
      {
        rate.sleep();
      }
      g_num_migrate_confirm_req_received = 0; // Reset the counter

      // Add the confirmed robots to the areas
      for (auto& area_confirmed_transits_pair : g_area_confirmed_transits_map)
      {
        Area& area = g_round_areas_map.find(g_current_round)->second.find(area_confirmed_transits_pair.first)->second;
        area.in_robot_transit_pairs.insert(area.in_robot_transit_pairs.end(),
                                           area_confirmed_transits_pair.second.begin(), area_confirmed_transits_pair.second.end());
      }
      for (auto& area_confirmed_transits_pair : area_confirmed_transits_map)
      {
        Area& area = g_round_areas_map.find(g_current_round)->second.find(area_confirmed_transits_pair.first)->second;
        area.in_robot_transit_pairs.insert(area.in_robot_transit_pairs.end(),
                                           area_confirmed_transits_pair.second.begin(), area_confirmed_transits_pair.second.end());
      }

      // Notify all solvers
      dmapf::Track track_msg;
      track_msg.type = 0;
      track_msg.active = false;

      auto round_areas_map_it = g_round_areas_map.find(g_current_round + 1);
      if (round_areas_map_it == g_round_areas_map.end())
      {
        round_areas_map_it = g_round_areas_map.emplace(g_current_round + 1, std::unordered_map<int, Area>{}).first;
      }
      std::unordered_map<int, Area>& child_areas = round_areas_map_it->second;
      for (const auto& area : g_round_areas_map.find(g_current_round)->second)
      {
        if (!area.second.in_robot_transit_pairs.empty() ||
            (!area.second.neighbor_tier_robots_map.empty() && (area.second.robot_transit_map.size() - area.second.out_robots.size()) > 0))
        {
          track_msg.active = true;
          track_msg.solver = g_solver_id;

          AreaInfo& area_info = area.second.area_info;
          auto child_areas_it = child_areas.find(area.first);
          if (child_areas_it == child_areas.end())
          {
            child_areas_it = child_areas.emplace(area.first, Area(area_info)).first;
          }
          Area& child_area = child_areas_it->second;
          create_child_area(child_area, area.second);

          for (const auto& neighbor_tier_robots_pair : child_area.neighbor_tier_robots_map)
          {
            dmapf::Cross cross;
            cross.area_from = area.first;
            cross.area_to = neighbor_tier_robots_pair.first;
            cross.solver_to = area_info.neighbor_connection_map.find(neighbor_tier_robots_pair.first)->second.solver;
            track_msg.crosses.emplace_back(std::move(cross));
          }

          area_latest_round_map.find(area.first)->second = g_current_round + 1;
        }
      }

      // Clear temporary data from the migration process
      g_area_emigrates_map.clear();
      g_area_immigrates_map.clear();
      g_area_canceled_robots_map.clear();
      g_area_canceled_robots_map_req.clear();
      g_area_rejected_robots_map.clear();
      g_area_rejected_robots_map_req.clear();
      g_area_confirmed_transits_map.clear();
      g_area_neighbor_confirmed_transits_map.clear();

      g_num_track_msg_expected += total_solvers;  // must use += and before publishing
      track_pub.publish(track_msg);
    }
    else
    {
      g_num_track_msg_ignored_1 += g_active_solvers_set.size();   // must use += and before publishing
      g_num_track_msg_ignored_2 += g_active_solvers_set.size();   // must use += and before publishing
      g_num_track_msg_expected += total_solvers;  // must use += and before publishing

      // Ensure that the tracking structure gets updated properly
      dmapf::Track track_msg;
      track_msg.type = 0;
      track_msg.active = false;
      track_pub.publish(track_msg);
    }

    // Wait for all the active solvers to report their status
    while (ros::ok() && g_num_track_msg_expected > 0)   // must use > 0
    {
      rate.sleep();
    }

    // No more active solver, we can output the solution
    if (g_active_solvers_set.empty())
    {
      std::chrono::steady_clock::time_point time_point_end = std::chrono::steady_clock::now();
      int64_t time_start = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::time_point_cast<std::chrono::milliseconds>(time_point_start).time_since_epoch()).count();
      int64_t time_end = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::time_point_cast<std::chrono::milliseconds>(time_point_end).time_since_epoch()).count();

      g_num_track_msg_ignored_1 += 1;   // must use += and before publishing

      if (g_solver_id == MASTER_SOLVER_ID)
      {
#ifdef DEBUG_TIMES
  t1 = std::chrono::steady_clock::now();
#endif

        // Wait for plans from other solvers
        while (ros::ok() && g_num_aggregate_req_received < total_solvers - 1)
        {
          rate.sleep();
        }

        // Combine its plan with other solvers plans
        for (const auto& round_areas_pair : g_round_areas_map)
        {
          auto round_moves_map_it = g_round_moves_map.find(round_areas_pair.first);
          if (round_moves_map_it == g_round_moves_map.end())
          {
            round_moves_map_it = g_round_moves_map.emplace(round_areas_pair.first, std::vector<dmapf::Move>{}).first;
          }
          std::vector<dmapf::Move>& moves = round_moves_map_it->second;

          for (const auto& area : round_areas_pair.second)
          {
            for (const auto& robot_moves_last_pair : area.second.robot_moves_last_pair_map)
            {
              moves.emplace_back(dmapf::Move{});
              dmapf::Move& a_move = moves.back();
              a_move.robot = robot_moves_last_pair.first;

              for (const Move& move : robot_moves_last_pair.second.first)
              {
                a_move.directions.emplace_back(dmapf::Coor{});
                a_move.directions.back().x = move.direction.x;
                a_move.directions.back().y = move.direction.y;
                a_move.timesteps.push_back(move.timestep);
              }
            }
          }
        }

        // Produce the solution
        std::string solution;
        int makespan = 0;
        int total_distance = 0;
        std::unordered_map<int, int> robot_arrival_timestep_map;

        for (const auto& round_moves_pair : g_round_moves_map)
        {
          int max_timestep = 0;
          for (const auto& move : round_moves_pair.second)
          {
            auto robot_arrival_timestep_it = robot_arrival_timestep_map.find(move.robot);
            if (robot_arrival_timestep_it == robot_arrival_timestep_map.end())
            {
              robot_arrival_timestep_it = robot_arrival_timestep_map.emplace(move.robot, 0).first;
            }

            for (int i = 0; i < move.directions.size(); ++i)
            {
              solution += "occurs(object(robot," + std::to_string(move.robot) + "),action(move,(" +
                          std::to_string(move.directions[i].x) + "," +
                          std::to_string(move.directions[i].y) + ")" + ")," +
                          std::to_string(makespan + move.timesteps[i]) + ").";

              if (max_timestep < move.timesteps[i])
              {
                max_timestep = move.timesteps[i];
              }
              if (robot_arrival_timestep_it->second < makespan + move.timesteps[i])
              {
                robot_arrival_timestep_it->second = makespan + move.timesteps[i];
              }
            }
            total_distance += move.directions.size();
          }
          makespan += max_timestep;
        }

        int soc = 0;
        for (const auto& robot_arrival_timestep_pair : robot_arrival_timestep_map)
        {
          soc += robot_arrival_timestep_pair.second;
        }

#ifdef DEBUG_TIMES
  t2 = std::chrono::steady_clock::now();
  std::cout << MAGENTA << "Time aggregating = "
            << format_time(std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()) << RESET << std::endl;
#endif

        // Get the maximum solving time across all the solvers
        if (time_start < g_time_start)
        {
          g_time_start = time_start;
        }
        if (time_end > g_time_end)
        {
          g_time_end = time_end;
        }

        // Initialize the answer file
        copy_file(problem_lp, answer_lp);

        // Write the solution
        std::string stat_string;
        if (append_file(answer_lp, solution))
        {
          stat_string += "The solution has been written to " + answer_lp + "\n";
        }
        else
        {
          stat_string += "Unable to write the solution to " + answer_lp + "\n";
        }

        stat_string +=
#ifdef ABSTRACT_ASP
          "Abstract : ASP\n"
#endif
#ifdef ABSTRACT_ASP_ITERABLE
          "Abstract : ASP_ITERABLE\n"
#endif
#ifdef ABSTRACT_BFS
          "Abstract : BFS\n"
#endif
#ifdef ABSTRACT_BFS_RANDOM
          "Abstract : BFS_RANDOM\n"
#endif
#ifdef ABSTRACT_RANDOM
          "Abstract : RANDOM\n"
#endif
#ifdef ABSTRACT_UCS
          "Abstract : UCS\n"
#endif
#ifdef ABSTRACT_UCS_CENTRAL
          "Abstract : UCS_CENTRAL\n"
#endif

#ifdef SOLVER_ASP
          "Solver   : ASP\n"
#endif
#ifdef SOLVER_CBSH2_RTC
          "Solver   : CBSH2_RTC\n"
#endif
#ifdef SOLVER_EECBS
          "Solver   : EECBS\n"
#endif
#ifdef SOLVER_PBS
          "Solver   : PBS\n"
#endif
          "Time     : " + format_time(time_end - time_start) + "\n" +
          "Makespan : " + std::to_string(makespan) + "\n" +
          "SoC      : " + std::to_string(soc) + "\n" +
          "Moves    : " + std::to_string(total_distance);

        // Print the statistics
        std::cout << stat_string << std::endl;

        // Notify the other solvers that the output has been written
        dmapf::Track track_msg;
        track_msg.type = 1;
        track_pub.publish(track_msg);
      }
      else
      {
        // Send the plan to the master solver
        dmapf::Aggregate aggregate_srv;
        aggregate_srv.request.time_start = time_start;
        aggregate_srv.request.time_end = time_end;

        for (const auto& round_areas_pair : g_round_areas_map)
        {
          aggregate_srv.request.plans.emplace_back(dmapf::Plan{});
          dmapf::Plan& plan = aggregate_srv.request.plans.back();
          plan.round = round_areas_pair.first;
          for (const auto& area : round_areas_pair.second)
          {
            for (const auto& robot_moves_last_pair : area.second.robot_moves_last_pair_map)
            {
              plan.moves.emplace_back(dmapf::Move{});
              dmapf::Move& a_move = plan.moves.back();
              a_move.robot = robot_moves_last_pair.first;

              for (const Move& move : robot_moves_last_pair.second.first)
              {
                a_move.directions.emplace_back(dmapf::Coor{});
                a_move.directions.back().x = move.direction.x;
                a_move.directions.back().y = move.direction.y;
                a_move.timesteps.push_back(move.timestep);
              }
            }
          }
        }

        const std::string SERVICE_NAME(SOLVER_NAME_PREFIX + std::to_string(MASTER_SOLVER_ID) + "/" + AGGREGATE_SERVICE_NAME);
        ros::service::waitForService(SERVICE_NAME);
        if (!ros::service::call(SERVICE_NAME, aggregate_srv))
        {
          std::cerr << RED << g_solver_name << ": Error in calling service " << SERVICE_NAME << RESET << std::endl;
          ros_exit();
        }
      }

      // Wait until the output is written
      while (ros::ok() && g_num_track_msg_ignored_1 > 0)
      {
        rate.sleep();
      }

      ros_exit();
      return EXIT_SUCCESS;
    }

    // Construct the next task and add any missing areas that will be migrating into
    task.reset();
    for (const auto& cross : g_crosses)
    {
      task.add_task(cross);

      if (g_solver_id != cross.solver_from || cross.solver_from == cross.solver_to)   // Receiver needs to make sure the area has been initialized
      {
        // Create the receiving area if it has not been created
        auto round_areas_map_it = g_round_areas_map.find(g_current_round + 1);
        if (round_areas_map_it == g_round_areas_map.end())
        {
          round_areas_map_it = g_round_areas_map.emplace(g_current_round + 1, std::unordered_map<int, Area>{}).first;
        }
        std::unordered_map<int, Area>& areas_map = round_areas_map_it->second;
        if (areas_map.find(cross.area_to) == areas_map.end())
        {
          AreaInfo& area_info = g_area_info_map.find(cross.area_to)->second;
          const auto area_latest_round_map_it = area_latest_round_map.find(cross.area_to);
          if (area_latest_round_map_it == area_latest_round_map.end())  // There is no history of this area, so create new
          {
            initialize_area(areas_map.emplace(cross.area_to, Area(area_info)).first->second);
            area_latest_round_map.emplace(cross.area_to, g_current_round + 1);
          }
          else  // Lookup for the old history
          {
            const int prev_latest_round = area_latest_round_map_it->second;
            create_child_area(areas_map.emplace(cross.area_to, Area(area_info)).first->second,
                              g_round_areas_map.find(prev_latest_round)->second.find(cross.area_to)->second);
            area_latest_round_map_it->second = g_current_round + 1;
          }
        }
      }
    }

#ifdef DEBUG_VERBOSE_3
    std::cout << g_solver_name << " finished round " << g_current_round << std::endl;
#endif

    ++g_current_round;

    // Another barrier to ensure that all the active solvers have the areas ready
    {
      dmapf::Track track_msg;
      track_msg.type = 1;

      g_num_track_msg_ignored_1 += g_active_solvers_set.size();   // must use += and before publishing
      if (g_active_solvers_set.find(g_solver_id) != g_active_solvers_set.end())
      {
        track_pub.publish(track_msg);
      }

      while (ros::ok() && g_num_track_msg_ignored_1 > 0)   // must use > 0
      {
        rate.sleep();
      }
    }
  }

  ros_exit();
  return EXIT_FAILURE;
}
