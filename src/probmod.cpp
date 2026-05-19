#include <iostream>
#include <map>
#include <sstream>
#include <vector>
#include <unordered_map>

#include "dmapf/color.h"
#include "dmapf/utility.h"

#define MIN_NUM_FREE_NODES 4      // This value should be greater or equal to the value used in the solver

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
};

struct CoorHasher
{
  std::size_t operator() (const Coor& key) const
  {
    // Assume size_t = 8 Bytes
    // |-- x (32 bits) --|-- y (32 bits) --|
    return ((std::size_t)key.x << (sizeof(std::size_t) << 2)) | key.y;
  }
};

struct Robot
{
  int robot;
  int area_goal;
  Coor coor_start;
  Coor coor_goal;

  std::string str() const
  {
    return std::to_string(robot) + " " + std::to_string(area_goal) + " " +
           std::to_string(coor_start.x) + " " + std::to_string(coor_start.y) + " " +
           std::to_string(coor_goal.x) + " " + std::to_string(coor_goal.y);
  }
};

struct Domain
{
  const std::string file_name;
  std::vector<std::string> area_strs;
  std::vector<std::string> lp_strs;
  std::map<int, std::vector<Robot>> area_robots_map;

  Domain(const std::string& file_name) : file_name(file_name) {}
};

void exit_failure(const std::string& message)
{
  std::cerr << message << std::endl;
  std::exit(EXIT_FAILURE);
}

int main(int argc, char **argv)
{
  const std::string PROGRAM_NAME("probmod");
  if (argc < 4)
  {
    std::cout << "Usage: rosrun dmapf " << PROGRAM_NAME << "[ALL domain files for the problem] SCENE_FILE NUM_AGENTS" << std::endl;
    return EXIT_SUCCESS;
  }

  std::unordered_map<int, std::pair<int, int>> area_congestion_map;
  std::unordered_map<Coor, int, CoorHasher> coor_area_map;
  std::unordered_map<int, int> area_domain_map;
  std::vector<Domain> domains;
  domains.reserve(argc - 3);
  for (int i = 1; i < argc - 2; ++i)
  {
    // Read the content of the domain file
    std::string dom_content;
    if (read_file(argv[i], dom_content))
    {
      std::vector<std::string> parts = tokenizeString(dom_content, "%");

      domains.emplace_back(Domain{argv[i]});
      auto& domain = domains.back();
      std::vector<int> ordered_areas;

      // Check the areas
      std::vector<std::string> header_parts = tokenizeString(parts[0], "\n");
      const int num_areas = std::stoi(header_parts[0]);
      domain.area_strs.reserve(num_areas);
      for (int j = 1; j < num_areas + 1; ++j)
      {
        std::istringstream iss(header_parts[j]);
        int area;
        iss >> area;

        ordered_areas.push_back(area);
        area_domain_map.emplace(area, domains.size() - 1);
        domain.area_strs.emplace_back(header_parts[j]);
      }

      // Check the coordinates
      int ordered_areas_index = -1;
      for (int j = 1; j < parts.size(); ++j)
      {
        parts[j] = trim(parts[j]);

        if (!parts[j].empty() && parts[j][0] == 'i')
        {
          const int area = ordered_areas[++ordered_areas_index];

          std::vector<std::string> coor_tokens = tokenizeString(parts[j], "i(,).");
          for (int k = 0; k < coor_tokens.size(); k += 2)
          {
            coor_area_map.emplace(Coor{std::stoi(coor_tokens[k]), std::stoi(coor_tokens[k + 1])}, area);
          }

          domain.area_robots_map.emplace(area, std::vector<Robot>{});
          area_congestion_map.emplace(area, std::pair<int, int>(0, coor_tokens.size() / 2));
        }
        domain.lp_strs.emplace_back(parts[j]);
      }

    }
    else
    {
      exit_failure("Cannot open " + std::string(argv[i]));
    }
  }

  // Read the scenario file
  std::ifstream file_scene(argv[argc - 2]);
  if (file_scene.is_open())
  {
    const int num_robots = std::stoi(argv[argc - 1]);
    int robot_num = 0;
    std::string line_scene;
    std::getline(file_scene, line_scene);   // Skip the first header line

    while (robot_num < num_robots && std::getline(file_scene, line_scene))
    {
      ++robot_num;

      std::istringstream iss(line_scene);

      int bucket;
      std::string map;
      int width = 0, height = 0;
      int start_col, start_row, goal_col, goal_row;
      double dist;

      if (iss >> bucket >> map >> width >> height >> start_col >> start_row >> goal_col >> goal_row)
      {
        const Coor coor_start(start_col + 1, start_row + 1);
        const Coor coor_goal(goal_col + 1, goal_row + 1);
        const int area_start = coor_area_map.find(coor_start)->second;
        const int area_goal = coor_area_map.find(coor_goal)->second;
        Domain& domain = domains[area_domain_map.find(area_start)->second];

        auto area_robots_map_it = domain.area_robots_map.find(area_start);
        area_robots_map_it->second.emplace_back(Robot{robot_num, area_goal, coor_start, coor_goal});

        ++area_congestion_map.find(area_goal)->second.first;
      }
      else
      {
        exit_failure("Invalid format in " + std::string(argv[argc - 2]));
      }
    }

    if (robot_num < num_robots)
    {
      std::cout << YELLOW << "The scenario file only contains " << robot_num << " robots" << RESET << std::endl;
    }

    file_scene.close();
  }
  else
  {
    exit_failure("Cannot open " + std::string(argv[argc - 2]));
  }

  for (const Domain& domain : domains)
  {
    std::string str;

    str += std::to_string(domain.area_strs.size()) + "\n";
    for (const std::string& area_str : domain.area_strs)
    {
      str += area_str + "\n";
    }

    str += std::to_string(domain.area_robots_map.size()) + "\n";
    for (const auto& area_robots_pair : domain.area_robots_map)
    {
      str += std::to_string(area_robots_pair.first) + " " + std::to_string(area_robots_pair.second.size()) + "\n";
      for (const Robot& robot : area_robots_pair.second)
      {
        str += robot.str() + "\n";
      }
    }

    for (const std::string& lp_str : domain.lp_strs)
    {
      str += "%\n" + lp_str + "\n";
    }

    if (write_file(domain.file_name, str))
    {
      std::cout << "Replaced " << domain.file_name << std::endl;
    }
    else
    {
      exit_failure("Cannot write " + domain.file_name);
    }
  }

  std::string warn_string;
  for (const auto& area_congestion_pair : area_congestion_map)
  {
    const int num_goals = area_congestion_pair.second.first;
    const int num_nodes = area_congestion_pair.second.second;

    if (num_nodes - num_goals <= MIN_NUM_FREE_NODES)
    {
      warn_string += "a" + std::to_string(area_congestion_pair.first) + ": " + std::to_string(num_goals) + " / " + std::to_string(num_nodes) + "\n";
    }
  }

  if (!warn_string.empty())
  {
    print_line();
    std::cout << "area: #goals / #nodes" << std::endl << warn_string;
  }

  return EXIT_SUCCESS;
}
