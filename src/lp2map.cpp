#include <climits>
#include <iostream>
#include <map>
#include <vector>
#include <unordered_map>
#include <unordered_set>

#include <boost/filesystem.hpp>

#include "dmapf/clingo_wrap.h"
#include "dmapf/color.h"
#include "dmapf/utility.h"

#define ONLY_ROBOTS       // Deal with problems with only robots (no shelves or orders)

#ifdef ONLY_ROBOTS
const char CONVERT_PROGRAM[] =
    "#external show_goals."
    "#external show_nodes."
    "#external show_robots."
    "#show."
    "#show (R,C) : goal(robot(R),C), show_goals."
    "#show C : init(object(node,_),value(at,C)), show_nodes."
    "#show (R,C) : init(object(robot,R),value(at,C)), show_robots."
    ;
#else
const char CONVERT_PROGRAM[] =
    "#external show_goals."
    "#external show_nodes."
    "#external show_robots."
    "goal(robot(R),C) :- init(object(order,R),value(line,(P,_))), init(object(product,P),value(on,(S,_))), init(object(shelf,S),value(at,C))."
    "#show."
    "#show (R,C) : goal(R,C), show_goals."
    "#show C : init(object(node,_),value(at,C)), show_nodes."
    "#show (R,C) : init(object(robot,R),value(at,C)), show_robots."
    ;
#endif

struct Coor
{
  int x;
  int y;

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

void exit_failure(const std::string& message)
{
  std::cerr << RED << message << RESET << std::endl;
  std::exit(EXIT_FAILURE);
}

void exit_failure(const std::string& message, clingo_control_t* ctl)
{
  clingo_control_free(ctl);
  exit_failure(message);
}

int main(int argc, char **argv)
{
  const std::string PROGRAM_NAME("lp2yaml");

  if (argc < 2)
  {
    std::cout << "Usage: rosrun dmapf " << PROGRAM_NAME << " [problem files in the format of ASPRILO]" << std::endl;
    return EXIT_SUCCESS;
  }

  // Evaluate input arguments
  for (int i = 1; i < argc; ++i)
  {
    const std::string input_path(argv[i]);

    // Open the input file
    clingo_control_t* ctl;
    const char* clingo_argv[] = {(const char*)"--warn=none"};
    clingo_control_new(clingo_argv, 1, NULL, NULL, 0, &ctl);
    if (!clingo_control_load(ctl, input_path.c_str()))
    {
      std::cerr << RED << "Cannot open " << input_path << " - skip" << RESET << std::endl;
      clingo_control_free(ctl);
      continue;
    }

    // Add the convert program
    clingo_control_add(ctl, "convert", NULL, 0, CONVERT_PROGRAM);

    // Ground the parts
    clingo_part_t parts[] = {{"base", NULL, 0}, {"convert", NULL, 0}};
    clingo_control_ground(ctl, parts, 2, NULL, NULL);

    // Get robot goals
    std::unordered_map<int, Coor> robot_goal_map;
    {
      clingo_symbol_t sym;
      clingo_literal_t atom;
      clingo_symbol_create_function("show_goals", NULL, 0, true, &sym);
      get_literal(ctl, sym, &atom);
      clingo_control_assign_external(ctl, atom, clingo_truth_value_true);

      std::string model;
      clingo_solve_result_bitset_t ret;
      if (solve(ctl, &ret, model))
      {
        if (ret & clingo_solve_result_satisfiable)
        {
          std::vector<std::string> tokens(tokenizeString(model));
          robot_goal_map.reserve(tokens.size() / 3);
          for (int i = 0; i < tokens.size(); i += 3)
          {
            const int robot = std::stoi(tokens[i]);
            if (robot_goal_map.find(robot) == robot_goal_map.end())
            {
              robot_goal_map.emplace(robot, Coor{std::stoi(tokens[i + 1]), std::stoi(tokens[i + 2])});
            }
            else
            {
              std::cout << YELLOW << "Robot " << robot << " has multiple goals" << RESET << std::endl;
            }
          }
        }
        else
        {
          exit_failure("Unsat in getting robot goals", ctl);
        }
      }
      else
      {
        exit_failure("Cannot get robot goals", ctl);
      }
      clingo_control_release_external(ctl, atom);
    }

    // Get robot starting locations
    std::map<int, Coor> robot_start_map;
    {
      clingo_symbol_t sym;
      clingo_literal_t atom;
      clingo_symbol_create_function("show_robots", NULL, 0, true, &sym);
      get_literal(ctl, sym, &atom);
      clingo_control_assign_external(ctl, atom, clingo_truth_value_true);

      std::string model;
      clingo_solve_result_bitset_t ret;
      if (solve(ctl, &ret, model))
      {
        if (ret & clingo_solve_result_satisfiable)
        {
          std::vector<std::string> tokens(tokenizeString(model));
          for (int i = 0; i < tokens.size(); i += 3)
          {
            const int robot = std::stoi(tokens[i]);
            if (robot_start_map.find(robot) == robot_start_map.end())
            {
              robot_start_map.emplace(robot, Coor{std::stoi(tokens[i + 1]), std::stoi(tokens[i + 2])});
            }
            else
            {
              std::cout << YELLOW << "Robot " << robot << " has multiple starting locations" << RESET << std::endl;
            }
          }
        }
        else
        {
          exit_failure("Unsat in getting robot starting locations", ctl);
        }
      }
      else
      {
        exit_failure("Cannot get robot starting locations", ctl);
      }
      clingo_control_release_external(ctl, atom);
    }

    // Get nodes
    Coor min_node(INT_MAX, INT_MAX);
    Coor max_node(INT_MIN, INT_MIN);
    std::unordered_set<Coor, CoorHasher> coor_set;
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
          std::vector<std::string> tokens(tokenizeString(model));
          coor_set.reserve(tokens.size() / 2);
          for (int i = 0; i < tokens.size(); i += 2)
          {
            const Coor coor{std::stoi(tokens[i]), std::stoi(tokens[i + 1])};
            if (coor_set.find(coor) == coor_set.end())
            {
              coor_set.emplace(coor);

              if (coor.x < min_node.x) min_node.x = coor.x;
              if (coor.y < min_node.y) min_node.y = coor.y;
              if (coor.x > max_node.x) max_node.x = coor.x;
              if (coor.y > max_node.y) max_node.y = coor.y;
            }
            else
            {
              std::cout << YELLOW << "Coordinate (" << coor.x << "," << coor.y << ") has multiple node IDs" << RESET << std::endl;
            }
          }
        }
        else
        {
          exit_failure("Unsat in getting nodes", ctl);
        }
      }
      else
      {
        exit_failure("Cannot get nodes", ctl);
      }
      clingo_control_release_external(ctl, atom);
    }

    // Free the Clingo control object
    clingo_control_free(ctl);

    // Decide on output file names
    boost::filesystem::path path(input_path);
    const int num_of_cols = max_node.x - min_node.x + 1;
    const int num_of_rows = max_node.y - min_node.y + 1;

    // Create the map file
    {
      const int map_size = num_of_cols * num_of_rows;
      std::vector<bool> my_map(map_size, true);

      for (const auto& coor : coor_set)
      {
        my_map[num_of_cols * (coor.y - min_node.y) + (coor.x - min_node.x)] = false;
      }

      std::string str("type octile\nheight " + std::to_string(num_of_rows) + "\nwidth " + std::to_string(num_of_cols) + "\nmap\n");
      int location = -1;
      for (int i = 0; i < num_of_rows; ++i)
      {
        for (int j = 0; j < num_of_cols; ++j)
        {
          str += my_map[++location] ? "@" : ".";
        }
        str += "\n";
      }

      const std::string output_map_path = (path.parent_path().empty()?
          path.stem().string() + ".map" :
          path.parent_path().string() + "/" + path.stem().string() + ".map");

      if (write_file(output_map_path, str))
      {
        std::cout << "Written " << output_map_path << std::endl;
      }
      else
      {
        std::cout << RED << "Cannot write " << output_map_path << " - skip" << RESET << std::endl;
      }
    }

    // Create the scene file
    {
      std::string str("version 1\n");
      for (const auto& robot_start_pair : robot_start_map)
      {
        const int robot = robot_start_pair.first;
        const Coor s_coor(robot_start_pair.second.x - min_node.x, robot_start_pair.second.y - min_node.y);
        Coor g_coor(-1, -1);
        int d = 0;

        const auto robot_goal_it = robot_goal_map.find(robot);
        if (robot_goal_it != robot_goal_map.end())
        {
          g_coor.x = robot_goal_it->second.x - min_node.x;
          g_coor.y = robot_goal_it->second.y - min_node.y;
          d = std::abs(s_coor.x - g_coor.x) + std::abs(s_coor.y - g_coor.y);
        }

        str +=  std::to_string(robot) +
               "\t" + path.stem().string() +
               "\t" + std::to_string(num_of_cols) +
               "\t" + std::to_string(num_of_rows) +
               "\t" + std::to_string(s_coor.x) + "\t" + std::to_string(s_coor.y) +
               "\t" + std::to_string(g_coor.x) + "\t" + std::to_string(g_coor.y) +
               "\t" + std::to_string(d) + "\n";
      }

      const std::string output_scen_path = (path.parent_path().empty()?
          path.stem().string() + ".scen" :
          path.parent_path().string() + "/" + path.stem().string() + ".scen");
      if (write_file(output_scen_path, str))
      {
        std::cout << "Written " << output_scen_path << std::endl;
      }
      else
      {
        std::cout << RED << "Cannot write " << output_scen_path << " - skip" << RESET << std::endl;
      }
    }
  }

  return EXIT_SUCCESS;
}
