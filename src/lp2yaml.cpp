#include <iostream>
#include <map>
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

  int x = 0;
  int y = 0;
  std::string path_string;

  // Evaluate input arguments
  for (int i = 1; i < argc; ++i)
  {
    if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "-h"))
    {
      goto help;
    }
    else if (i < argc - 1)  // for options that require 1 argument
    {
      if (!strcmp(argv[i], "--input") || !strcmp(argv[i], "-i"))
      {
        path_string = argv[++i];
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
              << "  --help, -h    Print help and exit" << std::endl
              << "  --input, -i   Input .lp file" << std::endl
              << "  --col, -x     Set the number of columns" << std::endl
              << "  --row, -y     Set the number of rows" << std::endl;
    return EXIT_SUCCESS;
  }

  // Check the input arguments
  bool invalid_arguments = false;
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
  if (path_string.empty())
  {
    std::cerr << "-i is required but missing" << std::endl;
    invalid_arguments = true;
  }

  // Exit if there is an invalid argument
  if (invalid_arguments)
  {
    return EXIT_FAILURE;
  }

  // Open the input file
  clingo_control_t* ctl;
  const char* clingo_argv[] = {(const char*)"--warn=none"};
  clingo_control_new(clingo_argv, 1, NULL, NULL, 0, &ctl);
  if (!clingo_control_load(ctl, path_string.c_str()))
  {
    exit_failure("Cannot open " + path_string, ctl);
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

  // Creating output
  std::string output_string("agents:\n");
  for (const auto& robot_start_pair : robot_start_map)
  {
    const auto robot_goal_map_it = robot_goal_map.find(robot_start_pair.first);
    if (robot_goal_map_it != robot_goal_map.end())
    {
      output_string +=
          "-   name: agent" + std::to_string(robot_start_pair.first - 1) + "\n" +
          "    start: [" + std::to_string(robot_start_pair.second.x - 1) + ", " + std::to_string(robot_start_pair.second.y - 1) + "]\n" +
          "    goal: [" + std::to_string(robot_goal_map_it->second.x - 1) + ", " + std::to_string(robot_goal_map_it->second.y - 1) + "]\n";
    }
    else
    {
      std::cout << YELLOW << "Skip robot " << robot_start_pair.first << " because it does not have a goal" << RESET << std::endl;
    }
  }

  output_string +=
      "map:\n"
      "    dimensions: [" + std::to_string(x) + ", " + std::to_string(y) + "]\n"
      "    obstacles:";

  bool has_obstacles = false;
  for (int i = 1; i <= x; ++i)
  {
    for (int j = 1; j <= y; ++j)
    {
      if (coor_set.find(Coor{i, j}) == coor_set.end())
      {
        output_string +=  "\n    - [" + std::to_string(i - 1) + ", " + std::to_string(j - 1) + "]";
        has_obstacles = true;
      }
    }
  }

  if (has_obstacles)
  {
    output_string += "\n";
  }
  else
  {
    output_string += " []\n";
  }

  // Extract filename from the input path
  boost::filesystem::path path(path_string);

  // Write output
  const std::string out_path = (path.parent_path().empty()?
    path.stem().string() + ".yaml" :
    path.parent_path().string() + "/" + path.stem().string() + ".yaml");

  if (write_file(out_path, output_string))
  {
    std::cout << "Written " << out_path << std::endl;
    return EXIT_SUCCESS;
  }
  else
  {
    exit_failure("Cannot write a yaml file " + out_path);
  }
}
