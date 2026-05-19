#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <boost/filesystem.hpp>

#include "dmapf/color.h"
#include "dmapf/utility.h"

#define ONLY_ROBOTS

void exit_failure(const std::string& message)
{
  std::cerr << message << std::endl;
  std::exit(EXIT_FAILURE);
}

int main(int argc, char **argv)
{
  const std::string PROGRAM_NAME("map2lp");

  int num_robots = -1;
  std::string path_map;
  std::string path_scene;

  // Evaluate input arguments
  for (int i = 1; i < argc; ++i)
  {
    if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "-h"))
    {
      goto help;
    }
    else if (i < argc - 1)  // for options that require 1 argument
    {
      if (!strcmp(argv[i], "--map") || !strcmp(argv[i], "-m"))
      {
        path_map = argv[++i];
        continue;
      }
      else if (!strcmp(argv[i], "--scene") || !strcmp(argv[i], "-s"))
      {
        path_scene = argv[++i];
        continue;
      }
      else
      {
        try
        {
          if (!strcmp(argv[i], "--robot") || !strcmp(argv[i], "-r"))
          {
            num_robots = std::stoi(argv[++i]);
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
              << "  --map, -m     Input .map file" << std::endl
              << "  --robot, -r   Set the number of robots to take from the scenario file" << std::endl
              << "  --scene, -s   Input .scen file" << std::endl;
    return EXIT_FAILURE;
  }

  // Check the input arguments
  bool invalid_arguments = false;
  if (path_map.empty())
  {
    std::cerr << "-m is required but missing" << std::endl;
    invalid_arguments = true;
  }
  if (!path_scene.empty())
  {
    if (num_robots < 0)
    {
      std::cerr << "-r must be non-negative" << std::endl;
      invalid_arguments = true;
    }
  }

  // Exit if there is an invalid argument
  if (invalid_arguments)
  {
    return EXIT_FAILURE;
  }

  // Creating output
  int num_nodes = 0;
  std::string node_str;
  std::string robot_str;
  std::string shelf_str;
#ifdef ONLY_ROBOTS
  std::string goal_str;
#else
  std::string order_str;
  std::string product_str;
#endif

  std::ifstream file_map(path_map);
  if (file_map.is_open())
  {
    int line_num = 0;
    int node_num = 0;
    int row_num = 0;
    std::string line_map;

    while (std::getline(file_map, line_map))
    {
      ++line_num;

      // Skip the first four header lines
      if (line_num > 4)
      {
        ++row_num;

        int col_num = 0;
        for (char& c : line_map)
        {
          ++col_num;

          // Passable terrain
          if (c == '.' || c == 'G')
          {
            node_str += "init(object(node," + std::to_string(++node_num) + "),value(at,(" +
                        std::to_string(col_num) + "," + std::to_string(row_num) + "))).\n";
            ++num_nodes;
          }
        }
      }
    }
    file_map.close();
  }
  else
  {
    exit_failure("Cannot open " + path_map);
  }

  if (!path_scene.empty())
  {
    std::ifstream file_scene(path_scene);
    if (file_scene.is_open())
    {
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
          robot_str += "init(object(robot," + std::to_string(robot_num) + "),value(at,(" + std::to_string(start_col + 1) + "," + std::to_string(start_row + 1) + "))).\n";
          shelf_str += "init(object(shelf," + std::to_string(robot_num) + "),value(at,(" + std::to_string(goal_col + 1) + "," + std::to_string(goal_row + 1) + "))).\n";
#ifdef ONLY_ROBOTS
          goal_str += "goal(robot(" + std::to_string(robot_num) + "),(" + std::to_string(goal_col + 1) + "," + std::to_string(goal_row + 1) + ")).\n";
#else
          order_str += "init(object(order," + std::to_string(robot_num) + "),value(line,(" + std::to_string(robot_num) + ",1))).\n";
          product_str += "init(object(product," + std::to_string(robot_num) + "),value(on,(" + std::to_string(robot_num) + ",1))).\n";
#endif
        }
        else
        {
          exit_failure("Invalid format in " + path_scene);
        }
      }
      file_scene.close();

      if (robot_num < num_robots)
      {
        std::cout << YELLOW << "The scenario file only contains " << robot_num << " robots" << RESET << std::endl;
        num_robots = robot_num;
      }
    }
    else
    {
      exit_failure("Cannot open " + path_scene);
    }
  }

  // Extract filename from the input path
  boost::filesystem::path path(path_map);

  // Write output
  std::string out_path;
  if (path.parent_path().empty())
  {
    out_path = path.stem().string() + "_n" + std::to_string(num_nodes) + "_r" + std::to_string(num_robots) + ".lp";
  }
  else
  {
    if (num_robots > -1)
    {
      out_path = path.parent_path().string() + "/" + path.stem().string() + "_n" + std::to_string(num_nodes) + "_r" + std::to_string(num_robots) + ".lp";
    }
    else
    {
      out_path = path.parent_path().string() + "/" + path.stem().string() + "_n" + std::to_string(num_nodes) + ".lp";
    }
  }

#ifdef ONLY_ROBOTS
  if (write_file(out_path, node_str + robot_str + shelf_str + goal_str))
#else
  if (write_file(out_path, node_str + robot_str + order_str + product_str + shelf_str))
#endif
  {
    std::cout << "Written " << out_path << std::endl;
    return EXIT_SUCCESS;
  }
  else
  {
    exit_failure("Cannot write an lp file " + out_path);
  }
}
