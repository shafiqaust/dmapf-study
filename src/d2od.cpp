/*
 *  Replace domain files of the current format with an old format used in ICLP2021 for testing purposes
 */

#include <iostream>
#include <sstream>
#include <vector>
#include <unordered_map>

#include "dmapf/utility.h"

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

struct Domain
{
  std::string header;
  std::string content;
};

void exit_failure(const std::string& message)
{
  std::cerr << message << std::endl;
  std::exit(EXIT_FAILURE);
}

int main(int argc, char **argv)
{
  const std::string PROGRAM_NAME("d2od");
  if (argc < 2)
  {
    std::cout << "Usage: rosrun dmapf " << PROGRAM_NAME << " [ALL domain files for the problem]" << std::endl;
    return EXIT_SUCCESS;
  }

  // Analyze contents of the input domain files
  std::vector<Domain> domains;
  domains.emplace_back(Domain{});   // add a dummy model to start indexing at 1

  int id = 0;
  std::unordered_map<Coor, int, CoorHasher> coor_id_map;

  for (int i = 1; i < argc; ++i)
  {
    // Read the content of the domain file
    std::string dom_content;
    if (read_file(argv[i], dom_content))
    {
      std::vector<std::string> parts = tokenizeString(dom_content, "%");

      // Analyze and convert the atoms
      std::string converted_content;
      for (int j = 1; j < parts.size(); ++j)
      {
        if (parts[j].empty())
        {
          continue;
        }

        converted_content += "%\n";

        std::vector<std::string> atoms = tokenizeString(parts[j], "(,).\n");
        for (int k = 0; k < atoms.size();)
        {
          if (atoms[k] == "o" || atoms[k] == "i")
          {
            Coor coor;
            try
            {
              coor.x = std::stoi(atoms[k + 1]);
              coor.y = std::stoi(atoms[k + 2]);
            }
            catch (const std::exception& e)
            {
              exit_failure("Invalid domain format in " + std::string(argv[i]));
            }

            const auto coor_it = coor_id_map.find(coor);
            if (coor_it == coor_id_map.end())
            {
              ++id;
              converted_content += atoms[k] + "(" + std::to_string(id) + ",(" + std::to_string(coor.x) + "," + std::to_string(coor.y) + ")).";
              coor_id_map.emplace(coor, id);
            }
            else
            {
              converted_content += atoms[k] + "(" + std::to_string(coor_it->second) + ",(" + std::to_string(coor.x) + "," + std::to_string(coor.y) + ")).";
            }

            k += 3;
          }
          else if (atoms[k] == "x")
          {
            Coor from;
            Coor to;
            Coor direction;
            try
            {
              from.x = std::stoi(atoms[k + 1]);
              from.y = std::stoi(atoms[k + 2]);
              to.x = std::stoi(atoms[k + 3]);
              to.y = std::stoi(atoms[k + 4]);
              direction.x = std::stoi(atoms[k + 5]);
              direction.y = std::stoi(atoms[k + 6]);
            }
            catch (const std::exception& e)
            {
              exit_failure("Invalid domain format in " + std::string(argv[i]));
            }

            const auto from_it = coor_id_map.find(from);
            const auto to_it = coor_id_map.find(to);
            if (from_it == coor_id_map.end())
            {
              exit_failure("Cannot identify coordinate (" + std::to_string(from.x) + "," + std::to_string(from.y) + ")");
            }
            if (to_it == coor_id_map.end())
            {
              exit_failure("Cannot identify coordinate (" + std::to_string(to.x) + "," + std::to_string(to.y) + ")");
            }

            converted_content += "x(" + std::to_string(from_it->second) + "," + std::to_string(to_it->second) + ",";

            if (direction.x == -1 && direction.y == 0)
            {
              converted_content += std::to_string(0) + ").";
            }
            else if (direction.x == 1 && direction.y == 0)
            {
              converted_content += std::to_string(1) + ").";
            }
            else if (direction.x == 0 && direction.y == -1)
            {
              converted_content += std::to_string(2) + ").";
            }
            else if (direction.x == 0 && direction.y == 1)
            {
              converted_content += std::to_string(3) + ").";
            }
            else
            {
              exit_failure("Invalid direction (" + std::to_string(direction.x) + "," + std::to_string(direction.y) + ")");
            }

            k += 7;
          }
          else if (atoms[k] == "l")
          {
            Coor from;
            Coor to;
            try
            {
              from.x = std::stoi(atoms[k + 1]);
              from.y = std::stoi(atoms[k + 2]);
              to.x = std::stoi(atoms[k + 3]);
              to.y = std::stoi(atoms[k + 4]);
            }
            catch (const std::exception& e)
            {
              exit_failure("Invalid domain format in " + std::string(argv[i]));
            }

            const auto from_it = coor_id_map.find(from);
            auto to_it = coor_id_map.find(to);
            if (from_it == coor_id_map.end())
            {
              exit_failure("Cannot identify coordinate (" + std::to_string(from.x) + "," + std::to_string(from.y) + ")");
            }
            if (to_it == coor_id_map.end())
            {
              to_it = coor_id_map.emplace(Coor(to.x, to.y), ++id).first;
            }

            converted_content += "l(" +
                std::to_string(from_it->second) + ",(" + std::to_string(from.x) + "," + std::to_string(from.y) + ")," +
                std::to_string(to_it->second) + ",(" + std::to_string(to.x) + "," + std::to_string(to.y) + ")).";

            k += 5;
          }
          else if (atoms[k] == "d")
          {
            try
            {
              std::stoi(atoms[k + 1]);
              std::stoi(atoms[k + 2]);
            }
            catch (const std::exception& e)
            {
              exit_failure("Invalid domain format in " + std::string(argv[i]));
            }

            k += 3;
          }
          else
          {
            exit_failure("Invalid domain format in " + std::string(argv[i]));
          }
        }

        converted_content += "\n";
      }

      domains.emplace_back(Domain{parts[0], converted_content});
    }
    else
    {
      exit_failure("Cannot open " + std::string(argv[i]));
    }
  }

  // Analyze headers of the domains
  for (int i = 1; i < argc; ++i)
  {
    std::stringstream info_stream(domains[i].header);

    // Get area info
    std::string area_str;
    int num_areas;
    info_stream >> num_areas;
    area_str += std::to_string(num_areas) + "\n";
    for (int j = 0; j < num_areas; ++j)
    {
      int area;
      int num_nodes;
      int num_connections;
      info_stream >> area;
      info_stream >> num_nodes;
      // Skip min_node and max_node
      {
        int dummy;
        info_stream >> dummy; info_stream >> dummy; info_stream >> dummy; info_stream >> dummy;
      }
      info_stream >> num_connections;
      area_str += std::to_string(area) + " " + std::to_string(num_nodes) + " " + std::to_string(num_connections);

      for (int k = 0; k < num_connections; ++k)
      {
        int connected_area;
        int solver;
        int num_links;
        int num_corners;
        info_stream >> connected_area;
        info_stream >> solver;
        info_stream >> num_links;
        info_stream >> num_corners;
        area_str += " " + std::to_string(connected_area) + " " + std::to_string(solver) + " " +
                    std::to_string(num_links) + " " + std::to_string(num_corners);

        for (int l = 0; l < num_corners; ++l)
        {
          Coor corner;
          info_stream >> corner.x;
          info_stream >> corner.y;

          const auto corner_it = coor_id_map.find(corner);
          if (corner_it == coor_id_map.end())
          {
            exit_failure("Cannot identify corner coordinate (" + std::to_string(corner.x) + "," + std::to_string(corner.y) + ")");
          }

          area_str += " " + std::to_string(corner_it->second);
        }
      }

      area_str += "\n";
    }

    // Get robot info
    int num_areas_with_robot;
    int num_robots_total = 0;
    std::string robot_str;
    info_stream >> num_areas_with_robot;
    for (int j = 0; j < num_areas_with_robot; ++j)
    {
      int area_start;
      int num_robots;
      info_stream >> area_start;
      info_stream >> num_robots;
      num_robots_total += num_robots;

      for (int k = 0; k < num_robots; ++k)
      {
        int robot;
        int area_goal;
        Coor start;
        Coor goal;
        info_stream >> robot;
        info_stream >> area_goal;
        info_stream >> start.x;
        info_stream >> start.y;
        info_stream >> goal.x;
        info_stream >> goal.y;

        const auto start_it = coor_id_map.find(start);
        const auto goal_it = coor_id_map.find(goal);
        if (start_it == coor_id_map.end())
        {
          exit_failure("Cannot identify start coordinate (" + std::to_string(start.x) + "," + std::to_string(start.y) + ")");
        }
        if (goal_it == coor_id_map.end())
        {
          exit_failure("Cannot identify goal coordinate (" + std::to_string(goal.x) + "," + std::to_string(goal.y) + ")");
        }
        robot_str += std::to_string(robot) + " " + std::to_string(area_start) + " " + std::to_string(area_goal) + " " +
                     std::to_string(start_it->second) + " " + std::to_string(goal_it->second) + "\n";
      }
    }

    domains[i].header = std::to_string(num_robots_total) + "\n" + robot_str + area_str;
  }

  // Replace the input domain files
  for (int i = 1; i < argc; ++i)
  {
    if (write_file(argv[i], domains[i].header + domains[i].content))
    {
      std::cout << "Replaced " << argv[i] << std::endl;
    }
    else
    {
      exit_failure("Cannot write " + std::string(argv[i]));
    }
  }

  return EXIT_SUCCESS;
}
