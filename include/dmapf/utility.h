#ifndef DMAPF_INCLUDE_UTILITY_H_
#define DMAPF_INCLUDE_UTILITY_H_

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

void print_line()
{
  std::cout << "----------------------------------------------------------------" << std::endl;
}

std::string format_time(const int64_t ms)
{
  const int64_t minutes = ms / 60000;
  int64_t remainder = ms % 60000;
  const int64_t seconds = remainder / 1000;
  remainder = remainder % 1000;
  std::string padding;
  if (remainder < 10) padding = "00";
  else if (remainder < 100) padding = "0";

  return std::to_string(minutes) + "m" + std::to_string(seconds) + "." + padding + std::to_string(remainder) + "s";
}

bool copy_file(const std::string& src_path, std::string& dst_path)
{
  std::ifstream src(src_path, std::ios::binary);
  std::ofstream dst(dst_path, std::ios::binary);
  if (src.is_open() && dst.is_open())
  {
    dst << src.rdbuf();
    src.close();
    dst.close();
    return true;
  }
  return false;
}

bool read_file(const std::string& path, std::string& content)
{
  std::ifstream file(path, std::ios::binary);
  if (file.is_open())
  {
    std::stringstream buffer;
    buffer << file.rdbuf();
    content = buffer.str();
    file.close();
    return true;
  }
  return false;
}

bool append_file(const std::string& path, const std::string& content)
{
  std::ofstream file(path, std::ios_base::app);
  if (file.is_open())
  {
    file << content;
    file.close();
    return true;
  }
  return false;
}

bool write_file(const std::string& path, const std::string& content)
{
  std::ofstream file(path);
  if (file.is_open())
  {
    file << content;
    file.close();
    return true;
  }
  return false;
}

int countSubstring(const std::string& str, const std::string& sub)
{
  if (sub.length() == 0)
    return 0;

  int count = 0;
  for (size_t offset = str.find(sub); offset != std::string::npos; offset = str.find(sub, offset + sub.length()))
  {
    ++count;
  }
  return count;
}

std::string& ltrim(std::string& str, const std::string& chars = "\t\n\v\f\r ")
{
  str.erase(0, str.find_first_not_of(chars));
  return str;
}

std::string& rtrim(std::string& str, const std::string& chars = "\t\n\v\f\r ")
{
  str.erase(str.find_last_not_of(chars) + 1);
  return str;
}

std::string& trim(std::string& str, const std::string& chars = "\t\n\v\f\r ")
{
  return ltrim(rtrim(str, chars), chars);
}

void replaceSubstring(std::string& str, const std::string& sub, const std::string& replace)
{
  // Get the first occurrence
  size_t pos = str.find(sub);

  // Repeat till end is reached
  while (pos != std::string::npos)
  {
    // Replace this occurrence of Sub String
    str.replace(pos, sub.length(), replace);
    // Get the next occurrence from the current position
    pos = str.find(sub, pos + sub.length());
  }
}

std::vector<std::string> tokenizeString(const std::string& str, const std::string& delimiters = "(,)")
{
  std::vector<std::string> tokens;
  tokens.reserve(str.size() >> 1);

  // Skip delimiters at beginning.
  std::string::size_type lastPos = str.find_first_not_of(delimiters, 0);
  // Find first "non-delimiter".
  std::string::size_type pos = str.find_first_of(delimiters, lastPos);

  while (std::string::npos != pos || std::string::npos != lastPos)
  {
    // Found a token, add it to the vector.
    tokens.emplace_back(str.substr(lastPos, pos - lastPos));
    // Skip delimiters.  Note the "not_of"
    lastPos = str.find_first_not_of(delimiters, pos);
    // Find next "non-delimiter"
    pos = str.find_first_of(delimiters, lastPos);
  }
  return tokens;
}

#endif
