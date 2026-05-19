/* atan2 example */
#include <stdio.h>      /* printf */
#include <math.h>       /* atan2 */

#define PI 3.14159265

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <boost/range/algorithm/count.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/tokenizer.hpp>

using Clock=std::chrono::high_resolution_clock;

// g++ -std=c++11 test.cpp && ./a.out

int countSubstring(const std::string& str, const std::string& sub)
{
  if (sub.length() == 0)
    return 0;

  int count = 0;
  for (size_t offset = str.find(sub); offset != std::string::npos; offset = str.find(sub, offset + sub.length()))
  {
    count++;
  }
  return count;
}

void replaceStringInPlace(std::string& str, const std::string& search, const std::string& replace)
{
  size_t pos = 0;
  while ((pos = str.find(search, pos)) != std::string::npos)
  {
    str.replace(pos, search.length(), replace);
    pos += replace.length();
  }
}

void replaceSubstring(std::string& str, const std::string& sub, const std::string& replace)
{
  // Get the first occurrence
  size_t pos = str.find(sub);

  // Repeat till end is reached
  while( pos != std::string::npos)
  {
    // Replace this occurrence of Sub String
    str.replace(pos, sub.length(), replace);
    // Get the next occurrence from the current position
    pos =str.find(sub, pos + sub.length());
  }
}

std::vector<std::string> tokenizeString(const std::string& str, const std::string& delimiters)
{  
   std::vector<std::string> tokens;
   tokens.reserve(str.size() >> 1);
   // Skip delimiters at beginning.
   std::string::size_type lastPos = str.find_first_not_of(delimiters, 0);
   // Find first "non-delimiter".
   std::string::size_type pos = str.find_first_of(delimiters, lastPos);

   while (std::string::npos != pos || std::string::npos != lastPos)
    {  // Found a token, add it to the vector.
      tokens.push_back(str.substr(lastPos, pos - lastPos));
      // Skip delimiters.  Note the "not_of"
      lastPos = str.find_first_not_of(delimiters, pos);
      // Find next "non-delimiter"
      pos = str.find_first_of(delimiters, lastPos);
   }
  return tokens;
}

void sub(int w, int* x, int& y)
{
  std::cout << &w << " = " << w << std::endl;
  std::cout << &x << " = " << x << std::endl;
  std::cout << &y << " = " << y << std::endl;
}

class A {
public:
  void f() {std::cout << "A::f ";}
  virtual void g() {std::cout << "A::g "; f();}
};

class B : public A{
public: 
  void f() {std::cout << "B::f ";}
  // void g() {std::cout << "B::g "; f();}
};

struct Problem
{
  int x;
  int y;
  int r;
  int col;
  int row;
};

class Foo
{
  public:
  static void bar()
  {
    // return baz();
    std::cout << 1 << std::endl;
  }

  static void bar(Foo& foo)
  {
    std::cout << 2 << std::endl;
  }

  // int baz()
  // {
  //   return 1;
  // }
};

std::string gen_random(const int len) {
    static const char alphanum[] =
        "0123456789"
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz";
    std::string tmp_s;
    tmp_s.reserve(len);

    for (int i = 0; i < len - 1; ++i) {
        tmp_s += alphanum[rand() % (sizeof(alphanum) - 1)];
    }
    tmp_s.push_back('.');

    return tmp_s;
}

int main (int argc, char *argv[])
{
  const int SIZE = 10000;
  const int STRING_SIZE = 10;
  std::vector<std::string> goals;
  for (int i = 0; i < SIZE; ++i)
  {
    goals.emplace_back(gen_random(STRING_SIZE));
  }

  std::string goal_str;
  for (const std::string& goal : goals)
  {
    goal_str += goal;
  }

  auto t1 = Clock::now();

  while (!goals.empty())
  {
    goal_str.clear();
    goals.pop_back();
    for (const std::string& goal : goals)
    {
      goal_str += goal;
    }
  }

//  int goal_length = SIZE * STRING_SIZE;
//  while (!goals.empty())
//  {
//    goals.pop_back();
//    goal_length -= STRING_SIZE;
//    goal_str = goal_str.substr(0, goal_length);
//  }

  auto t2 = Clock::now();

  std::cout << "Time: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << " ms" << std::endl;

//  std::ifstream src("dump.txt", std::ios::binary);
//  auto t1 = Clock::now();

  // std::stringstream buffer;
  // buffer << src.rdbuf();
  // src.close();

  // while(buffer) {
  //    int n;
  //    buffer >> n;
  // }

  //////////////////////////////////////////////////////////
  // std::stringstream buffer;
  // buffer << src.rdbuf();
  // src.close();

  // std::string content = buffer.str();
  // std::stringstream stream(content);

  // while(stream) {
  //    int n;
  //    stream >> n;
  // }

  //////////////////////////////////////////////////////////
//  std::stringstream buffer;
//  buffer << src.rdbuf();
//  src.close();
//
//  std::vector<std::string> tokens = tokenizeString(buffer.str(), " \n");
//  for (const std::string& str : tokens)
//  {
//    int n = std::stoi(str);
//  }

  //////////////////////////////////////////////////////////
  // std::stringstream buffer;
  // buffer << src.rdbuf();
  // src.close();

  // std::vector<std::string> tokens = tokenizeString(buffer.str(), "%");
  // std::stringstream stream(tokens[0]);

  // while(stream) {
  //    int n;
  //    stream >> n;
  // }

//  auto t2 = Clock::now();
//  std::cout << "Time: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << " ms" << std::endl;


  // std::vector<int> v{1,2,3,4,5};
  // std::cout << &v[0] << std::endl;
  // for (const auto& i : v)
  // {
  //   std::cout << &i << " : " << i << std::endl;
  // }

  // std::vector<int> c;
  // int* p = &v[0];
  // c.emplace_back(*p);
  // std::cout << &c[0] << " : " << c[0] << std::endl;

  // int&& a = 10;
  // int b;
  // a++;
  // std::cout << a << std::endl;
  // std::cout << &a << ", " << &b << std::endl;
  // std::cout << "argc = " << argc << std::endl;
  // std::cout << "========================" << std::endl;

  // const int SIZE = 200;
  // int count[SIZE] = {};
  // std::vector<int> factors[SIZE];

  // for (int i = 2; i < SIZE; i++)
  // {
  //   for (int j = i; j < SIZE; j += i)
  //   {
  //     count[j]++;
  //     factors[j].push_back(i);
  //   }
  // }

  // std::multimap<int, int, std::greater<int>> count_map;
  // for (int i = 2; i < SIZE; i++)
  // {
  //   count_map.insert(std::make_pair(count[i], i));
  // }

  // for (const auto& c : count_map)
  // {
  //   std::cout << c.first << " : " << c.second << " -";
  //   for (const auto& f : factors[c.second])
  //   {
  //     std::cout << " " << f;
  //   }
  //   std::cout << std::endl;
  // }

  // ////////
  // const std::string PARENT_DIR("/home/poom/catkin_ws/src/dmapf/examples/");
  // std::vector<Problem> problems;
  // problems.push_back({100, 100, 100, 10, 10});
  // problems.push_back({100, 100, 200, 10, 10});
  // problems.push_back({100, 100, 300, 10, 10});
  // problems.push_back({100, 100, 400, 10, 10});
  // problems.push_back({100, 100, 500, 10, 10});
  // problems.push_back({100, 100, 1000, 10, 10});
  // problems.push_back({25, 25, 25, 5, 5});

  // for (const auto& p : problems)
  // {
  //   const std::string DIR("x" + std::to_string(p.x) + "_y" + std::to_string(p.y) + "_n" + std::to_string(p.x * p.y) + "_r" + std::to_string(p.r) + "_c" + std::to_string(p.col) + "_r" + std::to_string(p.row));
  //   const std::string FILE("x" + std::to_string(p.x) + "_y" + std::to_string(p.y) + "_n" + std::to_string(p.x * p.y) + "_r" + std::to_string(p.r) + ".lp");

  //   std::cout << "mkdir " << PARENT_DIR << DIR << std::endl;
  //   std::cout << "rosrun dmapf generator _x:=" << p.x << " _y:=" << p.y << " _r:=" << p.r << " _out:=" << PARENT_DIR << DIR << std::endl;
  //   std::cout << "rosrun dmapf divider _c:=" << p.col << " _r:=" << p.row << " _in:=" << PARENT_DIR << DIR << "/" << FILE << std::endl;
  // }
  // ////////


  // std::vector<int> products = {4, 6, 1, 2, 10};
  // std::unordered_set<int> product_set{products.begin(), products.end()};
  // for (const auto& product : product_set)
  // {
  //   std::cout << product << std::endl;
  // }
  // std::cout << std::endl;

  // std::string* s = new std::string("poom");
  // std::cout << *s << std::endl;
  // delete s;
  // std::cout << *s << std::endl;

  // int w = 1, x = 2, y = 3;
  // std::cout << &w << " = " << w << std::endl;
  // std::cout << &x << " = " << x << std::endl;
  // std::cout << &y << " = " << y << std::endl;
  // std::cout << "----" << std::endl;
  // sub(w, &x, y);

  // A a;
  // B b;
  // a.f(); std::cout << std::endl;
  // a.g(); std::cout << std::endl;
  // b.f(); std::cout << std::endl;
  // b.g(); std::cout << std::endl;
  // std::cout << std::endl;

  // A* x = &b;
  // x->f(); std::cout << std::endl;
  // x->g(); std::cout << std::endl;
  // std::cout << std::endl;

  // B* y = &b;
  // y->f(); std::cout << std::endl;
  // y->g(); std::cout << std::endl;
  // std::cout << std::endl;

  // A* w = &a;
  // w->f(); std::cout << std::endl;
  // w->g(); std::cout << std::endl;
  // std::cout << std::endl;

  // char* A = "T";
  // printf("%ul\n", A);
  // printf("%ul\n", "T");

  // B* z = &a;
  // z->f(); std::cout << std::endl;
  // z->g(); std::cout << std::endl;

  // int* a = new int[10]();
  // std::cout << a[5] << std::endl;
  // delete[] a;


  // double x, y;
  // x = -10.0;
  // y = -10.0;
  // printf ("The arc tangent for (x=%f, y=%f) is %f radians\n", x, y, atan2(y,x));
  // printf ("The arc tangent for (x=%f, y=%f) is %f degrees\n", x, y, atan2(y,x) * 180.0 / PI);

  // std::string s("robot_position(robot(1),(1,3),1)");
  // auto t1 = Clock::now();
  // auto t2 = Clock::now();

  // std::string s1(s);
  // t1 = Clock::now();
  // boost::ireplace_all(s1, "robot_position", "position");
  // t2 = Clock::now();
  // std::cout << "s1 = " << s1 << std::endl;
  // std::cout << "Time: " << std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() << " ns" << std::endl;

  // std::string s2(s);
  // t1 = Clock::now();
  // replaceStringInPlace(s2, "robot_position", "position");
  // t2 = Clock::now();
  // std::cout << "s2 = " << s2 << std::endl;
  // std::cout << "Time: " << std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() << " ns" << std::endl;

  // std::string s3(s);
  // t1 = Clock::now();
  // replaceSubstring(s3, "robot_position", "position");
  // t2 = Clock::now();
  // std::cout << "s3 = " << s3 << std::endl;
  // std::cout << "Time: " << std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() << " ns" << std::endl;

  // ////////

  // std::vector<boost::iterator_range<std::string::const_iterator>> matches;
  // t1 = Clock::now();
  // boost::find_all(matches, s, "position");
  // t2 = Clock::now();
  // std::cout << "count = " << matches.size() << std::endl;
  // std::cout << "Time: " << std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() << " ns" << std::endl;

  // int count;
  // t1 = Clock::now();
  // count = countSubstring(s, "position");
  // t2 = Clock::now();
  // std::cout << "count = " << count << std::endl;
  // std::cout << "Time: " << std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() << " ns" << std::endl;

  // ////////

  // t1 = Clock::now();
  // boost::char_separator<char> sep{" (),."};
  // boost::tokenizer<boost::char_separator<char>> toks2{s, sep};
  // for (boost::tokenizer<boost::char_separator<char>>::const_iterator it = toks2.begin(); it != toks2.end(); it++);
  // t2 = Clock::now();
  // std::cout << "Time: " << std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() << " ns" << std::endl;

  // t1 = Clock::now();
  // std::vector<std::string> toks1 = tokenizeString(s, " (),.");
  // for (const auto& tok : toks1);
  // t2 = Clock::now();
  // std::cout << "Time: " << std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() << " ns" << std::endl;

  // ////////
  // std::unordered_map<int, int> map;
  // map.insert(std::make_pair(1,1));
  // map.insert(std::make_pair(1,1));
  // map.insert(std::make_pair(1,3));
  // map.insert(std::make_pair(1,2));

  // for (const auto& elem : map)
  // {
  //   std::cout << elem.first << " -> " << elem.second << std::endl;
  // }
  
  return 0;
}
