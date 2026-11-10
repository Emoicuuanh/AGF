#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <pwd.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/types.h>
#include <unistd.h>
#include <vector>

std::string exec(const char* cmd)
{
  char buffer[128];
  std::string result = "";
  FILE* pipe = popen(cmd, "r");
  if (!pipe)
    throw std::runtime_error("popen() failed!");

  while (!feof(pipe))
  {
    if (fgets(buffer, sizeof(buffer), pipe) != nullptr)
      result += buffer;
  }

  pclose(pipe);
  return result;
}

std::string md5(const std::string& input)
{
  char buffer[33];
  memset(buffer, 0, sizeof(buffer));

  FILE* pipe = popen(("echo -n " + input + " | md5sum").c_str(), "r");
  if (!pipe)
    throw std::runtime_error("popen() failed!");

  if (fgets(buffer, sizeof(buffer), pipe) != nullptr)
  {
    // Remove trailing newline character and any leading whitespace
    buffer[strcspn(buffer, "\r\n")] = 0;
  }

  pclose(pipe);
  return buffer;
}

std::string getHomeDirectory()
{
  const char* homeDir;
  if ((homeDir = getenv("HOME")) == nullptr)
  {
    struct passwd* pwd = getpwuid(getuid());
    if (pwd != nullptr)
    {
      homeDir = pwd->pw_dir;
    }
    else
    {
      throw std::runtime_error("Cannot determine home directory");
    }
  }
  return homeDir;
}

bool getUbuntuSerialHash()
{
  std::string homeDir = getHomeDirectory();
  const char* command = "dmesg | grep UUID | grep 'Kernel' | sed "
                        "'s/.*UUID=//g' | sed 's/\\ ro\\ quiet.*//g'";
  std::string output = exec(command);
  std::string serial = output;

  // Trim leading and trailing whitespace
  serial = serial.substr(serial.find_first_not_of(" \t\n\r\f\v"),
                         serial.find_last_not_of(" \t\n\r\f\v") + 1);

  serial = "Mkac@" + serial + "2917";
  std::string hash_fr_user = md5(serial);
  // std::cout << "Hash fr user: " << hash_fr_user << std::endl;

  std::string final_txt = "1007" + hash_fr_user + "meik0!";
  std::string final_hash = md5(final_txt);
  // std::cout << "Final hash: " << final_hash << std::endl;

  std::ifstream file(homeDir + "/tmp/ros/license");
  if (file.is_open())
  {
    std::string hardware_serial;
    std::getline(file, hardware_serial);
    file.close();
    // std::cout << hardware_serial << std::endl;
    // Print debug(hardware_serial); // You should implement the print_debug function
    if (final_hash == hardware_serial)
    {
      return true;
    }
  }
  return false;
}

int main()
{
  if (getUbuntuSerialHash())
  {
    std::cout << "License verification passed." << std::endl;
  }
  else
  {
    std::cout << "License verification failed." << std::endl;
  }
  return 0;
}
