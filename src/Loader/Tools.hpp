/**
 * @file Tools.hpp
 * @author S'tout mo vie (gilou.assistant@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2020-01-20
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#pragma once

#include <fstream>
#include <iostream>
#include <vector>

namespace Loader
{

/**
 * @brief split std::string function
 * 
 * @param str 
 * @param delim 
 * @return std::vector<std::string> 
 */
std::vector<std::string> split(const std::string &str, const std::string &delim)
{
    std::vector<std::string> tokens;
    size_t prev = 0, pos = 0;
    do
    {
        pos = str.find(delim, prev);
        if (pos == std::string::npos)
            pos = str.length();
        std::string token = str.substr(prev, pos - prev);
        if (!token.empty())
            tokens.push_back(token);
        prev = pos + delim.length();
    } while (pos < str.length() && prev < str.length());
    return tokens;
}

/**
 * @brief Return the directory name of the input filename;
 * 
 * @param fname 
 * @return std::string 
 */
std::string dirnameOf(const std::string &fname)
{
    size_t pos = fname.find_last_of("\\/");
    return (std::string::npos == pos)
               ? ""
               : fname.substr(0, pos);
}

/**
 * @brief Return the file name of the input path;
 * 
 * @param fname 
 * @return std::string 
 */
std::string getFileName(const std::string &s)
{

    char sep = '/';

#ifdef _WIN32
    sep = '\\';
#endif

    size_t i = s.rfind(sep, s.length());
    if (i != std::string::npos)
    {
        std::string result = s.substr(i + 1, s.length() - i);
        return split(result, ".")[0];
    }

    return ("");
}

/**
 * @brief String to int conversion
 * 
 * @param str 
 * @param h 
 * @return constexpr unsigned int 
 */
constexpr unsigned int str2int(const char *str, int h = 0)
{
    return !str[h] ? 5381 : (str2int(str, h + 1) * 33) ^ str[h];
}

/**
 * @brief Function to parse all txt file
 * 
 * @param filename 
 * @return std::string 
 */
std::string readFile(const std::string &filename)
{
    std::fstream file;
    file.open(filename, std::fstream::in);
    std::string result = "";
    std::string line;
    while (!file.eof())
    {
        getline(file, line);
        result += line + "\n";
    }
    return result;
}

/**
 * @brief Save file
 * 
 * @param filename 
 * @param file 
 */
void saveFile(std::string filename, std::string file)
{
    std::ofstream stream;
    stream.open(filename);
    stream << file;
    stream.close();
}
} // namespace Loader