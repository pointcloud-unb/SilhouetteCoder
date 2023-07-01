#ifndef CG_CONFIG_PARSER_H
#define CG_CONFIG_PARSER_H

#include <map>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

class ConfigParser {
    private:
        std::map<std::string, std::string> dic;
    public:
        void addEntry(std::string key, std::string value);
        std::map<std::string, std::string> getDic();
        void parseFile(std::string filename);
        void showDic();
};

#endif // CG_CONFIG_PARSER_H