#include "cg_config_parser.h"

void ConfigParser::addEntry(std::string key, std::string value){
    dic[key] = value;
}

std::map<std::string, std::string> ConfigParser::getDic(){
    return dic;
}

void ConfigParser::showDic(){
    for (auto itr = dic.begin(); itr != dic.end(); ++itr) {
        std::cout << itr->first << " = " << itr->second << '\n';
    }
}

void ConfigParser::parseFile(std::string filename){
    std::ifstream input(filename);

    std::string line;
    while(std::getline(input, line)){
        if(line[0] != '#'){
            std::istringstream is_line(line);
            std::string key;
            if(std::getline(is_line, key, '=') ){
                std::string value;
                if(std::getline(is_line, value)){
                    key.pop_back();
                    value.erase(value.begin());
                    addEntry(key, value);
                }
            }
        }
    }
}
