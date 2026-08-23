#include "assetpack/Config.h"
#include <fstream>
#include <rapidjson/document.h>
#include <rapidjson/error/en.h>

namespace ap {

Config& Config::instance() {
    static Config cfg;
    return cfg;
}

bool Config::loadFromFile(const std::string& path) {
    std::ifstream f(path);
    if (!f) {
        std::ifstream f2("assetpack/config.json");
        if (!f2) return false;
        f.swap(f2);
    }
    std::string json((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    if (json.empty()) return false;
    rapidjson::Document d;
    d.Parse(json.c_str());
    if (d.HasParseError()) return false;
    if (d.HasMember("cacheImage") && d["cacheImage"].IsBool()) cacheImage = d["cacheImage"].GetBool();
    if (d.HasMember("cacheDir") && d["cacheDir"].IsString()) cacheDir = d["cacheDir"].GetString();
    return true;
}

bool Config::saveToFile(const std::string& path) const {
    std::ofstream f(path);
    if (!f) return false;
    f << "{\n";
    f << "    \"cacheImage\": " << (cacheImage ? "true" : "false") << ",\n";
    f << "    \"cacheDir\": \"" << cacheDir << "\"\n";
    f << "}\n";
    return true;
}

}
