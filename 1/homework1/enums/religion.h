#ifndef RELIGION_H
#define RELIGION_H

#include <map>

enum Religion {
    REL1,
    REL2,
    REL3,
    REL4,
    REL5,
    REL6,
    REL7,
    REL8,
    NONREL
};

extern std::map<Religion, std::string> religionMap;

#endif