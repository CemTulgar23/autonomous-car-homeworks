#ifndef ASTRONAUT_H
#define ASTRONAUT_H

#include <iostream>
#include <string>
#include "Person.h"

using namespace std;

class Astronaut : public Person {
private:
    int numMission = 0;

public:
    Astronaut(string _name, string _surname, int _numMission);
    
    void completeMission ();

    bool operator!=(const Astronaut& _astronaut);

    void printAstronautInfos();
};

#endif