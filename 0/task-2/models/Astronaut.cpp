#include <iostream>
//#include <Person.cpp>
#include <string>
#include "Astronaut.h"

using namespace std;

    Astronaut::Astronaut(string _name, string _surname, int _numMission)
    :Person(_name, _surname), numMission(_numMission) {}

    void Astronaut::completeMission () {
        numMission ++;
    }

    bool Astronaut::operator!=(const Astronaut& _astronaut) {
        return(this->name != _astronaut.name 
        || this->surname != _astronaut.surname 
        || this->numMission != _astronaut.numMission);
    }

    void Astronaut::printAstronautInfos() {
        cout << "Name: " << name << " Completed Mission: " << numMission << endl;
    }
