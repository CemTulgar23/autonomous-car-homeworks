#ifndef MISSION_H
#define MISSION_H

#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <string> 
#include "Passenger.h"
#include "Astronaut.h"

using namespace std;

class Mission {
private:
    static int numMissions;
    string name = "AA-00";
    int missionNumber = 0;
    int cost = 0;
    int faultProbability = 0;
    bool completed = false;
    vector<Passenger*> passengers = {};
    vector<Astronaut*> crew = {};

public:
Mission(
 string _name,
 int _cost,
 int _faultProbability,
 vector<Passenger*>& _passengers,
 vector<Astronaut*>& _crew
 );

bool isMissionNameValid(const string& name);

void addSpaceShipStaf (Passenger& passenger,
    Astronaut& astronaut);

bool executeMission ();

int calculateProfit (int ticketPrice);

void missionInfo ();

int getMissionNumber ();

static int getNumMissions ();

};

#endif // MISSION_H