#ifndef AGENCY_H
#define AGENCY_H

#include <iostream>
#include <vector>
#include <string>
#include "Mission.h"

using namespace std;


class Agency {
private:
    string name = "";
    int cash = 0;
    int ticketPrice = 0;
    vector<Mission*> completedMissions = {};
    vector<Mission*> nextMissions = {};
public:
    Agency(
        string _name, 
        int _cash,
        int _ticketPrice,
        vector<Mission*>& _completedMissions, 
        vector<Mission*>& _nextMissions
    );

    void addMission (Mission& mission);

    void executeNextMission ();

    void agencyInfo (); 
};

#endif