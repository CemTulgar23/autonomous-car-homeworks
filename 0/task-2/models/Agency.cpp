#include <iostream>
#include <string>
#include "Agency.h"

using namespace std;

Agency::Agency(
    string _name, 
    int _cash, 
    int _ticketPrice, 
    vector<Mission *> &_completedMissions, 
    vector<Mission *> &_nextMissions)
    :name(_name),
    cash(_cash),
    ticketPrice(_ticketPrice){
        for(auto& mission : _completedMissions){
            completedMissions.push_back(mission);
        }
        for(auto& mission : _nextMissions){
            nextMissions.push_back(mission);
        }
    }

    void Agency::addMission(Mission& mission){
        cout << mission.getMissionNumber() << " numarali mission eklendi" << endl;
        nextMissions.push_back(&mission);
    }

    void Agency::executeNextMission(){
        Mission* missin = nextMissions.front();
        bool isSuccessful = missin->executeMission();
        if (isSuccessful == true) {
            completedMissions.push_back(missin);
            cout << " Listeden cikarildi" << endl;
        }
        nextMissions.erase(nextMissions.begin());
        int profit = missin->calculateProfit(ticketPrice);
        cout << profit << endl;
        cash = cash + profit;
        cout << cash << endl;
    }

    void Agency::agencyInfo(){
        cout << "Agency Name: " << name << endl;
        cout << "Agency Current Cash: " << cash << endl; 
        cout << "Agency Ticket Price: " << ticketPrice << endl;
        cout << "--Completed Missions Info--" << endl;
        for(Mission* mission : completedMissions){
            mission->missionInfo();
        }
        cout << "--Next Missions Info--" << endl;
        for(Mission* mission : nextMissions){
            mission->missionInfo();
        }
    }
