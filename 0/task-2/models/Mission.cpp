#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <string>
#include "Mission.h"

using namespace std;


Mission::Mission(
 string _name,
 int _cost,
 int _faultProbability,
 vector<Passenger*>& _passengers,
 vector<Astronaut*>& _crew
 )
 :name(_name),
 cost(_cost),
 faultProbability(_faultProbability)
 {
    for(auto& passenger : _passengers) {
        passengers.push_back(passenger);
    }
    for(auto& astronaut : _crew) {
        crew.push_back(astronaut);
    }
    if(_faultProbability>100 || _faultProbability<0){
        throw runtime_error("MISSION COULDN'T CREATE! FAULTY PROBABLITY MUST BE BETWEN 0 AND 100.");
    }
    if(isMissionNameValid(name) == false){
        name = "AA-00";
        cout << "WRONG NAME FORMAT! ASSIGNED AUTOMATICALLY AS 'AA-00'." << endl;
    }
    numMissions ++;
    missionNumber = numMissions;
}

bool Mission::isMissionNameValid(const string& name) {
    if (name.length() != 5) return false;
    else if (!isupper(name[0]) || !isupper(name[1])) return false; 
    else if (name[2] != '-') return false;                        
    else if (!isdigit(name[3]) || !isdigit(name[4])) return false; 
    else return true;
}


void Mission::addSpaceShipStaf (Passenger& passenger,
    Astronaut& astronaut) {
    if(passenger != Passenger("","",0) && passenger.getTicket() == true) {
        passengers.push_back(&passenger);   
        cout << "Passenger added" << endl; 
    } else {
        cout << "FOR JOIN THE MISSIN PLEASE TAKE TICKET." << endl;
    }
    if (astronaut != Astronaut("","",0)) {
        crew.push_back(&astronaut);
        cout << "Astronaut added" << endl; 
    }
}

bool Mission::executeMission () {
    srand(static_cast<unsigned int>(time(0)));
    int random_number = rand() % 101;
    if (random_number > faultProbability) {
        cout << "MISSINON IS SUCCESSFUL!" << endl;
        for (auto& astronaut : crew) {
            astronaut->completeMission();
            astronaut->printAstronautInfos();
        }
        completed = true;
        return true;
    } else {
        cout << "MISSINON FAILD!" << endl;
        completed = false;
        return false;
    }
}

int Mission::calculateProfit(int ticketPrice) {
    int income = 0;
    if (completed == true){
        int passengerNumber = passengers.size();
        income = passengerNumber * ticketPrice;
    } 
    int profit = income - cost;
    return profit;
}

void Mission::missionInfo(){
    cout << "Mission Name: " << name << endl;
    cout << "Mission Number: " << missionNumber << endl;
    cout << "Mission Cost: " << cost << endl;
}

int Mission::getMissionNumber(){
    return missionNumber;
}

int Mission::getNumMissions(){
    return numMissions;
}


