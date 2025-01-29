#include <iostream>
#include <string>
#include <vector>
#include "models/Mission.cpp"
#include "models/Passenger.cpp"
#include "models/Person.cpp"
#include "models/Astronaut.cpp"
#include "models/Agency.cpp"


using namespace std;


int Mission::numMissions = 0;

int main () {
    try {
        cout << "STARTED" << endl;
        Passenger passenger1 = Passenger("T","G",1000);
        Passenger passenger2 = Passenger("Mustafa","Kemal",2000);
        Astronaut astranout1 = Astronaut("Ali", "Fuat",0);
        Astronaut astranout2 = Astronaut("Ismet", "İnönü",0);
        vector<Passenger*> passengerList = {&passenger1, &passenger2};
        vector<Astronaut*> astranoutList = {&astranout1, &astranout2};
        Mission mission1 = Mission("TC-01",1000,0,passengerList,astranoutList);
        Passenger passenger3 = Passenger("Ayşe", "Banu",1500);
        passenger3.buyTicket(100);
        Astronaut astranout3 = Astronaut("Mehmet", "Akif",5);
        mission1.addSpaceShipStaf(passenger3, astranout3);
        Mission mission2 = Mission("TC-02",500,0,passengerList,astranoutList);
        vector<Mission*> completedMissionList = {}; 
        vector<Mission*> nextMissionList = {&mission1}; 
        Agency agency = Agency("Uzay",20000,100,completedMissionList,nextMissionList);
        agency.addMission(mission2);
        agency.executeNextMission();
        agency.agencyInfo();
    }
    catch(const runtime_error& e) {
        cout << "AN ERROR OCCURED: " << e.what() << endl;
        exit(1);
    }
    

}