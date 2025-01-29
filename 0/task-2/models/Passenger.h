#ifndef PASSENGER_H
#define PASSENGER_H

#include <iostream>
#include <string>
#include "Person.h"

using namespace std;

class Passenger : public Person {
private:
    int cash = 0;
    bool ticket = false;

public:
    Passenger(string _name, string _surname, int _cash);

    bool buyTicket (int ticketPrice);

    bool operator!=(const Passenger& _passenger);

    bool getTicket ();
};

#endif // PASSENGER_H