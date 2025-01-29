#include <iostream>
//#include <Person.cpp>
#include <string>
#include "Passenger.h"
#include "Person.h"

using namespace std;

Passenger::Passenger(string _name, string _surname, int _cash)
    :Person(_name, _surname), cash(_cash) {}

    bool Passenger::buyTicket (int ticketPrice) {
        if (cash >= ticketPrice) {
            cash = cash - ticketPrice;
            ticket = true;
            return true;
        } else {
            cout << "Bilet alabilmek için yeterli para yok." << endl;
            ticket = false;
            return false;
        }
    }

    bool Passenger::operator!=(const Passenger& _passenger){
        return(this->name != _passenger.name 
        || this->surname != _passenger.surname 
        || this->cash != _passenger.cash 
        || this->ticket != _passenger.ticket);
    }

    bool Passenger::getTicket(){
        return ticket;    
    }



