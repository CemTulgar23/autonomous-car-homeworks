#ifndef PERSON_H
#define PERSON_H

#include <iostream>
#include <string>

using namespace std;

class Person {
protected:
    string name = "";
    string surname = "";

public:
Person(string _name, string _surname);
};

#endif