#include <iostream>
#include "tests/point.h"

using namespace std;

int main() {
    Point point1 = Point(-1.0f,1.0f,1.0f);  
    Point point2 = Point(-2.0f,1.0f,1.0f);
    point1.zero_distance();
    point2.zero_distance();
    Point::distance(point1, point2);
    Point::compare(point1, point2);
    std::cout << "Religion: " << religionMap[point1.religion()] << std::endl;
    Point::is_in_same_region(point1, point2);
    return 0;
}

