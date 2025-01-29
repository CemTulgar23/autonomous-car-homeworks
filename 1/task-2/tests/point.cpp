#include <iostream>
#include <vector>
#include "point.h"

Point::Point (float _x, float _y, float _z): x(_x), y(_y), z(_z) {}

void Point::printPoint() {
    std::cout << "POINT" << std::endl;
    std::cout << "x: " << x << std::endl;
    std::cout << "y: " << y << std::endl;
    std::cout << "z: " << z << std::endl;
};

// sqrt -> karekök almayı sağlıyor.
// pow -> sayının üssünü almayı sağlıyor.
float Point::zero_distance () {
    float _zero_distance = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
    std::cout << "Zero Distance: " << _zero_distance << std::endl;
    return _zero_distance;
}

// static -> nesne oluşturmadan doğrudan sınıf üzerinden o metoda erişmemizi sağlıyor.
float Point::distance(Point point1, Point point2) {
    float _distance = sqrt(pow(point1.x - point2.x,2) + pow(point1.y - point2.y,2) + pow(point1.z - point2.z,2));
    std::cout << "Distance: " << _distance << std::endl;
    return _distance;
}

Point Point::compare(Point point1, Point point2) {
    float p1_z_dist = point1.zero_distance();
    float p2_z_dist = point2.zero_distance();
    Point point = p1_z_dist > p2_z_dist ? point1 : point2; // if komutunun kısa hali (Ternary Operator)
    point.printPoint();
    return point;
}

Religion Point::religion() {
    Religion religion;

    if (x < 0 && y < 0 && z < 0) {
        religion = REL7;
    }
    else if (x < 0 && y < 0 && z > 0) {
        religion = REL3;
    }
    else if (x < 0 && y > 0 && z < 0) {
        religion = REL6;
    }
    else if (x > 0 && y < 0 && z < 0) {
        religion = REL8;
    }
    else if (x < 0 && y > 0 && z > 0) {
        religion = REL2;
    }
    else if (x > 0 && y < 0 && z > 0) {
        religion = REL4;
    }
    else if (x > 0 && y > 0 && z < 0) {
        religion = REL5;
    }
    else if (x > 0 && y > 0 && z > 0) {
        religion = REL1;
    }
    else {
        religion = NONREL;
    }

    return religion;
}


bool Point::is_in_same_region(Point point1, Point point2) {
    bool status = point1.religion() == point2.religion() ? true : false;
    if (status == true) {
        std::cout << "They are in same religion." << std::endl;
    } else {
        std::cout << "They are not in same religion." << std::endl;
    }
    return status;
}
