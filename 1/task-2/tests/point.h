#ifndef POINT_H
#define POINT_H

#include <cmath>
#include "religion.h"

// struct tıpkı class gibi sadece struct her zaman public.
struct Point {
    float x, y, z;

    Point (float _x, float _y, float _z);

    void printPoint();

    float zero_distance();
    
    // static -> nesne oluşturmadan doğrudan sınıf üzerinden o metoda erişmemizi sağlıyor.
    static float distance(Point point1, Point point2);

    static Point compare(Point point1, Point point2);

    Religion religion();

    static bool is_in_same_region(Point point1, Point point2);
};

#endif