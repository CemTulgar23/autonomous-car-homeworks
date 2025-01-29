#include <iostream>
#include <vector>

double sum(std::vector<int> a) {
    double res = 0;
    for (auto v : a) {
    res += v;
    }
    return res;
}
 
double sum(std::vector<double> a) {
    double res = 0;
    for (auto v : a) {
    res += v;
    }
    return res;
}
 
int main() {
    // ---- Solution 1 ----
    std::vector<int> list1 = {1,2,3,4,5};
    std::vector<double> list2 = {1.0,2.0,3.0,4.0,5.0};

    double res1 = sum(list1);
    double res2 = sum(list2);
    
    std::cout << res1 << std::endl;
    std::cout << res2 << std::endl;

    // ---- Solution 2 ----
    std::vector<double> list = {1, 2, 3, 4, 5};
    double res = sum(list);
    std::cout << res << std::endl;

}



