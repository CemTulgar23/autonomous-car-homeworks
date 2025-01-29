#include <iostream>
#include <iomanip>
#include <string>
#include <limits>

int main () {
    int int_v;
    int* ptr_int = &int_v;
    
    int64_t int64_t_v;
    int64_t* ptr_int64_t = &int64_t_v;
    int32_t int32_t_v;
    int32_t* ptr_int32_t = &int32_t_v;
    int16_t int16_t_v;
    int16_t* ptr_int16_t = &int16_t_v;
    int8_t int8_t_v;
    int8_t* ptr_int8_t = &int8_t_v;
    

    unsigned int u_int_v;
    uint* ptr_u_int = &u_int_v;
    
    uint64_t u_int64_t_v;
    uint64_t* ptr_u_int64_t = &u_int64_t_v;
    uint32_t u_int32_t_v;
    uint32_t* ptr_u_int32_t = &u_int32_t_v;
    uint16_t u_int16_t_v;
    uint16_t* ptr_u_int16_t = &u_int16_t_v;
    uint8_t u_int8_t_v;
    uint8_t* ptr_u_int8_t = &u_int8_t_v;

    long int l_int_v;
    long int* ptr_l_int = &l_int_v;
    unsigned long int u_l_int_v;
    unsigned long int* ptr_u_l_int = &u_l_int_v;
    long long_v;
    long* ptr_long = &long_v;
    unsigned long u_long_v;
    unsigned long* ptr_u_long = &u_long_v;
    long long l_long_v;
    long long* ptr_l_long = &l_long_v; 
    unsigned long long u_l_long_v;
    unsigned long long* ptr_u_l_long = &u_l_long_v; 
    short int s_int_v;
    short int* ptr_s_int = &s_int_v;
    unsigned short int u_s_int_v;
    unsigned short int* ptr_u_s_int = &u_s_int_v;
    short short_v;
    short* ptr_short = &short_v;
    unsigned short u_short_v;
    unsigned short* ptr_u_short = &u_short_v;
    double double_v;
    double* ptr_doubl= &double_v;
    long double l_double_v;
    long double* ptr_l_double = &l_double_v;
    float float_v;
    float* ptr_float = &float_v;    



    // unsigned sadece tam sayılar için kullanılır.



    std::cout << "" << std::endl;


    std::cout << std::setw(15) << std::left << "VARIABLES' SIZES" << std::endl;

    std::cout << std::setw(24) << std::left << "Type" 
              << std::setw(25) << "Size" << std::endl;

    std::cout << std::setw(40) << std::setfill('-') << "" << std::endl;

    // sizeof() -> değişkenin bellekte kapladığı yerin bayt cinsinden değerini verir.
    std::cout << std::setfill(' ')
              << std::setw(25) << std::left << "int"
              << std::setw(25) << sizeof(int_v) << std::endl;

    std::cout << std::setfill(' ')
              << std::setw(25) << std::left << "int8_t"
              << std::setw(25) << sizeof(int8_t) << std::endl;

    std::cout << std::setfill(' ')
              << std::setw(25) << std::left << "int16_t"
              << std::setw(25) << sizeof(int16_t) << std::endl;
    
    std::cout << std::setfill(' ')
              << std::setw(25) << std::left << "int32_t"
              << std::setw(25) << sizeof(int32_t) << std::endl;

    std::cout << std::setfill(' ')
              << std::setw(25) << std::left << "int64_t"
              << std::setw(25) << sizeof(int64_t) << std::endl;

    std::cout << std::setfill(' ')
              << std::setw(25) << std::left << "unsigned int"
              << std::setw(25) << sizeof(unsigned int) << std::endl;
    
    std::cout << std::setfill(' ')
              << std::setw(25) << std::left << "uint8_t"
              << std::setw(25) << sizeof(uint8_t) << std::endl;

    std::cout << std::setfill(' ')
              << std::setw(25) << std::left << "int8_t"
              << std::setw(25) << sizeof(int8_t) << std::endl;

    std::cout << std::setfill(' ')
              << std::setw(25) << std::left << "uint16_t"
              << std::setw(25) << sizeof(uint16_t) << std::endl;
    
    std::cout << std::setfill(' ')
              << std::setw(25) << std::left << "uint32_t"
              << std::setw(25) << sizeof(uint32_t) << std::endl;

    std::cout << std::setfill(' ')
              << std::setw(25) << std::left << "uint64_t"
              << std::setw(25) << sizeof(uint64_t) << std::endl;

    std::cout << std::setfill(' ')
              << std::setw(25) << std::left << "long int"
              << std::setw(25) << sizeof(long int) << std::endl;

    
    std::cout << std::setfill(' ')
              << std::setw(25) << std::left << "unsigned long int"
              << std::setw(25) << sizeof(unsigned long int) << std::endl;

    std::cout << std::setfill(' ')
              << std::setw(25) << std::left << "long"
              << std::setw(25) << sizeof(long) << std::endl;

    std::cout << std::setfill(' ')
              << std::setw(25) << std::left << "unsigned long"
              << std::setw(25) << sizeof(unsigned long) << std::endl;
    
    std::cout << std::setfill(' ')
              << std::setw(25) << std::left << "long long"
              << std::setw(25) << sizeof(long long) << std::endl;

    std::cout << std::setfill(' ')
              << std::setw(25) << std::left << "unsigned long long"
              << std::setw(25) << sizeof(unsigned long long) << std::endl;

    std::cout << std::setfill(' ')
              << std::setw(25) << std::left << "short int"
              << std::setw(25) << sizeof(short int) << std::endl;
    
    std::cout << std::setfill(' ')
              << std::setw(25) << std::left << "unsigned short int"
              << std::setw(25) << sizeof(unsigned short int) << std::endl;

    std::cout << std::setfill(' ')
              << std::setw(25) << std::left << "short"
              << std::setw(25) << sizeof(short) << std::endl;

    std::cout << std::setfill(' ')
              << std::setw(25) << std::left << "unsigned short"
              << std::setw(25) << sizeof(unsigned short) << std::endl;
    
    std::cout << std::setfill(' ')
              << std::setw(25) << std::left << "double"
              << std::setw(25) << sizeof(double) << std::endl;

    std::cout << std::setfill(' ')
              << std::setw(25) << std::left << "long double"
              << std::setw(25) << sizeof(long double) << std::endl;

    std::cout << std::setfill(' ')
              << std::setw(25) << std::left << "float"
              << std::setw(25) << sizeof(float) << std::endl;


    std::cout << "" << std::endl;


    std::cout << std::setw(15) << std::left << "VARIABLES' MAX AND MIN VALUES" << std::endl;

    std::cout << std::setw(35) << std::left << "Type" 
              << std::setw(35) << "Max" 
              << std::setw(35) << "Min" << std::endl;

    std::cout << std::setw(100) << std::setfill('-') << "" << std::endl;
   
    // sizeof() -> değişkenin bellekte kapladığı yerin bayt cinsinden değerini verir.
    std::cout << std::setfill(' ')
              << std::setw(33) << std::left << "int"
              << std::setw(33) << std::numeric_limits<int>::max()
              << std::setw(33) << std::numeric_limits<int>::min() << std::endl;

    std::cout << std::setfill(' ')
              << std::setw(33) << std::left << "int8_t"
              << std::setw(33) << static_cast<int>(std::numeric_limits<int8_t>::max())
              << std::setw(33) << static_cast<int>(std::numeric_limits<int8_t>::min()) << std::endl;

    std::cout << std::setfill(' ')
              << std::setw(33) << std::left << "int16_t"
              << std::setw(33) << std::numeric_limits<int16_t>::max()
              << std::setw(33) << std::numeric_limits<int16_t>::min() << std::endl;

    std::cout << std::setfill(' ')
              << std::setw(33) << std::left << "int32_t"
              << std::setw(33) << std::numeric_limits<int32_t>::max()
              << std::setw(33) << std::numeric_limits<int32_t>::min() << std::endl;
    
    std::cout << std::setfill(' ')
              << std::setw(33) << std::left << "int64_t"
              << std::setw(33) << std::numeric_limits<int64_t>::max()
              << std::setw(33) << std::numeric_limits<int64_t>::min() << std::endl;

    std::cout << std::setfill(' ')
              << std::setw(33) << std::left << "unsigned int"
              << std::setw(33) << std::numeric_limits<unsigned int>::max()
              << std::setw(33) << std::numeric_limits<unsigned int>::min() << std::endl;

    std::cout << std::setfill(' ')
              << std::setw(33) << std::left << "uint8_t"
              << std::setw(33) << static_cast<int>(std::numeric_limits<uint8_t>::max())
              << std::setw(33) << static_cast<int>(std::numeric_limits<uint8_t>::min()) << std::endl;

    std::cout << std::setfill(' ')
              << std::setw(33) << std::left << "uint16_t"
              << std::setw(33) << std::numeric_limits<uint16_t>::max()
              << std::setw(33) << std::numeric_limits<uint16_t>::min() << std::endl;

    std::cout << std::setfill(' ')
              << std::setw(33) << std::left << "uint32_t"
              << std::setw(33) << std::numeric_limits<uint32_t>::max()
              << std::setw(33) << std::numeric_limits<uint32_t>::min() << std::endl;
    
    std::cout << std::setfill(' ')
              << std::setw(33) << std::left << "uint64_t"
              << std::setw(33) << std::numeric_limits<uint64_t>::max()
              << std::setw(33) << std::numeric_limits<uint64_t>::min() << std::endl;

    std::cout << std::setfill(' ')
              << std::setw(33) << std::left << "long int"
              << std::setw(33) << std::numeric_limits<long int>::max()
              << std::setw(33) << std::numeric_limits<long int>::min() << std::endl;

    std::cout << std::setfill(' ')
              << std::setw(33) << std::left << "unsigned long int"
              << std::setw(33) << std::numeric_limits<unsigned long int>::max()
              << std::setw(33) << std::numeric_limits<unsigned long int>::min() << std::endl;

    std::cout << std::setfill(' ')
              << std::setw(33) << std::left << "long"
              << std::setw(33) << std::numeric_limits<long>::max()
              << std::setw(33) << std::numeric_limits<long>::min() << std::endl;

    std::cout << std::setfill(' ')
              << std::setw(33) << std::left << "unsigned long"
              << std::setw(33) << std::numeric_limits<unsigned long>::max()
              << std::setw(33) << std::numeric_limits<unsigned long>::min() << std::endl;
    
    std::cout << std::setfill(' ')
              << std::setw(33) << std::left << "long long"
              << std::setw(33) << std::numeric_limits<long long>::max()
              << std::setw(33) << std::numeric_limits<long long>::min() << std::endl;

    std::cout << std::setfill(' ')
              << std::setw(33) << std::left << "unsigned long long"
              << std::setw(33) << std::numeric_limits<unsigned long long>::max()
              << std::setw(33) << std::numeric_limits<unsigned long long>::min() << std::endl;

    std::cout << std::setfill(' ')
              << std::setw(33) << std::left << "short int"
              << std::setw(33) << std::numeric_limits<short int>::max()
              << std::setw(33) << std::numeric_limits<short int>::min() << std::endl;

    std::cout << std::setfill(' ')
              << std::setw(33) << std::left << "unsigned short int"
              << std::setw(33) << std::numeric_limits<unsigned short int>::max()
              << std::setw(33) << std::numeric_limits<unsigned short int>::min() << std::endl;

    std::cout << std::setfill(' ')
              << std::setw(33) << std::left << "short"
              << std::setw(33) << std::numeric_limits<short>::max()
              << std::setw(33) << std::numeric_limits<short>::min() << std::endl;

    std::cout << std::setfill(' ')
              << std::setw(33) << std::left << "unsigned short"
              << std::setw(33) << std::numeric_limits<unsigned short>::max()
              << std::setw(33) << std::numeric_limits<unsigned short>::min() << std::endl;
    
    std::cout << std::setfill(' ')
              << std::setw(33) << std::left << "double"
              << std::setw(33) << std::numeric_limits<double>::max()
              << std::setw(33) << std::numeric_limits<double>::min() << std::endl;

    std::cout << std::setfill(' ')
              << std::setw(33) << std::left << "long double"
              << std::setw(33) << std::numeric_limits<long double>::max()
              << std::setw(33) << std::numeric_limits<long double>::min() << std::endl;

    std::cout << std::setfill(' ')
              << std::setw(33) << std::left << "float"
              << std::setw(33) << std::numeric_limits<float>::max()
              << std::setw(33) << std::numeric_limits<float>::min() << std::endl;

}