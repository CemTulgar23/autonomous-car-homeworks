#include <iostream>
#include <memory> // for using make_... and auto_ptr

class MyCLass
{
private:
    int value;

public:
    MyCLass(int _value) : value(_value)
    {
        std::cout << "MyClass Constructed has launched." << std::endl;
    };

    ~MyCLass()
    {
        std::cout << "MyCLass Destructer has launched." << std::endl;
    }

    const int &getValue() const
    {
        return value;
    }
};

class SmartPointer
{
private:
    MyCLass *ptr;

public:
    SmartPointer(int value)
    {
        ptr = new MyCLass(value);
        std::cout << "SmartPointer Constructed has launched." << std::endl;
    }
    ~SmartPointer()
    {
        delete ptr;
        std::cout << "SmartPointer Destructer has launched." << std::endl;
    }

    /*
    getter fonksiyonunu const olarak tanımladık çünkü getter fonksiyonu nesne üzerinde bir
    değişiklik yapmamalı ancak setter fonksyionunun nesnede değişiklik yapmasını istediğimiz için
    onu const olarak tanımlamıyoruz.
    */
    void setPtr(MyCLass *&_ptr)
    {
        this->ptr = _ptr;
    }

    MyCLass *&getPtr()
    {
        return ptr;
    }

    /*
    Fonksiyonun döndürdüğü değer olarak direkt T& yazsaydık o zaman bizim mevcut pointerımızı
    döndürmüş olurdu. Ama biz pointerı döndürmek değil pointerın işaret ettiği nesneyi döndürmek
    istiyoruz. Bu yüzden de T::element_type diyerek T nin bir altında bulunan değere erişmiş oluyoeuz.
    Ancak bu şekilde yazdığımızda derleyici bunun bir alt tür olduğunu anlamıyor ve typename kullanmamız gerekiyor.
    */
    MyCLass &operator*()
    {
        return *ptr; // ptr zaten bir pointer olduğundan doğrudan işaret ettiği nesneye erişiyoruz.
    }

    MyCLass *operator->()
    {
        return ptr;
    }
};

int main()
{

    {
        std::cout << "\n--------------MY SMART POINTER--------------\n" << std::endl;

        SmartPointer smartPtr(10);
        std::cout << "SmartPointer: " << smartPtr.getPtr() << std::endl;
        std::cout << "SmartPointer Value: " << (*smartPtr).getValue() << std::endl;
        std::cout << "SmartPointer Value: " << smartPtr->getValue() << std::endl;
    }

    {
        std::cout << "\n--------------SHARED POINTER--------------\n" << std::endl;

        std::shared_ptr<int> sharedPtr1 = std::make_shared<int>(20);
        std::shared_ptr<int> sharedPtr2 = sharedPtr1;
        std::shared_ptr<int> sharedPtr3 = sharedPtr1;
        std::cout << "Shared Pointer: " << sharedPtr1.get() << std::endl;
        std::cout << "Shared Pointer: " << sharedPtr2.get() << std::endl;
        std::cout << "Shared Pointer: " << sharedPtr3.get() << std::endl;
        std::cout << "Shared Pointer Value: " << *sharedPtr1 << std::endl;
        std::cout << "Shared Pointer Value: " << *sharedPtr2 << std::endl;
        std::cout << "Shared Pointer Value: " << *sharedPtr3 << std::endl;
        std::cout << "Shared Pointer Referance Number: " << sharedPtr1.use_count() << std::endl;
        sharedPtr3.reset();
        std::cout << "Shared Pointer New Referance Number: " << sharedPtr1.use_count() << std::endl;
        std::shared_ptr<int> sharedPtr4 = std::make_shared<int>(25);
        sharedPtr2 = std::move(sharedPtr4);
        std::cout << "Moved Shared Pointer Value: " << *sharedPtr2 << std::endl;
        std::cout << "Shared Pointer Last Referance Number: " << sharedPtr1.use_count() << std::endl;

    }

    {
        std::cout << "\n--------------UNIQUE POINTER--------------\n" << std::endl;

        std::unique_ptr<int> uniquePtr1 = std::make_unique<int>(30);
        std::cout << "Unique Pointer 1: " << uniquePtr1.get() << std::endl;
        std::cout << "Unique Pointer 1 Value: " << *uniquePtr1 << std::endl;
        std::unique_ptr<int> uniquePtr2 = std::move(uniquePtr1);
        std::cout << "Moved Unique Pointer: " << uniquePtr2.get() << std::endl;
        std::cout << "OLd Unique Pointer: " << uniquePtr1.get() << std::endl;
        std::cout << "Moved Unique Pointer Value: " << *uniquePtr2 << std::endl;
    }

    {
        std::cout << "\n--------------WEAK POINTER--------------\n" << std::endl;
        std::shared_ptr<int> sharedPtr = std::make_shared<int>(40);
        std::weak_ptr<int> weakPtr1 = sharedPtr;
        std::cout << "Weak Pointer: " << weakPtr1.lock().get() << std::endl;
        std::weak_ptr<int> weakPtr2 = std::move(weakPtr1);
        if(weakPtr1.expired()) {
           std::cout << "werakPtr1 is moved." << std::endl; 
        }
        std::cout << "New Weak Pointer Value: " << *weakPtr2.lock() << std::endl;

    }

    /*
    {
        std::cout << "\n--------------AUTO POINTER--------------\n" << std::endl;
        std::auto_ptr<int> autoPtr(new int(10));
        std::cout << "Moved Unique Pointer Value: " << *autoPtr << std::endl;    
    }
    */

    return 0;
}
