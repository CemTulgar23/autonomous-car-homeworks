#ifndef MENU_HPP
#define MENU_HPP

#include <iostream>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>

enum Gender
{
    MALE,
    FEMALE
};
enum class StarterHeat
{
    COLD,
    HOT,
    NONE
};
enum class AppetizerTime
{
    BEFORE,
    AFTER,
    NONE
};

namespace menu
{

    class TasteBalance
    {
    private:
        double sweet;
        double sour;
        double bitter;
        double salty;
        double savory;

    public:
        TasteBalance(double _sweet, double _sour, double _bitter, double _salty, double _savory);
        static TasteBalance from_json(const nlohmann::json &json);
        double getSweet();
        double getSour();
        double getBitter();
        double getSalty();
        double getSavory();
    };

    class MenuItem
    {
    private:
        std::string food_name;
        double food_price;
        TasteBalance food_taste_balance;

    public:
        MenuItem(std::string _food_name, double _food_price, TasteBalance _food_taste_balance);
        virtual ~MenuItem() = default; // Sanal yıkıcı (Destructor)
        static MenuItem from_json(const nlohmann::json &json);
        std::string getFoodName() const;
        double getFoodPrice() const;
        TasteBalance getFoodTasteBalance() const;
        // MenuItem &operator=(const MenuItem &other);
    };

    class Menu
    {
    private:
        std::vector<std::shared_ptr<MenuItem>> menuItems;
        double taste_balance;
        double total_cost;

    public:
        std::vector<std::shared_ptr<MenuItem>> &getMenuItems();

        double &getTotalCost();

        void setTotalCost(const double newMenuItemCost);

        void addMenu(std::vector<std::shared_ptr<MenuItem>> &menuItemList);

        void deleteMenu();

        void readMenu();

        void updateMenu(Menu newMenu);
    };

    class Starter : public MenuItem
    {
    private:
        StarterHeat starterHeat;

    public:
        Starter(std::string _food_name,
                double _food_price,
                TasteBalance _food_taste_balance,
                StarterHeat _starter_heat);

        StarterHeat getStarterHeat() const;
        void setStarterHeat(const StarterHeat &starterHeat);

        // Starter(const Starter &other);

        // Starter &operator=(const Starter &other);

        // bool operator==(const std::shared_ptr<Starter> &other) const;

        /*
        // KOPYALAMA ENGELLENDİ
        Starter(const Starter &other) = delete;
        Starter &operator=(const Starter &other) = delete;

        // TAŞIMA İZİN VERİLDİ
        Starter(Starter &&other) noexcept = default;
        Starter &operator=(Starter &&other) noexcept = default;
        */
    };

    class Salad : public MenuItem
    {
    private:
        bool isTopping;

    public:
        Salad(std::string _food_name,
              double _food_price,
              TasteBalance _food_taste_balance,
              bool _isTopping);

        bool getIsTopping();
        void setIsTopping(const bool &newIsTopping);

        // bool operator==(const std::shared_ptr<Salad> &other) const;
    };

    class MainCourse : public MenuItem
    {
    public:
        MainCourse(std::string _food_name,
                   double _food_price,
                   TasteBalance _food_taste_balance);

        // bool operator==(const std::shared_ptr<MainCourse> &other) const;
    };

    class Drink : public MenuItem
    {
    private:
        bool isCarbonated;
        bool isAdditionalAlcohol;

    public:
        Drink(std::string _food_name,
              double _food_price,
              TasteBalance _food_taste_balance,
              bool _isCarbonated,
              bool _isAdditionalAlcohol);

        bool getIsCarbonated();
        bool getIsAdditionalAlcohol();
        void setIsCarbonated(const bool &newIsCarbonated);
        void setIsAdditionalAlcohol(const bool &newIsAdditionalAlcohol);

        // bool operator==(const std::shared_ptr<Drink> &other) const;
    };

    class Appetizer : public MenuItem
    {
    private:
        AppetizerTime appetizerTime;

    public:
        Appetizer(std::string _food_name,
                  double _food_price,
                  TasteBalance _food_taste_balance,
                  AppetizerTime _appetizerTime);

        AppetizerTime getAppetizerTime();
        void setAppetizerTime(const AppetizerTime &newAppetizerTime);
        // bool operator==(const std::shared_ptr<Appetizer> &other) const;
    };

    class Dessert : public MenuItem
    {
    private:
        bool isExtraChocolate;

    public:
        Dessert(std::string _food_name,
                double _food_price,
                TasteBalance _food_taste_balance,
                bool _isExtraChocolate);

        void setIsExtraChocolate(const bool &newIsExtraChocolate);
        bool getIsExtraChocolate();

        // bool operator==(const std::shared_ptr<Dessert> &other) const;
    };

    class User
    {
    private:
        std::string name;
        std::string surname;
        Gender gender;
        Menu menu;

    public:
        User();
        User(std::string _name, std::string _surname, Gender _gender);
        Gender getGender() const;
        Menu &getMenu();
        void setName(std::string name);
        void setSurname(std::string surname);
        void setGender(Gender gender);
    };

}

#endif
