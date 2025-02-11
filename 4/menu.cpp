#include "menu.hpp"

using namespace menu;

// TASTEBALANCE
menu::TasteBalance::TasteBalance(
    double _sweet,
    double _sour,
    double _bitter,
    double _salty,
    double _savory) : sweet(_sweet),
                      sour(_sour),
                      bitter(_bitter),
                      salty(_salty),
                      savory(_savory) {}

TasteBalance menu::TasteBalance::from_json(const nlohmann::json &json)
{
    return TasteBalance(
        json.at("sweet").get<double>(),
        json.at("sour").get<double>(),
        json.at("bitter").get<double>(),
        json.at("salty").get<double>(),
        json.at("savory").get<double>());
}

double menu::TasteBalance::getSweet()
{
    return sweet;
}

double menu::TasteBalance::getSour()
{
    return sour;
}

double menu::TasteBalance::getBitter()
{
    return bitter;
}

double menu::TasteBalance::getSalty()
{
    return salty;
}

double menu::TasteBalance::getSavory()
{
    return savory;
}

// MENU
std::vector<std::shared_ptr<MenuItem>> &menu::Menu::getMenuItems()
{
    return menuItems;
}

double &menu::Menu::getTotalCost()
{
    return total_cost;
}

void menu::Menu::setTotalCost(const double newMenuItemCost)
{
    newMenuItemCost != 0 ? this->total_cost = this->total_cost + newMenuItemCost : this->total_cost = newMenuItemCost;
}

void menu::Menu::addMenu(std::vector<std::shared_ptr<MenuItem>> &menuItemList)
{
    menuItems = menuItemList;
}

// MENUITEM
menu::MenuItem::MenuItem(
    std::string _food_name,
    double _food_price,
    TasteBalance _food_taste_balance) : food_name(_food_name),
                                        food_price(_food_price),
                                        food_taste_balance(_food_taste_balance) {}

std::string menu::MenuItem::getFoodName() const
{
    return food_name;
}

double menu::MenuItem::getFoodPrice() const
{
    return food_price;
}

TasteBalance menu::MenuItem::getFoodTasteBalance() const
{
    return food_taste_balance;
}

MenuItem MenuItem::from_json(const nlohmann::json &json)
{
    return MenuItem(json.at("name").get<std::string>(),
                    json.at("price").get<double>(),
                    TasteBalance::from_json(json.at("taste_balance")));
}

/*
MenuItem &MenuItem::operator=(const MenuItem &other)
{
    if (this != &other) // Kendine atama kontrolü
    {
        food_name = other.food_name;
        food_price = other.food_price;
        food_taste_balance = other.food_taste_balance;
    }
    return *this;
}
*/

// STARTER
menu::Starter::Starter(
    std::string _food_name,
    double _food_price,
    TasteBalance _food_taste_balance,
    StarterHeat _starter_heat) : MenuItem(_food_name, _food_price, _food_taste_balance),
                                 starterHeat(_starter_heat) {}

StarterHeat menu::Starter::getStarterHeat() const
{
    return starterHeat;
}

void menu::Starter::setStarterHeat(const StarterHeat &newStarterHeat)
{
    this->starterHeat = newStarterHeat;
}

/*
// Kopyalama Kurucusu
Starter::Starter(const Starter &other)
    : MenuItem(other), starterHeat(other.starterHeat) {}

// Atama Operatörü
Starter &Starter::operator=(const Starter &other)
{
    if (this != &other) // Kendine atama kontrolü
    {
        MenuItem::operator=(other); // Üst sınıfı kopyala
        starterHeat = other.starterHeat;
    }
    return *this;
}
*/

/*
bool Starter::operator==(const std::shared_ptr<Starter> &other) const
{
    return this->getFoodName() == other->getFoodName();
}
*/

// SALAD
menu::Salad::Salad(
    std::string _food_name,
    double _food_price,
    TasteBalance _food_taste_balance,
    bool _isTopping) : MenuItem(_food_name, _food_price, _food_taste_balance),
                       isTopping(_isTopping) {}

bool menu::Salad::getIsTopping()
{
    return isTopping;
}

void menu::Salad::setIsTopping(const bool &newIsTopping)
{
    this->isTopping = newIsTopping;
}

/*
bool Salad::operator==(const std::shared_ptr<Salad> &other) const
{
    return this->getFoodName() == other->getFoodName();
}
*/

// MAINCOURSE
menu::MainCourse::MainCourse(
    std::string _food_name,
    double _food_price,
    TasteBalance _food_taste_balance) : MenuItem(_food_name, _food_price, _food_taste_balance) {}

/*
bool MainCourse::operator==(const std::shared_ptr<MainCourse> &other) const
{
    return this->getFoodName() == other->getFoodName();
}
*/

// DRINK
menu::Drink::Drink(
    std::string _food_name,
    double _food_price,
    TasteBalance _food_taste_balance,
    bool _isCarbonated,
    bool _isAdditionalAlcohol) : MenuItem(_food_name, _food_price, _food_taste_balance),
                                 isCarbonated(_isCarbonated),
                                 isAdditionalAlcohol(_isAdditionalAlcohol) {}

bool menu::Drink::getIsCarbonated()
{
    return isCarbonated;
}

bool menu::Drink::getIsAdditionalAlcohol()
{
    return isAdditionalAlcohol;
}

void menu::Drink::setIsCarbonated(const bool &newIsCarbonated)
{
    this->isCarbonated = newIsCarbonated;
}

void menu::Drink::setIsAdditionalAlcohol(const bool &newIsAdditionalAlcohol)
{
    this->isAdditionalAlcohol = newIsAdditionalAlcohol;
}

/*
bool Drink::operator==(const std::shared_ptr<Drink> &other) const
{
    return this->getFoodName() == other->getFoodName();
}
*/

// APPETIZER
menu::Appetizer::Appetizer(
    std::string _food_name,
    double _food_price,
    TasteBalance _food_taste_balance,
    AppetizerTime _appetizerTime) : MenuItem(_food_name, _food_price, _food_taste_balance),
                                    appetizerTime(_appetizerTime) {}

AppetizerTime menu::Appetizer::getAppetizerTime()
{
    return appetizerTime;
}

void menu::Appetizer::setAppetizerTime(const AppetizerTime &newAppetizerTime)
{
    this->appetizerTime = newAppetizerTime;
}

/*
bool Appetizer::operator==(const std::shared_ptr<Appetizer> &other) const
{
    return this->getFoodName() == other->getFoodName();
}
*/

// DESSERT
menu::Dessert::Dessert(
    std::string _food_name,
    double _food_price,
    TasteBalance _food_taste_balance,
    bool _isExtraChocolate) : MenuItem(_food_name, _food_price, _food_taste_balance),
                              isExtraChocolate(_isExtraChocolate) {}

void menu::Dessert::setIsExtraChocolate(const bool &newIsExtraChocolate)
{
    this->isExtraChocolate = newIsExtraChocolate;
}

bool menu::Dessert::getIsExtraChocolate()
{
    return isExtraChocolate;
}

/*
bool Dessert::operator==(const std::shared_ptr<Dessert> &other) const
{
    return this->getFoodName() == other->getFoodName();
}
*/

// USER
menu::User::User() : name(""), surname(""), gender(Gender::MALE) {}

menu::User::User(
    std::string _name,
    std::string _surname,
    Gender _gender) : name(_name),
                      surname(_surname),
                      gender(_gender) {}

Gender menu::User::getGender() const
{
    return gender;
}

Menu &menu::User::getMenu()
{
    return menu;
}

void menu::User::setName(std::string name)
{
    this->name = name;
}

void menu::User::setSurname(std::string surname)
{
    this->surname = surname;
}

void menu::User::setGender(Gender gender)
{
    this->gender = gender;
}
