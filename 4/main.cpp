#include <iostream>
#include <nlohmann/json.hpp>
#include <fstream>
#include <vector>
#include <ncurses.h>
#include <cmath>
#include <random>
#include <functional>
#include <optional>
#include "menu.hpp"

using json = nlohmann::json;
using namespace menu;

int getRandomNumber(int min, int max)
{
    std::random_device rd;                             // Gerçek rastgelelik kaynağı
    std::mt19937 gen(rd());                            // Mersenne Twister rastgele sayı üreteci
    std::uniform_int_distribution<int> dist(min, max); // min ile max arasında sayı seç
    return dist(gen);
}

void showPopup(const std::string &message)
{
    int height = 5, width = message.length() + 6; // Pencere boyutları
    int startY = (LINES - height) / 2;            // Dikey ortalama
    int startX = (COLS - width) / 2;              // Yatay ortalama

    WINDOW *popup = newwin(height, width, startY, startX); // Yeni pencere oluştur
    box(popup, 0, 0);                                      // Kenarlık çiz

    mvwprintw(popup, 2, 3, "%s", message.c_str()); // Mesajı ekrana yaz
    wrefresh(popup);                               // Pencereyi güncelle

    getch(); // Kullanıcının bir tuşa basmasını bekle

    delwin(popup); // Pencereyi sil
}

int show_selectable_popup(const char *message, const char *selection_0, const char *selection_1)
{
    int width = strlen(message) + 50, height = 7;
    int startx = (COLS - width) / 2;
    int starty = (LINES - height) / 2;

    // Popup penceresini oluştur
    WINDOW *popup = newwin(height, width, starty, startx);
    box(popup, 0, 0); // Kenarlık çiz

    mvwprintw(popup, 1, (width - strlen(message)) / 2, "%s", message);

    int choice = 0; // 0: selection_0, 1: selection_1
    int key;

    // Arka planı güncelle
    refresh();
    wrefresh(popup);

    keypad(popup, TRUE); // Özel tuşları etkinleştir (ok tuşları vs.)

    while (true)
    {
        // Seçili olanı vurgula
        if (choice == 0)
        {
            wattron(popup, A_REVERSE);
            mvwprintw(popup, 3, width * 2 / 8, "%s", selection_0);
            wattroff(popup, A_REVERSE);
            mvwprintw(popup, 3, width * 5 / 8, "%s", selection_1);
        }
        else
        {
            mvwprintw(popup, 3, width * 2 / 8, "%s", selection_0);
            wattron(popup, A_REVERSE);
            mvwprintw(popup, 3, width * 5 / 8, "%s", selection_1);
            wattroff(popup, A_REVERSE);
        }

        wrefresh(popup);

        // Ana ekrandan değil, popup penceresinden giriş al
        key = getch();

        switch (key)
        {
        case KEY_LEFT:
        case 'a':
            choice = 0; // selection_0
            break;
        case KEY_RIGHT:
        case 'd':
            choice = 1; // selection_1
            break;
        case 10:           // Enter tuşu
            wclear(popup); // Pencereyi temizle
            wrefresh(popup);
            delwin(popup); // Pencereyi sil
            refresh();     // Ana ekranı güncelle
            return choice;
        }
    }
}

bool menuOperatorConditions(
    int &currentChoice,
    int &menuPartIndex,
    int &currentScreen,
    bool &changeScreen,
    int &choice,
    std::vector<std::shared_ptr<MenuItem>> &foodList,
    std::vector<std::shared_ptr<Starter>> &chosenStarters,
    std::vector<std::shared_ptr<Salad>> &chosenSalads,
    std::vector<std::shared_ptr<MainCourse>> &chosenMainCourses,
    std::vector<std::shared_ptr<Appetizer>> &chosenAppetizers,
    std::vector<std::shared_ptr<Drink>> &chosenDrinks,
    std::vector<std::shared_ptr<Dessert>> &chosenDesserts,
    User &user)
{
    if (currentChoice == foodList.size()) // Clear Menu
    {
        chosenStarters.clear();
        chosenSalads.clear();
        chosenMainCourses.clear();
        chosenAppetizers.clear();
        chosenDrinks.clear();
        chosenDesserts.clear();
        changeScreen = true;
        menuPartIndex = 0;
        user.getMenu().setTotalCost(0);
        return true;
    }
    if (currentChoice == foodList.size() + 1) // Exit Menu
    {
        currentScreen = -1;
        changeScreen = true;
        return true;
    }
    if (currentChoice == foodList.size() + 2) // Commit Menu
    {
        if (!chosenStarters.empty() ||
            !chosenSalads.empty() ||
            !chosenMainCourses.empty() ||
            !chosenAppetizers.empty() ||
            !chosenDrinks.empty() ||
            !chosenDesserts.empty())
        {
            showPopup("Your orders will be served in the short time. Enjoy your meal!");
            refresh();
            endwin();
            return 0;
        }
        else
        {
            showPopup("There aren't any chosen foods.");
        }
        return true;
    }
    else
    {
        return false;
    }
}

void menuDidYouLikeOperators(
    bool &changeScreen,
    int &currentChoice,
    int &key,
    int &currentScreen,
    double &totalCost,
    std::vector<std::shared_ptr<MenuItem>> &menuItemList,
    User &user,
    const std::optional<int> &extraToppingChoice,
    const std::optional<int> &carbonatedChoice,
    const std::optional<int> &alcoholChoice,
    const std::optional<int> &chocolateChoice)
{
    attron(COLOR_PAIR(2));
    attron(A_BOLD);
    mvprintw(11, 2, "%s", "Did you like the menu?");
    attroff(COLOR_PAIR(2));
    attroff(A_BOLD);
    while (!changeScreen)
    {
        if (currentChoice == 0)
        {
            attron(A_REVERSE);
        }
        mvprintw(12, 2, "%s", "Yes, I want to order the menu.");
        attroff(A_REVERSE);
        if (currentChoice == 1)
        {
            attron(A_REVERSE);
        }
        mvprintw(13, 2, "%s", "No, I want to try again.");
        attroff(A_REVERSE);
        if (currentChoice == 2)
        {
            attron(A_REVERSE);
        }
        mvprintw(14, 2, "%s", "Back to Main Page");
        attroff(A_REVERSE);

        key = getch();

        switch (key)
        {
        case KEY_UP:
            if (currentChoice > 0)
                currentChoice--; // Yukarı git
            break;
        case KEY_DOWN:
            if (currentChoice < 2)
                currentChoice++; // Aşağı git
            break;
        case 10:
            if (currentChoice == 0) // order
            {

                if (extraToppingChoice == 0)
                {
                    user.getMenu().setTotalCost(2.25);
                }
                if (carbonatedChoice == 0)
                   // key = getch();
             {
                    user.getMenu().setTotalCost(0.5);
                }
                if (alcoholChoice == 0)
                {
                    user.getMenu().setTotalCost(2.5);
                }
                if (chocolateChoice == 0)
                {
                    user.getMenu().setTotalCost(1.5);
                }

                user.getMenu().setTotalCost(totalCost);
                user.getMenu().addMenu(menuItemList);
                showPopup("Your orders will be served in the short time. Enjoy your meal!");
                endwin();
                exit(0);
            }
            if (currentChoice == 1) // try again
            {
                changeScreen = true;
            }
            if (currentChoice == 2) // back to the main page
            {
                currentScreen = -1;
                changeScreen = true;
            }
            break;
        case 27:
            currentScreen = -1;
            changeScreen = true;
            break;
        }
    }
}

void printMenuOperatorButtons(
    int &currentChoice,
    std::vector<std::shared_ptr<MenuItem>> &foodList)
{
    if (currentChoice == foodList.size())
    {
        attron(A_REVERSE);
    }
    mvprintw(foodList.size() + 4, 2, "%s", "Clear Menu");
    attroff(A_REVERSE);

    if (currentChoice == foodList.size() + 1)
    {
        attron(A_REVERSE);
    }
    mvprintw(foodList.size() + 5, 2, "%s", "Exit Menu");
    attroff(A_REVERSE);

    if (currentChoice == foodList.size() + 2)
    {
        attron(A_REVERSE);
    }
    mvprintw(foodList.size() + 6, 2, "%s", "Commit Menu");
    attroff(A_REVERSE);
}


int main()
{
    std::ifstream inputFile("menu.json"); // JSON dosyasını aç
    json jsonData;
    inputFile >> jsonData; // JSON dosyasını değişkene aktar
    inputFile.close();     // Dosyayı kapat

    std::vector<std::shared_ptr<Starter>> starterList;
    std::vector<std::shared_ptr<Salad>> saladList;
    std::vector<std::shared_ptr<MainCourse>> vegetarianMainCourseList;
    std::vector<std::shared_ptr<MainCourse>> non_vegetarianMainCourseList;
    std::vector<std::shared_ptr<MainCourse>> allMainCoursesList;
    std::vector<std::shared_ptr<Appetizer>> appetizerList;
    std::vector<std::shared_ptr<Drink>> drinkList;
    std::vector<std::shared_ptr<Dessert>> dessertList;

    std::vector<std::shared_ptr<Starter>> chosenStarters;
    std::vector<std::shared_ptr<Salad>> chosenSalads;
    std::vector<std::shared_ptr<MainCourse>> chosenMainCourses;
    std::vector<std::shared_ptr<Appetizer>> chosenAppetizers;
    std::vector<std::shared_ptr<Drink>> chosenDrinks;
    std::vector<std::shared_ptr<Dessert>> chosenDesserts;

    // STARTER
    for (const auto &starter : jsonData["starters"])
    {
        std::shared_ptr<MenuItem> newMenuItem = std::make_shared<MenuItem>(MenuItem::from_json(starter));
        std::shared_ptr<Starter> newStarter = std::make_shared<Starter>(
            newMenuItem->getFoodName(),
            newMenuItem->getFoodPrice(),
            newMenuItem->getFoodTasteBalance(),
            StarterHeat::NONE);
        starterList.push_back(newStarter);
    }

    // SALAD
    for (const auto &salad : jsonData["salads"])
    {
        std::shared_ptr<MenuItem> newMenuItem = std::make_shared<MenuItem>(MenuItem::from_json(salad));
        std::shared_ptr<Salad> newSalad = std::make_shared<Salad>(
            newMenuItem->getFoodName(),
            newMenuItem->getFoodPrice(),
            newMenuItem->getFoodTasteBalance(),
            false);
        saladList.push_back(newSalad);
    }

    // MAINCOURSE
    for (const auto &mainCourse : jsonData["main_courses"])
    {
        std::shared_ptr<MenuItem> newMenuItem = std::make_shared<MenuItem>(MenuItem::from_json(mainCourse));
        std::shared_ptr<MainCourse> newMainCourse = std::make_shared<MainCourse>(
            newMenuItem->getFoodName(),
            newMenuItem->getFoodPrice(),
            newMenuItem->getFoodTasteBalance());
        allMainCoursesList.push_back(newMainCourse);
        if (newMainCourse->getFoodName() == "Pesto Pasta")
        {
            vegetarianMainCourseList.push_back(newMainCourse);
        }
        if (newMainCourse->getFoodName() == "Vegetable Stir-Fry")
        {
            vegetarianMainCourseList.push_back(newMainCourse);
        }
        if (newMainCourse->getFoodName() == "Beef Stroganoff")
        {
            vegetarianMainCourseList.push_back(newMainCourse);
        }
        if (newMainCourse->getFoodName() == "Chicken Alfredo")
        {
            non_vegetarianMainCourseList.push_back(newMainCourse);
        }
        if (newMainCourse->getFoodName() == "Grilled Salmon")
        {
            non_vegetarianMainCourseList.push_back(newMainCourse);
        }
    }

    // APPETIZER
    for (const auto &appetizer : jsonData["appetizers"])
    {
        std::shared_ptr<MenuItem> newMenuItem = std::make_shared<MenuItem>(MenuItem::from_json(appetizer));
        std::shared_ptr<Appetizer> newAppetizer = std::make_shared<Appetizer>(
            newMenuItem->getFoodName(),
            newMenuItem->getFoodPrice(),
            newMenuItem->getFoodTasteBalance(),
            AppetizerTime::NONE);
        appetizerList.push_back(newAppetizer);
    }

    // DRINK
    for (const auto &drink : jsonData["drinks"])
    {
        std::shared_ptr<MenuItem> newMenuItem = std::make_shared<MenuItem>(MenuItem::from_json(drink));
        std::shared_ptr<Drink> newDrink = std::make_shared<Drink>(
            newMenuItem->getFoodName(),
            newMenuItem->getFoodPrice(),
            newMenuItem->getFoodTasteBalance(),
            false,
            false);
        drinkList.push_back(newDrink);
    }

    // DESSERT
    for (const auto &dessert : jsonData["desserts"])
    {
        std::shared_ptr<MenuItem> newMenuItem = std::make_shared<MenuItem>(MenuItem::from_json(dessert));
        std::shared_ptr<Dessert> newDessert = std::make_shared<Dessert>(
            newMenuItem->getFoodName(),
            newMenuItem->getFoodPrice(),
            newMenuItem->getFoodTasteBalance(),
            false);
        dessertList.push_back(newDessert);
    }

    initscr();            // ncurses başlat
    start_color();        // Renkleri etkinleştir
    curs_set(1);          // İmleci göster
    keypad(stdscr, TRUE); // Özel tuşları etkinleştir (Yön tuşları gibi)

    init_pair(1, COLOR_RED, COLOR_BLACK);
    init_pair(2, COLOR_BLUE, COLOR_BLACK);

    int height = 5, width = 20;
    int startx = 0, starty = 0;
    WINDOW *veg_win = newwin(height, width, starty, startx);                // Vegetarian sekmesi
    WINDOW *nonveg_win = newwin(height, width, starty, startx + width + 1); // Non-Vegetarian sekmesi

    User user;
    int key;
    int currentChoice = 0; // Seçili öğe indeksi
    bool changeScreen = false;
    char name[50];
    char surname[50];
    Gender gender;

    mvprintw(0, 2, "%s", "Hello! I'm intelligent restaurant bot. I will help you to choose your best food."); // Öğeyi yazdır
    mvprintw(1, 2, "%s", "Can I learn your name? ");
    refresh();
    getnstr(name, 49);
    mvprintw(2, 2, "%s", "Can I learn your surname? ");
    refresh();
    getnstr(surname, 49);
    noecho();    // Kullanıcının girdiğini gizle
    curs_set(0); // İmleci gizle
    mvprintw(3, 2, "%s", "Finally, please select your gender.");
    while (!changeScreen)
    {
        if (currentChoice == 0)
        {
            attron(A_REVERSE);
        }
        mvprintw(4, 2, "%s", "Male");
        attroff(A_REVERSE);
        if (currentChoice == 1)
        {
            attron(A_REVERSE);
        }
        mvprintw(5, 2, "%s", "Female");
        attroff(A_REVERSE);

        refresh();

        key = getch();

        switch (key)
        {
        case KEY_UP:
            if (currentChoice > 0)
                currentChoice--; // Yukarı git
            break;
        case KEY_DOWN:
            if (currentChoice < 1)
                currentChoice++; // Aşağı git
            break;
        case 10:
            user.setName(name);
            user.setSurname(surname);
            if (currentChoice == 0)
            {
                user.setGender(Gender::MALE);
            }
            else
            {
                user.setGender(Gender::FEMALE);
            }
            changeScreen = true;
            break;
        }
    }
    changeScreen = false;
    currentChoice = 0;
    clear();

    int currentScreen = 0;
    int menuPartIndex = 0;
    int choice;

    while (true)
    {
        clear();
        if (user.getGender() == Gender::MALE)
        {
            std::string message = "Hello again Mr. " + std::string(name) + " " + std::string(surname);
            mvprintw(0, 2, "%s", message.c_str());
        }
        else
        {
            std::string message = "Hello again Ms. " + std::string(name) + " " + std::string(surname);
            mvprintw(0, 2, "%s", message.c_str());
        }
        refresh();

        if (!user.getMenu().getMenuItems().empty())
        {
            for (const auto &menuItem : user.getMenu().getMenuItems())
            {
                for (const auto &starter : starterList)
                {
                    if (starter->getFoodName() == menuItem->getFoodName())
                    {
                        chosenStarters.push_back(starter);
                    }
                }
                for (const auto &salad : saladList)
                {
                    if (salad->getFoodName() == menuItem->getFoodName())
                    {
                        chosenSalads.push_back(salad);
                    }
                }
                for (const auto &vegetarianMainCourse : vegetarianMainCourseList)
                {
                    if (vegetarianMainCourse->getFoodName() == menuItem->getFoodName())
                    {
                        chosenMainCourses.push_back(vegetarianMainCourse);
                    }
                }
                for (const auto &non_vegetarianMainCourse : non_vegetarianMainCourseList)
                {
                    if (non_vegetarianMainCourse->getFoodName() == menuItem->getFoodName())
                    {
                        chosenMainCourses.push_back(non_vegetarianMainCourse);
                    }
                }
                for (const auto &drink : drinkList)
                {
                    if (drink->getFoodName() == menuItem->getFoodName())
                    {
                        chosenDrinks.push_back(drink);
                    }
                }
                for (const auto &appetizer : appetizerList)
                {
                    if (appetizer->getFoodName() == menuItem->getFoodName())
                    {
                        chosenAppetizers.push_back(appetizer);
                    }
                }
                for (const auto &dessert : dessertList)
                {
                    if (dessert->getFoodName() == menuItem->getFoodName())
                    {
                        chosenDesserts.push_back(dessert);
                    }
                }
            }
        }

        if (currentScreen == 0)
        {
            attron(A_REVERSE);
        }
        mvprintw(2, 2, "%s", "Menu");
        attroff(A_REVERSE);
        if (currentScreen == 1)
        {
            attron(A_REVERSE);
        }
        mvprintw(3, 2, "%s", "Suggested Menu");
        attroff(A_REVERSE);
        if (currentScreen == 2)
        {
            attron(A_REVERSE);
        }
        mvprintw(4, 2, "%s", "Random Menu");
        attroff(A_REVERSE);
        if (currentScreen == 3)
        {
            attron(A_REVERSE);
        }
        mvprintw(5, 2, "%s", "Leave Restaurant");
        attroff(A_REVERSE);

        refresh();

        key = getch();

        switch (key)
        {
        case KEY_UP:
            if (currentScreen > 0)
                currentScreen--; // Yukarı git
            break;
        case KEY_DOWN:
            if (currentScreen < 3)
                currentScreen++; // Aşağı git
            break;
        case 10: // enter

            while (currentScreen == 0)
            {
                clear();
                changeScreen = false;
                bool vegetarianSelected;
                switch (menuPartIndex)
                {
                case 0: // Starters
                    currentChoice = 0;
                    while (!changeScreen)
                    {
                        clear();
                        attron(A_BOLD);
                        attron(COLOR_PAIR(2));
                        attron(A_UNDERLINE);
                        mvwprintw(stdscr, 0, 2, "%s", "STARTERS");
                        attroff(A_UNDERLINE);
                        attroff(COLOR_PAIR(2));
                        attroff(A_BOLD);

                        // Menü öğelerini ekrana yazdır
                        for (int i = 0; i < starterList.size(); i++)
                        {
                            if (!chosenStarters.empty())
                            {

                                for (const auto &chosenStarter : chosenStarters)
                                {
                                    if (chosenStarter->getFoodName() == starterList[i]->getFoodName())
                                    {
                                        attron(A_REVERSE);
                                        break;
                                    }
                                }
                                if (currentChoice == i)
                                {
                                    attron(COLOR_PAIR(1));
                                }

                                std::string starterHeat;
                                if (starterList[i]->getStarterHeat() == StarterHeat::COLD)
                                {
                                    starterHeat = "(Cold)";
                                }
                                if (starterList[i]->getStarterHeat() == StarterHeat::HOT)
                                {
                                    starterHeat = "(Hot)";
                                }
                                if (starterList[i]->getStarterHeat() == StarterHeat::NONE)
                                {
                                    starterHeat = "";
                                }

                                std::string message = std::to_string(starterList[i]->getFoodPrice()) + "$ " + starterList[i]->getFoodName() + " " + starterHeat;
                                mvprintw(i + 1, 2, "%s", message.c_str()); // Öğeyi yazdır
                                attroff(A_REVERSE);
                                attroff(COLOR_PAIR(1));
                            }
                            else
                            {
                                if (currentChoice == i)
                                {
                                    attron(COLOR_PAIR(1)); // Seçili öğeyi kırmızı yap
                                }
                                std::string message = std::to_string(starterList[i]->getFoodPrice()) + "$ " + starterList[i]->getFoodName();
                                mvprintw(i + 1, 2, "%s", message.c_str()); // Öğeyi yazdır
                                attroff(COLOR_PAIR(1));
                            }
                        }
                        std::string message = std::to_string(user.getMenu().getTotalCost()) + "$ ";
                        mvprintw(starterList.size() + 2, 2, "%s", message.c_str());

                        std::vector<std::shared_ptr<menu::MenuItem>> menuItemList(starterList.begin(), starterList.end());
                        printMenuOperatorButtons(currentChoice, menuItemList);

                        refresh();
                        key = getch(); // Kullanıcının bastığı tuşu al

                        switch (key)
                        {
                        case 27: // esc

                            currentScreen = -1;
                            changeScreen = true;

                            break;

                        case KEY_UP:
                            if (currentChoice > 0)
                            {
                                currentChoice--; // Yukarı git
                            }
                            break;

                        case KEY_DOWN:
                            if (currentChoice < starterList.size() + 2)
                            {
                                currentChoice++; // Aşağı git
                            }
                            break;

                        case KEY_RIGHT:
                            if (menuPartIndex < 5)
                            {
                                menuPartIndex++;
                                changeScreen = true;
                            }
                            break;

                        case KEY_LEFT:
                            if (menuPartIndex > 0)
                            {
                                menuPartIndex--;
                                changeScreen = true;
                            }
                            break;

                        case 10: // Enter tuşu

                            /*
                            auto result = std::find(chosenStarters.begin(), chosenStarters.end(), starterList[currentChoice]);
                            if (result != chosenStarters.end()) // yemek zaten daha önceden seçilmişse
                            {
                                for (int i = 0; i < chosenStarters.size(); i++)
                                {
                                    if (chosenStarters[i]->getFoodName() == starterList[currentChoice]->getFoodName()) // listede seçilen yemeği bul
                                    {
                                        // yemeği seçili durumdan kaldır
                                        user.getMenu().setTotalCost(-starterList[currentChoice]->getFoodPrice());
                                        starterList[currentChoice]->setStarterHeat(StarterHeat::NONE);
                                        chosenStarters.erase(chosenStarters.begin() + i);
                                    }
                                }
                            }
                            else // yemek daha önceden seçili değilse
                            {
                                std::string message = "How do you want to your " + starterList[currentChoice]->getFoodName() + "?";
                                int choice = show_selectable_popup(message.c_str(), "Cold", "Hot");
                                if (choice == 0) // cold
                                {
                                    starterList[currentChoice]->setStarterHeat(StarterHeat::COLD);
                                }
                                if (choice == 1) // hot
                                {
                                    starterList[currentChoice]->setStarterHeat(StarterHeat::HOT);
                                }
                                user.getMenu().setTotalCost(starterList[currentChoice]->getFoodPrice());
                                chosenStarters.push_back(starterList[currentChoice]);
                            }
                            */

                            std::vector<std::shared_ptr<menu::MenuItem>> menuItemList(starterList.begin(), starterList.end());
                            if (menuOperatorConditions(
                                    currentChoice,
                                    menuPartIndex,
                                    currentScreen,
                                    changeScreen,
                                    choice,
                                    menuItemList,
                                    chosenStarters,
                                    chosenSalads,
                                    chosenMainCourses,
                                    chosenAppetizers,
                                    chosenDrinks,
                                    chosenDesserts,
                                    user) != true)
                            {
                                auto result = std::find(chosenStarters.begin(), chosenStarters.end(), starterList[currentChoice]);
                                if (result != chosenStarters.end()) // yemek zaten daha önceden seçilmişse
                                {
                                    for (int i = 0; i < chosenStarters.size(); i++)
                                    {
                                        if (chosenStarters[i]->getFoodName() == starterList[currentChoice]->getFoodName()) // listede seçilen yemeği bul
                                        {
                                            // yemeği seçili durumdan kaldır
                                            user.getMenu().setTotalCost(-starterList[currentChoice]->getFoodPrice());
                                            starterList[currentChoice]->setStarterHeat(StarterHeat::NONE);
                                            chosenStarters.erase(chosenStarters.begin() + i);
                                        }
                                    }
                                }
                                else // yemek daha önceden seçili değilse
                                {
                                    std::string message = "How do you want to your " + starterList[currentChoice]->getFoodName() + "?";
                                    choice = show_selectable_popup(message.c_str(), "Cold", "Hot");
                                    if (choice == 0) // cold
                                    {
                                        starterList[currentChoice]->setStarterHeat(StarterHeat::COLD);
                                    }
                                    if (choice == 1) // hot
                                    {
                                        starterList[currentChoice]->setStarterHeat(StarterHeat::HOT);
                                    }
                                    user.getMenu().setTotalCost(starterList[currentChoice]->getFoodPrice());
                                    chosenStarters.push_back(starterList[currentChoice]);
                                }
                            }
                            break;
                        }
                    }
                    break;
                case 1: // Salads
                    currentChoice = 0;
                    while (!changeScreen)
                    {
                        clear();
                        attron(A_BOLD);
                        attron(COLOR_PAIR(2));
                        attron(A_UNDERLINE);
                        mvwprintw(stdscr, 0, 2, "%s", "SALADS");
                        attroff(A_UNDERLINE);
                        attroff(COLOR_PAIR(2));
                        attroff(A_BOLD);

                        // Menü öğelerini ekrana yazdır
                        for (int i = 0; i < saladList.size(); i++)
                        {
                            if (!chosenSalads.empty())
                            {

                                for (const auto &chosenSalad : chosenSalads)
                                {
                                    if (chosenSalad->getFoodName() == saladList[i]->getFoodName())
                                    {
                                        attron(A_REVERSE);
                                        break;
                                    }
                                }
                                if (currentChoice == i)
                                {
                                    attron(COLOR_PAIR(1));
                                }

                                std::string topping;
                                if (saladList[i]->getIsTopping() == true)
                                {
                                    topping = "(extra topping)";
                                }
                                if (saladList[i]->getIsTopping() == false)
                                {
                                    topping = "";
                                }

                                std::string message = std::to_string(saladList[i]->getFoodPrice()) + "$ " + saladList[i]->getFoodName() + " " + topping;
                                mvprintw(i + 1, 2, "%s", message.c_str()); // Öğeyi yazdır
                                attroff(A_REVERSE);
                                attroff(COLOR_PAIR(1));
                            }
                            else
                            {
                                if (currentChoice == i)
                                {
                                    attron(COLOR_PAIR(1)); // Seçili öğeyi vurgula
                                }
                                std::string message = std::to_string(saladList[i]->getFoodPrice()) + "$ " + saladList[i]->getFoodName();
                                mvprintw(i + 1, 2, "%s", message.c_str()); // Öğeyi yazdır
                                attroff(COLOR_PAIR(1));
                            }
                        }
                        std::string message = std::to_string(user.getMenu().getTotalCost()) + "$ ";
                        mvprintw(saladList.size() + 2, 2, "%s", message.c_str());

                        std::vector<std::shared_ptr<menu::MenuItem>> menuItemList(saladList.begin(), saladList.end());
                        printMenuOperatorButtons(currentChoice, menuItemList);

                        refresh();

                        key = getch(); // Kullanıcının bastığı tuşu al

                        switch (key)
                        {
                        case 27:
                            currentScreen = -1;
                            changeScreen = true;
                            break;

                        case KEY_UP:
                            if (currentChoice > 0)
                            {
                                currentChoice--; // Yukarı git
                            }
                            break;

                        case KEY_DOWN:
                            if (currentChoice < saladList.size() + 2)
                            {
                                currentChoice++; // Aşağı git
                            }
                            break;

                        case KEY_RIGHT:
                            if (menuPartIndex < 5)
                            {
                                menuPartIndex++;
                                changeScreen = true;
                            }
                            break;

                        case KEY_LEFT:
                            if (menuPartIndex > 0)
                            {
                                menuPartIndex--;
                                changeScreen = true;
                            }
                            break;

                        case 10: // Enter tuşu

                            /*
                            if (currentChoice == saladList.size() + 3) // Clear Menu
                            {
                                chosenStarters.clear();
                                chosenSalads.clear();
                                chosenMainCourses.clear();
                                chosenAppetizers.clear();
                                chosenDrinks.clear();
                                chosenDesserts.clear();
                                changeScreen = true;
                                menuPartIndex = 0;
                            }
                            if (currentChoice == saladList.size() + 4) // Exit Menu
                            {
                                choice = show_selectable_popup("Are you sure that you want to back to the main page. Your choicies will not be deleted.", "Yes", "No");
                                refresh();
                                if (choice == 0) // YES
                                {
                                    currentScreen = -1;
                                    changeScreen = true;
                                }
                            }
                            if (currentChoice == saladList.size() + 5) // Commit Menu
                            {
                                if (!chosenStarters.empty() ||
                                    !chosenSalads.empty() ||
                                    !chosenMainCourses.empty() ||
                                    !chosenAppetizers.empty() ||
                                    !chosenDrinksempty() ||
                                    !chosenDesserts.empty())
                                {
                                    showPopup("Your orders will be served in the short time. Enjoy your meal!");
                                    refresh();
                                    return 0;
                                }
                                else
                                {
                                    showPopup("There aren't any chosen foods.");
                                }
                            }
                            else
                            {
                                auto result = std::find(chosenSalads.begin(), chosenSalads.end(), saladList[currentChoice]);
                                if (result != chosenSalads.end()) // yemek zaten daha önceden seçilmişse
                                {
                                    for (int i = 0; i < chosenSalads.size(); i++)
                                    {
                                        if (chosenSalads[i]->getFoodName() == saladList[currentChoice]->getFoodName()) // listede seçilen yemeği bul
                                        {
                                            // yemeği seçili durumdan kaldır
                                            if (chosenSalads[i]->getIsTopping() == true)
                                            {
                                                user.getMenu().setTotalCost(-2.25);
                                                saladList[currentChoice]->setIsTopping(false);
                                            }
                                            user.getMenu().setTotalCost(-saladList[currentChoice]->getFoodPrice());
                                            chosenSalads.erase(chosenSalads.begin() + i);
                                        }
                                    }
                                }
                                else // yemek daha önceden seçili değilse
                                {
                                    std::string message = "Do you want to extra topping onto your " + saladList[currentChoice]->getFoodName() + "?";
                                    int choice = show_selectable_popup(message.c_str(), "Yes", "No");
                                    if (choice == 0) // yes
                                    {
                                        user.getMenu().setTotalCost(saladList[currentChoice]->getFoodPrice() + 2.25);
                                        saladList[currentChoice]->setIsTopping(true);
                                    }
                                    if (choice == 1) // no
                                    {
                                        user.getMenu().setTotalCost(saladList[currentChoice]->getFoodPrice());
                                        saladList[currentChoice]->setIsTopping(false);
                                    }
                                    chosenSalads.push_back(saladList[currentChoice]);
                                }
                            }
                            */

                            std::vector<std::shared_ptr<menu::MenuItem>> menuItemList(saladList.begin(), saladList.end());
                            if (menuOperatorConditions(
                                    currentChoice,
                                    menuPartIndex,
                                    currentScreen,
                                    changeScreen,
                                    choice,
                                    menuItemList,
                                    chosenStarters,
                                    chosenSalads,
                                    chosenMainCourses,
                                    chosenAppetizers,
                                    chosenDrinks,
                                    chosenDesserts,
                                    user) != true)
                            {
                                auto result = std::find(chosenSalads.begin(), chosenSalads.end(), saladList[currentChoice]);
                                if (result != chosenSalads.end()) // yemek zaten daha önceden seçilmişse
                                {
                                    for (int i = 0; i < chosenSalads.size(); i++)
                                    {
                                        if (chosenSalads[i]->getFoodName() == saladList[currentChoice]->getFoodName()) // listede seçilen yemeği bul
                                        {
                                            // yemeği seçili durumdan kaldır
                                            if (chosenSalads[i]->getIsTopping() == true)
                                            {
                                                user.getMenu().setTotalCost(-2.25);
                                                saladList[currentChoice]->setIsTopping(false);
                                            }
                                            user.getMenu().setTotalCost(-saladList[currentChoice]->getFoodPrice());
                                            chosenSalads.erase(chosenSalads.begin() + i);
                                        }
                                    }
                                }
                                else // yemek daha önceden seçili değilse
                                {
                                    std::string message = "Do you want to extra topping onto your " + saladList[currentChoice]->getFoodName() + "?";
                                    choice = show_selectable_popup(message.c_str(), "Yes", "No");
                                    if (choice == 0) // yes
                                    {
                                        user.getMenu().setTotalCost(saladList[currentChoice]->getFoodPrice() + 2.25);
                                        saladList[currentChoice]->setIsTopping(true);
                                    }
                                    if (choice == 1) // no
                                    {
                                        user.getMenu().setTotalCost(saladList[currentChoice]->getFoodPrice());
                                        saladList[currentChoice]->setIsTopping(false);
                                    }
                                    chosenSalads.push_back(saladList[currentChoice]);
                                }
                            }
                            break;
                        }
                    }
                    break;
                case 2: // Main Courses
                    currentChoice = 0;
                    vegetarianSelected = false;
                    while (!changeScreen)
                    {
                        clear();
                        refresh();
                        attron(A_BOLD);
                        attron(COLOR_PAIR(2));
                        attron(A_UNDERLINE);
                        mvwprintw(stdscr, 0, 2, "%s", "MAIN COURSES");
                        attroff(A_UNDERLINE);
                        attroff(COLOR_PAIR(2));
                        attroff(A_BOLD);

                        wattron(veg_win, A_BOLD);
                        wattron(nonveg_win, A_BOLD);
                        if (vegetarianSelected == false)
                        {
                            wattron(nonveg_win, WA_REVERSE);
                        }
                        mvwprintw(nonveg_win, 1, 2, "%s", "Non-Vegetarian");
                        wattroff(nonveg_win, WA_REVERSE);
                        if (vegetarianSelected == true)
                        {
                            wattron(veg_win, WA_REVERSE);
                        }
                        mvwprintw(veg_win, 1, 2, "%s", "Vegetarian");
                        wattroff(veg_win, WA_REVERSE);
                        wattroff(veg_win, A_BOLD);
                        wattroff(nonveg_win, A_BOLD);

                        wrefresh(veg_win);
                        wrefresh(nonveg_win);

                        // Menü öğelerini ekrana yazdır
                        if (vegetarianSelected == true)
                        {
                            for (int i = 0; i < vegetarianMainCourseList.size(); i++)
                            {
                                if (!chosenMainCourses.empty())
                                {
                                    for (const auto &chosenMainCourse : chosenMainCourses)
                                    {
                                        if (chosenMainCourse->getFoodName() == vegetarianMainCourseList[i]->getFoodName())
                                        {
                                            attron(A_REVERSE);
                                            break;
                                        }
                                    }
                                    if (currentChoice == i)
                                    {
                                        attron(COLOR_PAIR(1));
                                    }

                                    std::string message = std::to_string(vegetarianMainCourseList[i]->getFoodPrice()) + "$ " + vegetarianMainCourseList[i]->getFoodName();
                                    mvprintw(i + 3, 2, "%s", message.c_str()); // Öğeyi yazdır
                                    attroff(A_REVERSE);
                                    attroff(COLOR_PAIR(1));
                                }
                                else
                                {
                                    if (currentChoice == i)
                                    {
                                        attron(COLOR_PAIR(1)); // Seçili öğeyi vurgula
                                    }
                                    std::string message = std::to_string(vegetarianMainCourseList[i]->getFoodPrice()) + "$ " + vegetarianMainCourseList[i]->getFoodName();
                                    mvprintw(i + 3, 2, "%s", message.c_str()); // Öğeyi yazdır
                                    attroff(COLOR_PAIR(1));
                                }
                            }

                            std::vector<std::shared_ptr<menu::MenuItem>> menuItemList(vegetarianMainCourseList.begin(), vegetarianMainCourseList.end());
                            printMenuOperatorButtons(currentChoice, menuItemList);

                            std::string message = std::to_string(user.getMenu().getTotalCost()) + "$ ";
                            mvprintw(vegetarianMainCourseList.size() + 8, 2, "%s", message.c_str());
                        }
                        else
                        {
                            for (int i = 0; i < non_vegetarianMainCourseList.size(); i++)
                            {
                                if (!chosenMainCourses.empty())
                                {
                                    for (const auto &chosenMainCourse : chosenMainCourses)
                                    {
                                        if (chosenMainCourse->getFoodName() == non_vegetarianMainCourseList[i]->getFoodName())
                                        {
                                            attron(A_REVERSE);
                                            break;
                                        }
                                    }
                                    if (currentChoice == i)
                                    {
                                        attron(COLOR_PAIR(1));
                                    }

                                    std::string message = std::to_string(non_vegetarianMainCourseList[i]->getFoodPrice()) + "$ " + non_vegetarianMainCourseList[i]->getFoodName();
                                    mvprintw(i + 3, 2, "%s", message.c_str()); // Öğeyi yazdır
                                    attroff(A_REVERSE);
                                    attroff(COLOR_PAIR(1));
                                }
                                else
                                {
                                    if (currentChoice == i)
                                    {
                                        attron(COLOR_PAIR(1)); // Seçili öğeyi vurgula
                                    }
                                    std::string message = std::to_string(non_vegetarianMainCourseList[i]->getFoodPrice()) + "$ " + non_vegetarianMainCourseList[i]->getFoodName();
                                    mvprintw(i + 3, 2, "%s", message.c_str()); // Öğeyi yazdır
                                    attroff(COLOR_PAIR(1));
                                }
                            }

                            std::vector<std::shared_ptr<menu::MenuItem>> menuItemList(non_vegetarianMainCourseList.begin(), non_vegetarianMainCourseList.end());
                            printMenuOperatorButtons(currentChoice, menuItemList);

                            std::string message = std::to_string(user.getMenu().getTotalCost()) + "$ ";
                            mvprintw(non_vegetarianMainCourseList.size() + 8, 2, "%s", message.c_str());
                        }

                        refresh();
                        key = getch();

                        switch (key)
                        {
                        case 27:
                            currentScreen = -1;
                            changeScreen = true;
                            break;

                        case KEY_UP:
                            if (currentChoice > 0)
                            {
                                currentChoice--; // Yukarı git
                            }
                            break;

                        case KEY_DOWN:
                            if (vegetarianSelected == true)
                            {
                                if (currentChoice < vegetarianMainCourseList.size() + 2)
                                {
                                    currentChoice++; // Aşağı git
                                }
                            }
                            if (vegetarianSelected == false)
                            {
                                if (currentChoice < non_vegetarianMainCourseList.size() + 2)
                                {
                                    currentChoice++; // Aşağı git
                                }
                            }
                            break;

                        case KEY_RIGHT:
                            if (menuPartIndex < 5)
                            {
                                menuPartIndex++;
                                changeScreen = true;
                            }
                            break;

                        case KEY_LEFT:
                            if (menuPartIndex > 0)
                            {
                                menuPartIndex--;
                                changeScreen = true;
                            }
                            break;

                        case 49: // 1
                            if (vegetarianSelected == false)
                            {
                                vegetarianSelected = true;
                                currentChoice = 0;
                            }
                            break;

                        case 50: // 2
                            if (vegetarianSelected == true)
                            {
                                vegetarianSelected = false;
                                currentChoice = 0;
                            }
                            break;

                        case 10: // Enter tuşu
                            std::vector<std::shared_ptr<menu::MenuItem>> menuItemList =
                                vegetarianSelected ? std::vector<std::shared_ptr<menu::MenuItem>>(vegetarianMainCourseList.begin(), vegetarianMainCourseList.end())
                                                   : std::vector<std::shared_ptr<menu::MenuItem>>(non_vegetarianMainCourseList.begin(), non_vegetarianMainCourseList.end());

                            if (menuOperatorConditions(
                                    currentChoice,
                                    menuPartIndex,
                                    currentScreen,
                                    changeScreen,
                                    choice,
                                    menuItemList,
                                    chosenStarters,
                                    chosenSalads,
                                    chosenMainCourses,
                                    chosenAppetizers,
                                    chosenDrinks,
                                    chosenDesserts,
                                    user) != true)
                            {
                                if (vegetarianSelected == true)
                                {
                                    auto result = std::find(chosenMainCourses.begin(), chosenMainCourses.end(), vegetarianMainCourseList[currentChoice]);
                                    if (result != chosenMainCourses.end())
                                    {
                                        for (int i = 0; i < chosenMainCourses.size(); i++)
                                        {
                                            if (chosenMainCourses[i]->getFoodName() == vegetarianMainCourseList[currentChoice]->getFoodName())
                                            {
                                                user.getMenu().setTotalCost(-vegetarianMainCourseList[currentChoice]->getFoodPrice());
                                                chosenMainCourses.erase(chosenMainCourses.begin() + i);
                                            }
                                        }
                                    }
                                    else
                                    {
                                        user.getMenu().setTotalCost(vegetarianMainCourseList[currentChoice]->getFoodPrice());
                                        chosenMainCourses.push_back(vegetarianMainCourseList[currentChoice]);
                                    }
                                }
                                if (vegetarianSelected == false)
                                {
                                    auto result = std::find(chosenMainCourses.begin(), chosenMainCourses.end(), non_vegetarianMainCourseList[currentChoice]);
                                    if (result != chosenMainCourses.end())
                                    {
                                        for (int i = 0; i < chosenMainCourses.size(); i++)
                                        {
                                            if (chosenMainCourses[i]->getFoodName() == non_vegetarianMainCourseList[currentChoice]->getFoodName())
                                            {
                                                user.getMenu().setTotalCost(-non_vegetarianMainCourseList[currentChoice]->getFoodPrice());
                                                chosenMainCourses.erase(chosenMainCourses.begin() + i);
                                            }
                                        }
                                    }
                                    else
                                    {
                                        user.getMenu().setTotalCost(non_vegetarianMainCourseList[currentChoice]->getFoodPrice());
                                        chosenMainCourses.push_back(non_vegetarianMainCourseList[currentChoice]);
                                    }
                                }
                            }

                            break;
                        }
                    }
                    break;
                case 3: // Appetizers
                    currentChoice = 0;
                    while (!changeScreen)
                    {
                        clear();
                        attron(A_BOLD);
                        attron(COLOR_PAIR(2));
                        attron(A_UNDERLINE);
                        mvwprintw(stdscr, 0, 2, "%s", "APPETIZERS");
                        attroff(A_UNDERLINE);
                        attroff(COLOR_PAIR(2));
                        attroff(A_BOLD);

                        // Menü öğelerini ekrana yazdır
                        for (int i = 0; i < appetizerList.size(); i++)
                        {
                            if (!chosenAppetizers.empty())
                            {
                                for (const auto &chosenAppetizer : chosenAppetizers)
                                {
                                    if (chosenAppetizer->getFoodName() == appetizerList[i]->getFoodName())
                                    {
                                        attron(A_REVERSE);
                                        break;
                                    }
                                }
                                if (currentChoice == i)
                                {
                                    attron(COLOR_PAIR(1));
                                }
                                std::string appetizerTime;
                                if (appetizerList[i]->getAppetizerTime() == AppetizerTime::BEFORE)
                                {
                                    appetizerTime = "(Before)";
                                }
                                if (appetizerList[i]->getAppetizerTime() == AppetizerTime::AFTER)
                                {
                                    appetizerTime = "(After)";
                                }
                                if (appetizerList[i]->getAppetizerTime() == AppetizerTime::NONE)
                                {
                                    appetizerTime = "";
                                }

                                std::string message = std::to_string(appetizerList[i]->getFoodPrice()) + "$ " + appetizerList[i]->getFoodName() + " " + appetizerTime;
                                mvprintw(i + 1, 2, "%s", message.c_str()); // Öğeyi yazdır
                                attroff(A_REVERSE);
                                attroff(COLOR_PAIR(1));
                            }
                            else
                            {
                                if (currentChoice == i)
                                {
                                    attron(COLOR_PAIR(1)); // Seçili öğeyi vurgula
                                }
                                std::string message = std::to_string(appetizerList[i]->getFoodPrice()) + "$ " + appetizerList[i]->getFoodName();
                                mvprintw(i + 1, 2, "%s", message.c_str()); // Öğeyi yazdır
                                attroff(COLOR_PAIR(1));
                            }
                        }
                        std::string message = std::to_string(user.getMenu().getTotalCost()) + "$ ";
                        mvprintw(appetizerList.size() + 2, 2, "%s", message.c_str());

                        std::vector<std::shared_ptr<menu::MenuItem>> menuItemList(appetizerList.begin(), appetizerList.end());
                        printMenuOperatorButtons(currentChoice, menuItemList);

                        refresh();
                        key = getch(); // Kullanıcının bastığı tuşu al

                        switch (key)
                        {
                        case 27:
                            currentScreen = -1;
                            changeScreen = true;
                            break;

                        case KEY_UP:
                            if (currentChoice > 0)
                            {
                                currentChoice--; // Yukarı git
                            }
                            break;

                        case KEY_DOWN:
                            if (currentChoice < appetizerList.size() + 2)
                            {
                                currentChoice++; // Aşağı git
                            }
                            break;

                        case KEY_RIGHT:
                            if (menuPartIndex < 5)
                            {
                                menuPartIndex++;
                                changeScreen = true;
                            }
                            break;

                        case KEY_LEFT:
                            if (menuPartIndex > 0)
                            {
                                menuPartIndex--;
                                changeScreen = true;
                            }
                            break;

                        case 10: // Enter tuşu

                            /*
                            auto result = std::find(chosenAppetizers.begin(), chosenAppetizers.end(), appetizerList[currentChoice]);
                            if (result != chosenAppetizers.end())
                            {
                                for (int i = 0; i < chosenAppetizers.size(); i++)
                                {
                                    if (chosenAppetizers[i]->getFoodName() == appetizerList[currentChoice]->getFoodName())
                                    {
                                        appetizerList[currentChoice]->setAppetizerTime(AppetizerTime::NONE);
                                        user.getMenu().setTotalCost(-appetizerList[currentChoice]->getFoodPrice());
                                        chosenAppetizers.erase(chosenAppetizers.begin() + i);
                                    }
                                }
                            }
                            else
                            {
                                std::string message = "When do you want to your " + appetizerList[currentChoice]->getFoodName() + "?";
                                int choice = show_selectable_popup(message.c_str(), "Before the Main Course", "After the Main Course");
                                if (choice == 0) // Before
                                {
                                    appetizerList[currentChoice]->setAppetizerTime(AppetizerTime::BEFORE);
                                }
                                if (choice == 1) // After
                                {
                                    appetizerList[currentChoice]->setAppetizerTime(AppetizerTime::AFTER);
                                }
                                user.getMenu().setTotalCost(appetizerList[currentChoice]->getFoodPrice());
                                chosenAppetizers.push_back(appetizerList[currentChoice]);
                            }
                            */

                            std::vector<std::shared_ptr<menu::MenuItem>> menuItemList(appetizerList.begin(), appetizerList.end());
                            if (menuOperatorConditions(
                                    currentChoice,
                                    menuPartIndex,
                                    currentScreen,
                                    changeScreen,
                                    choice,
                                    menuItemList,
                                    chosenStarters,
                                    chosenSalads,
                                    chosenMainCourses,
                                    chosenAppetizers,
                                    chosenDrinks,
                                    chosenDesserts,
                                    user) != true)
                            {
                                auto result = std::find(chosenAppetizers.begin(), chosenAppetizers.end(), appetizerList[currentChoice]);
                                if (result != chosenAppetizers.end())
                                {
                                    for (int i = 0; i < chosenAppetizers.size(); i++)
                                    {
                                        if (chosenAppetizers[i]->getFoodName() == appetizerList[currentChoice]->getFoodName())
                                        {
                                            appetizerList[currentChoice]->setAppetizerTime(AppetizerTime::NONE);
                                            user.getMenu().setTotalCost(-appetizerList[currentChoice]->getFoodPrice());
                                            chosenAppetizers.erase(chosenAppetizers.begin() + i);
                                        }
                                    }
                                }
                                else
                                {
                                    std::string message = "When do you want to your " + appetizerList[currentChoice]->getFoodName() + "?";
                                    choice = show_selectable_popup(message.c_str(), "Before the Main Course", "After the Main Course");
                                    if (choice == 0) // Before
                                    {
                                        appetizerList[currentChoice]->setAppetizerTime(AppetizerTime::BEFORE);
                                    }
                                    if (choice == 1) // After
                                    {
                                        appetizerList[currentChoice]->setAppetizerTime(AppetizerTime::AFTER);
                                    }
                                    user.getMenu().setTotalCost(appetizerList[currentChoice]->getFoodPrice());
                                    chosenAppetizers.push_back(appetizerList[currentChoice]);
                                }
                            }

                            break;
                        }
                    }
                    break;
                case 4: // Drinks
                    currentChoice = 0;
                    while (!changeScreen)
                    {
                        clear();
                        attron(A_BOLD);
                        attron(COLOR_PAIR(2));
                        attron(A_UNDERLINE);
                        mvwprintw(stdscr, 0, 2, "%s", "DRINKS");
                        attroff(A_UNDERLINE);
                        attroff(COLOR_PAIR(2));
                        attroff(A_BOLD);

                        // Menü öğelerini ekrana yazdır
                        for (int i = 0; i < drinkList.size(); i++)
                        {
                            if (!chosenDrinks.empty())
                            {
                                for (const auto &chosenDrink : chosenDrinks)
                                {
                                    if (chosenDrink->getFoodName() == drinkList[i]->getFoodName())
                                    {
                                        attron(A_REVERSE);
                                        break;
                                    }
                                }
                                if (currentChoice == i)
                                {
                                    attron(COLOR_PAIR(1));
                                }
                                std::string carbonated;
                                std::string additionalAlcohol;
                                if (drinkList[i]->getIsCarbonated() == true)
                                {
                                    carbonated = "(carbonated)";
                                }
                                if (drinkList[i]->getIsCarbonated() == false)
                                {
                                    carbonated = "";
                                }
                                if (drinkList[i]->getIsAdditionalAlcohol() == true)
                                {
                                    additionalAlcohol = "(extra alcohol)";
                                }
                                if (drinkList[i]->getIsAdditionalAlcohol() == false)
                                {
                                    additionalAlcohol = "";
                                }

                                std::string message = std::to_string(drinkList[i]->getFoodPrice()) + "$ " + drinkList[i]->getFoodName() + " " + carbonated + " " + additionalAlcohol;
                                mvprintw(i + 1, 2, "%s", message.c_str()); // Öğeyi yazdır
                                attroff(A_REVERSE);
                                attroff(COLOR_PAIR(1));
                            }
                            else
                            {
                                if (currentChoice == i)
                                {
                                    attron(COLOR_PAIR(1)); // Seçili öğeyi vurgula
                                }
                                std::string message = std::to_string(drinkList[i]->getFoodPrice()) + "$ " + drinkList[i]->getFoodName();
                                mvprintw(i + 1, 2, "%s", message.c_str()); // Öğeyi yazdır
                                attroff(COLOR_PAIR(1));
                            }
                        }
                        std::string message = std::to_string(user.getMenu().getTotalCost()) + "$ ";
                        mvprintw(drinkList.size() + 2, 2, "%s", message.c_str());

                        std::vector<std::shared_ptr<menu::MenuItem>> menuItemList(drinkList.begin(), drinkList.end());
                        printMenuOperatorButtons(currentChoice, menuItemList);

                        refresh();
                        key = getch(); // Kullanıcının bastığı tuşu al

                        switch (key)
                        {
                        case 27:
                            currentScreen = -1;
                            changeScreen = true;
                            break;

                        case KEY_UP:
                            if (currentChoice > 0)
                            {
                                currentChoice--; // Yukarı git
                            }
                            break;

                        case KEY_DOWN:
                            if (currentChoice < drinkList.size() + 2)
                            {
                                currentChoice++; // Aşağı git
                            }
                            break;

                        case KEY_RIGHT:
                            if (menuPartIndex < 5)
                            {
                                menuPartIndex++;
                                changeScreen = true;
                            }
                            break;
                        case KEY_LEFT:
                            if (menuPartIndex > 0)
                            {
                                menuPartIndex--;
                                changeScreen = true;
                            }
                            break;

                        case 10: // Enter tuşu

                            /*
                            auto result = std::find(chosenDrinks.begin(), chosenDrinks.end(), drinkList[currentChoice]);
                            if (result != chosenDrinks.end())
                            {
                                for (int i = 0; i < chosenDrinks.size(); i++)
                                {
                                    if (chosenDrinks[i]->getFoodName() == drinkList[currentChoice]->getFoodName())
                                    {
                                        if (chosenDrinks[i]->getIsAdditionalAlcohol() == true && drinkList[currentChoice]->getIsCarbonated() == true)
                                        {
                                            user.getMenu().setTotalCost(-drinkList[currentChoice]->getFoodPrice() - 0.5 - 2.5);
                                        }
                                        if (chosenDrinks[i]->getIsAdditionalAlcohol() == true && drinkList[currentChoice]->getIsCarbonated() == false)
                                        {
                                            user.getMenu().setTotalCost(-drinkList[currentChoice]->getFoodPrice() - 2.5);
                                        }
                                        if (chosenDrinks[i]->getIsAdditionalAlcohol() == false && drinkList[currentChoice]->getIsCarbonated() == true)
                                        {
                                            user.getMenu().setTotalCost(-drinkList[currentChoice]->getFoodPrice() - 0.5);
                                        }
                                        if (chosenDrinks[i]->getIsAdditionalAlcohol() == false && drinkList[currentChoice]->getIsCarbonated() == false)
                                        {
                                            user.getMenu().setTotalCost(-drinkList[currentChoice]->getFoodPrice());
                                        }

                                        drinkList[currentChoice]->setIsCarbonated(false);
                                        drinkList[currentChoice]->setIsAdditionalAlcohol(false);
                                        chosenDrinks.erase(chosenDrinks.begin() + i);
                                    }
                                }
                            }
                            else
                            {
                                std::string carbonatedMessage = "Do you want carbonated to your " + drinkList[currentChoice]->getFoodName() + "?";
                                std::string alcoholMessage = "Do you want to extra alcohol to your " + drinkList[currentChoice]->getFoodName() + "?";
                                int carbonatedChoice = show_selectable_popup(carbonatedMessage.c_str(), "Yes", "No");
                                int alcoholChoice = show_selectable_popup(alcoholMessage.c_str(), "Yes", "No");
                                if (carbonatedChoice == 0) // Yes
                                {
                                    drinkList[currentChoice]->setIsCarbonated(true);
                                    user.getMenu().setTotalCost(0.5);
                                }
                                if (carbonatedChoice == 1) // No
                                {
                                    drinkList[currentChoice]->setIsCarbonated(false);
                                }
                                if (alcoholChoice == 0) // Yes
                                {
                                    drinkList[currentChoice]->setIsAdditionalAlcohol(true);
                                    user.getMenu().setTotalCost(2.5);
                                }
                                if (alcoholChoice == 1) // No
                                {
                                    drinkList[currentChoice]->setIsAdditionalAlcohol(false);
                                }
                                user.getMenu().setTotalCost(drinkList[currentChoice]->getFoodPrice());
                                chosenDrinks.push_back(drinkList[currentChoice]);
                            }
                            */

                            std::vector<std::shared_ptr<menu::MenuItem>> menuItemList(drinkList.begin(), drinkList.end());
                            if (menuOperatorConditions(
                                    currentChoice,
                                    menuPartIndex,
                                    currentScreen,
                                    changeScreen,
                                    choice,
                                    menuItemList,
                                    chosenStarters,
                                    chosenSalads,
                                    chosenMainCourses,
                                    chosenAppetizers,
                                    chosenDrinks,
                                    chosenDesserts,
                                    user) != true)
                            {
                                auto result = std::find(chosenDrinks.begin(), chosenDrinks.end(), drinkList[currentChoice]);
                                if (result != chosenDrinks.end())
                                {
                                    for (int i = 0; i < chosenDrinks.size(); i++)
                                    {
                                        if (chosenDrinks[i]->getFoodName() == drinkList[currentChoice]->getFoodName())
                                        {
                                            if (chosenDrinks[i]->getIsAdditionalAlcohol() == true && drinkList[currentChoice]->getIsCarbonated() == true)
                                            {
                                                user.getMenu().setTotalCost(-drinkList[currentChoice]->getFoodPrice() - 0.5 - 2.5);
                                            }
                                            if (chosenDrinks[i]->getIsAdditionalAlcohol() == true && drinkList[currentChoice]->getIsCarbonated() == false)
                                            {
                                                user.getMenu().setTotalCost(-drinkList[currentChoice]->getFoodPrice() - 2.5);
                                            }
                                            if (chosenDrinks[i]->getIsAdditionalAlcohol() == false && drinkList[currentChoice]->getIsCarbonated() == true)
                                            {
                                                user.getMenu().setTotalCost(-drinkList[currentChoice]->getFoodPrice() - 0.5);
                                            }
                                            if (chosenDrinks[i]->getIsAdditionalAlcohol() == false && drinkList[currentChoice]->getIsCarbonated() == false)
                                            {
                                                user.getMenu().setTotalCost(-drinkList[currentChoice]->getFoodPrice());
                                            }

                                            drinkList[currentChoice]->setIsCarbonated(false);
                                            drinkList[currentChoice]->setIsAdditionalAlcohol(false);
                                            chosenDrinks.erase(chosenDrinks.begin() + i);
                                        }
                                    }
                                }
                                else
                                {
                                    std::string carbonatedMessage = "Do you want carbonated to your " + drinkList[currentChoice]->getFoodName() + "?";
                                    std::string alcoholMessage = "Do you want to extra alcohol to your " + drinkList[currentChoice]->getFoodName() + "?";
                                    int carbonatedChoice = show_selectable_popup(carbonatedMessage.c_str(), "Yes", "No");
                                    int alcoholChoice = show_selectable_popup(alcoholMessage.c_str(), "Yes", "No");
                                    if (carbonatedChoice == 0) // Yes
                                    {
                                        drinkList[currentChoice]->setIsCarbonated(true);
                                        user.getMenu().setTotalCost(0.5);
                                    }
                                    if (carbonatedChoice == 1) // No
                                    {
                                        drinkList[currentChoice]->setIsCarbonated(false);
                                    }
                                    if (alcoholChoice == 0) // Yes
                                    {
                                        drinkList[currentChoice]->setIsAdditionalAlcohol(true);
                                        user.getMenu().setTotalCost(2.5);
                                    }
                                    if (alcoholChoice == 1) // No
                                    {
                                        drinkList[currentChoice]->setIsAdditionalAlcohol(false);
                                    }
                                    user.getMenu().setTotalCost(drinkList[currentChoice]->getFoodPrice());
                                    chosenDrinks.push_back(drinkList[currentChoice]);
                                }
                            }

                            break;
                        }
                    }
                    break;
                case 5: // Desserts
                    currentChoice = 0;
                    while (!changeScreen)
                    {
                        clear();
                        attron(A_BOLD);
                        attron(COLOR_PAIR(2));
                        attron(A_UNDERLINE);
                        mvwprintw(stdscr, 0, 2, "%s", "DESSERTS");
                        attroff(A_UNDERLINE);
                        attroff(COLOR_PAIR(2));
                        attroff(A_BOLD);

                        // Menü öğelerini ekrana yazdır
                        for (int i = 0; i < dessertList.size(); i++)
                        {
                            if (!chosenDesserts.empty())
                            {
                                for (const auto &chosenDessert : chosenDesserts)
                                {
                                    if (chosenDessert->getFoodName() == dessertList[i]->getFoodName())
                                    {
                                        attron(A_REVERSE);
                                        break;
                                    }
                                }
                                if (currentChoice == i)
                                {
                                    attron(COLOR_PAIR(1));
                                }
                                std::string extraChocolate;
                                if (dessertList[i]->getIsExtraChocolate() == true)
                                {
                                    extraChocolate = "(extra chocolate)";
                                }
                                if (dessertList[i]->getIsExtraChocolate() == false)
                                {
                                    extraChocolate = "";
                                }

                                std::string message = std::to_string(dessertList[i]->getFoodPrice()) + "$ " + dessertList[i]->getFoodName() + " " + extraChocolate;
                                mvprintw(i + 1, 2, "%s", message.c_str()); // Öğeyi yazdır
                                attroff(A_REVERSE);
                                attroff(COLOR_PAIR(1));
                            }
                            else
                            {
                                if (currentChoice == i)
                                {
                                    attron(COLOR_PAIR(1)); // Seçili öğeyi vurgula
                                }
                                std::string message = std::to_string(dessertList[i]->getFoodPrice()) + "$ " + dessertList[i]->getFoodName();
                                mvprintw(i + 1, 2, "%s", message.c_str()); // Öğeyi yazdır
                                attroff(COLOR_PAIR(1));
                            }
                        }
                        std::string message = std::to_string(user.getMenu().getTotalCost()) + "$ ";
                        mvprintw(dessertList.size() + 2, 2, "%s", message.c_str());

                        std::vector<std::shared_ptr<menu::MenuItem>> menuItemList(dessertList.begin(), dessertList.end());
                        printMenuOperatorButtons(currentChoice, menuItemList);

                        refresh();
                        key = getch(); // Kullanıcının bastığı tuşu al

                        switch (key)
                        {
                        case 27:
                            currentScreen = -1;
                            changeScreen = true;
                            break;

                        case KEY_UP:
                            if (currentChoice > 0)
                            {
                                currentChoice--; // Yukarı git
                            }
                            break;

                        case KEY_DOWN:
                            if (currentChoice < dessertList.size() + 2)
                            {
                                currentChoice++; // Aşağı git
                            }
                            break;

                        case KEY_RIGHT:
                            if (menuPartIndex < 5)
                            {
                                menuPartIndex++;
                                changeScreen = true;
                            }
                            break;

                        case KEY_LEFT:
                            if (menuPartIndex > 0)
                            {
                                menuPartIndex--;
                                changeScreen = true;
                            }
                            break;

                        case 10: // Enter tuşu
                            std::vector<std::shared_ptr<menu::MenuItem>> menuItemList(dessertList.begin(), dessertList.end());
                            if (menuOperatorConditions(
                                    currentChoice,
                                    menuPartIndex,
                                    currentScreen,
                                    changeScreen,
                                    choice,
                                    menuItemList,
                                    chosenStarters,
                                    chosenSalads,
                                    chosenMainCourses,
                                    chosenAppetizers,
                                    chosenDrinks,
                                    chosenDesserts,
                                    user) != true)
                            {
                                auto result = std::find(chosenDesserts.begin(), chosenDesserts.end(), dessertList[currentChoice]);
                                if (result != chosenDesserts.end()) // tatlı daha önceden seçilmişse
                                {
                                    for (int i = 0; i < chosenDesserts.size(); i++)
                                    {
                                        if (chosenDesserts[i]->getFoodName() == dessertList[currentChoice]->getFoodName())
                                        {
                                            if (dessertList[currentChoice]->getIsExtraChocolate() == true)
                                            {
                                                user.getMenu().setTotalCost(-1.5);
                                            }
                                            dessertList[currentChoice]->setIsExtraChocolate(false);
                                            user.getMenu().setTotalCost(-dessertList[currentChoice]->getFoodPrice());
                                            chosenDesserts.erase(chosenDesserts.begin() + i);
                                        }
                                    }
                                }
                                else
                                {
                                    std::string message = "Do you want to extra chocolate onto your " + dessertList[currentChoice]->getFoodName() + "?";
                                    choice = show_selectable_popup(message.c_str(), "Yes", "No");
                                    if (choice == 0) // yes
                                    {
                                        user.getMenu().setTotalCost(dessertList[currentChoice]->getFoodPrice() + 1.5);
                                        dessertList[currentChoice]->setIsExtraChocolate(true);
                                    }
                                    if (choice == 1) // no
                                    {
                                        user.getMenu().setTotalCost(dessertList[currentChoice]->getFoodPrice());
                                        dessertList[currentChoice]->setIsExtraChocolate(false);
                                    }
                                    chosenDesserts.push_back(dessertList[currentChoice]);
                                }
                            }

                            break;
                        }
                    }
                    break;
                }
            }
            while (currentScreen == 1)
            {
                currentChoice = 0;

                clear();
                curs_set(1); // İmleci göster
                echo();      // Kullancının girdiği değerleri göster

                int _sweet;
                int _sour;
                int _bitter;
                int _salty;
                int _savory;

                std::string starterHeat;
                std::string topping;
                std::string appetizerTime;
                std::string carbonated;
                std::string additionalAlcohol;
                std::string extraChocolate;

                mvprintw(0, 2, "%s", "Give me a taste number from 0 to 10 for each tastes (sweet, sour, bitter, salty, savory). Then, I will suggest the best menu for you. If you don't want to suggested menu, you can change your taste numbers.");
                mvprintw(2, 2, "%s", "Please enter sweet number. ");
                refresh();
                scanw("%d", &_sweet);
                mvprintw(3, 2, "%s", "Please enter sour number. ");
                refresh();
                scanw("%d", &_sour);
                mvprintw(4, 2, "%s", "Please enter bitter number. ");
                refresh();
                scanw("%d", &_bitter);
                mvprintw(5, 2, "%s", "Please enter salty number. ");
                refresh();
                scanw("%d", &_salty);
                mvprintw(6, 2, "%s", "Please enter savory number. ");
                refresh();
                scanw("%d", &_savory);
                clear();
                curs_set(0); // İmleci göster
                noecho();

                clear();
                refresh();
                int starterHeatChoice = show_selectable_popup("How do you want to your starter?", "Cold", "Hot");
                clear();
                refresh();
                int extraToppingChoice = show_selectable_popup("Do you want to extra topping onto your salad?", "Yes", "No");
                clear();
                refresh();
                int vegetarianChoice = show_selectable_popup("How do you want to your main course?", "Vegetarian", "Non-vegetarian");
                clear();
                refresh();
                int appetizerTimeChoice = show_selectable_popup("When do you want to your appetizer?", "Before the Main Course", "After the Main Course");
                clear();
                refresh();
                int carbonatedChoice = show_selectable_popup("Do you want carbonated to your drink?", "Yes", "No");
                clear();
                refresh();
                int alcoholChoice = show_selectable_popup("Do you want to extra alcohol to your drink?", "Yes", "No");
                clear();
                refresh();
                int chocolateChoice = show_selectable_popup("Do you want to extra chocolate onto your dessert?", "Yes", "No");
                clear();
                refresh();

                double minTotalDifference = 100000;
                std::vector<std::shared_ptr<MainCourse>> mainCourseList;
                double totalCost = 0.0;
                Menu balancedMenu;

                for (const auto &starter : starterList)
                {
                    for (const auto &salad : saladList)
                    {
                        if (vegetarianChoice == 0) // vegetarian
                        {
                            mainCourseList = vegetarianMainCourseList;
                        }
                        if (vegetarianChoice == 1) // non-vegetarian
                        {
                            mainCourseList = non_vegetarianMainCourseList;
                        }

                        for (const auto &mainCourse : mainCourseList)
                        {
                            for (const auto &appetizer : appetizerList)
                            {
                                for (const auto &drink : drinkList)
                                {
                                    for (const auto &dessert : dessertList)
                                    {

                                        double foodSweetAvarage = (starter->getFoodTasteBalance().getSweet() +
                                                                   salad->getFoodTasteBalance().getSweet() +
                                                                   mainCourse->getFoodTasteBalance().getSweet() +
                                                                   appetizer->getFoodTasteBalance().getSweet() +
                                                                   drink->getFoodTasteBalance().getSweet() +
                                                                   dessert->getFoodTasteBalance().getSweet()) /
                                                                  6;

                                        double foodSourAvarage = (starter->getFoodTasteBalance().getSour() +
                                                                  salad->getFoodTasteBalance().getSour() +
                                                                  mainCourse->getFoodTasteBalance().getSour() +
                                                                  appetizer->getFoodTasteBalance().getSour() +
                                                                  drink->getFoodTasteBalance().getSour() +
                                                                  dessert->getFoodTasteBalance().getSour()) /
                                                                 6;

                                        double foodBitterAvarage = (starter->getFoodTasteBalance().getBitter() +
                                                                    salad->getFoodTasteBalance().getBitter() +
                                                                    mainCourse->getFoodTasteBalance().getBitter() +
                                                                    appetizer->getFoodTasteBalance().getBitter() +
                                                                    drink->getFoodTasteBalance().getBitter() +
                                                                    dessert->getFoodTasteBalance().getBitter()) /
                                                                   6;

                                        double foodSaltyAvarage = (starter->getFoodTasteBalance().getSalty() +
                                                                   salad->getFoodTasteBalance().getSalty() +
                                                                   mainCourse->getFoodTasteBalance().getSalty() +
                                                                   appetizer->getFoodTasteBalance().getSalty() +
                                                                   drink->getFoodTasteBalance().getSalty() +
                                                                   dessert->getFoodTasteBalance().getSalty()) /
                                                                  6;

                                        double foodSavoryAvarage = (starter->getFoodTasteBalance().getSavory() +
                                                                    salad->getFoodTasteBalance().getSavory() +
                                                                    mainCourse->getFoodTasteBalance().getSavory() +
                                                                    appetizer->getFoodTasteBalance().getSavory() +
                                                                    drink->getFoodTasteBalance().getSavory() +
                                                                    dessert->getFoodTasteBalance().getSavory()) /
                                                                   6;

                                        double totalDifference = abs(_sweet - foodSweetAvarage) +
                                                                 abs(_sour - foodSourAvarage) +
                                                                 abs(_bitter - foodBitterAvarage) +
                                                                 abs(_salty - foodSaltyAvarage) +
                                                                 abs(_savory - foodSavoryAvarage);

                                        if (totalDifference < minTotalDifference)
                                        {
                                            totalCost = 0;
                                            if (starterHeatChoice == 0) // cold
                                            {
                                                starter->setStarterHeat(StarterHeat::COLD);
                                                starterHeat = "(cold)";
                                            }
                                            if (starterHeatChoice == 1) // hot
                                            {
                                                starter->setStarterHeat(StarterHeat::HOT);
                                                starterHeat = "(hot)";
                                            }
                                            if (extraToppingChoice == 0)
                                            {
                                                salad->setIsTopping(true);
                                                topping = "(extra topping)";
                                                totalCost = totalCost + 2.25;
                                            }
                                            if (extraToppingChoice == 1)
                                            {
                                                salad->setIsTopping(false);
                                                topping = "";
                                            }
                                            if (appetizerTimeChoice == 0) // before
                                            {
                                                appetizer->setAppetizerTime(AppetizerTime::BEFORE);
                                                appetizerTime = "(before)";
                                            }
                                            if (appetizerTimeChoice == 1) // after
                                            {
                                                appetizer->setAppetizerTime(AppetizerTime::AFTER);
                                                appetizerTime = "(after)";
                                            }
                                            if (carbonatedChoice == 0)
                                            {
                                                drink->setIsCarbonated(true);
                                                carbonated = "(carbonated)";
                                                totalCost = totalCost + 0.5;
                                            }
                                            if (carbonatedChoice == 1)
                                            {
                                                drink->setIsCarbonated(false);
                                                carbonated = "";
                                            }
                                            if (alcoholChoice == 0)
                                            {
                                                drink->setIsAdditionalAlcohol(true);
                                                additionalAlcohol = "(extra alcohol)";
                                                totalCost = totalCost + 2.5;
                                            }
                                            if (alcoholChoice == 1)
                                            {
                                                drink->setIsAdditionalAlcohol(false);
                                                additionalAlcohol = "";
                                            }
                                            if (chocolateChoice == 0)
                                            {
                                                dessert->setIsExtraChocolate(true);
                                                extraChocolate = "(extra chocolate)";
                                                totalCost = totalCost + 1.5;
                                            }
                                            if (chocolateChoice == 1)
                                            {
                                                dessert->setIsExtraChocolate(false);
                                                extraChocolate = "";
                                            }

                                            minTotalDifference = totalDifference;
                                            std::vector<std::shared_ptr<MenuItem>> selectedMenuItems;
                                            selectedMenuItems.push_back(starter);
                                            selectedMenuItems.push_back(salad);
                                            selectedMenuItems.push_back(mainCourse);
                                            selectedMenuItems.push_back(drink);
                                            selectedMenuItems.push_back(appetizer);
                                            selectedMenuItems.push_back(dessert);
                                            balancedMenu.addMenu(selectedMenuItems);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }

                attron(COLOR_PAIR(1));
                attron(A_BOLD);
                mvprintw(0, 2, "Suggested Menu");
                attroff(COLOR_PAIR(1));
                attroff(A_BOLD);

                for (int i = 0; i < balancedMenu.getMenuItems().size(); i++)
                {
                    totalCost = totalCost + balancedMenu.getMenuItems()[i]->getFoodPrice();

                    for (const auto &starter : starterList)
                    {
                        if (balancedMenu.getMenuItems()[i]->getFoodName() == starter->getFoodName())
                        {
                            std::string message = std::to_string(starter->getFoodPrice()) + "$ " + starter->getFoodName() + " " + starterHeat;
                            mvprintw(i + 1, 2, "%s", message.c_str()); // Öğeyi yazdır
                        }
                    }
                    for (const auto &salad : saladList)
                    {
                        if (balancedMenu.getMenuItems()[i]->getFoodName() == salad->getFoodName())
                        {
                            std::string message = std::to_string(salad->getFoodPrice()) + "$ " + salad->getFoodName() + " " + topping;
                            mvprintw(i + 1, 2, "%s", message.c_str()); // Öğeyi yazdır
                        }
                    }
                    for (const auto &mainCourse : mainCourseList)
                    {
                        if (balancedMenu.getMenuItems()[i]->getFoodName() == mainCourse->getFoodName())
                        {
                            std::string message = std::to_string(mainCourse->getFoodPrice()) + "$ " + mainCourse->getFoodName();
                            mvprintw(i + 1, 2, "%s", message.c_str()); // Öğeyi yazdır
                        }
                    }
                    for (const auto &appetizer : appetizerList)
                    {
                        if (balancedMenu.getMenuItems()[i]->getFoodName() == appetizer->getFoodName())
                        {
                            std::string message = std::to_string(appetizer->getFoodPrice()) + "$ " + appetizer->getFoodName() + " " + appetizerTime;
                            mvprintw(i + 1, 2, "%s", message.c_str()); // Öğeyi yazdır
                        }
                    }
                    for (const auto &drink : drinkList)
                    {
                        if (balancedMenu.getMenuItems()[i]->getFoodName() == drink->getFoodName())
                        {
                            std::string message = std::to_string(drink->getFoodPrice()) + "$ " + drink->getFoodName() + " " + carbonated + " " + additionalAlcohol;
                            mvprintw(i + 1, 2, "%s", message.c_str()); // Öğeyi yazdır
                        }
                    }
                    for (const auto &dessert : dessertList)
                    {
                        if (balancedMenu.getMenuItems()[i]->getFoodName() == dessert->getFoodName())
                        {
                            std::string message = std::to_string(dessert->getFoodPrice()) + "$ " + dessert->getFoodName() + " " + extraChocolate;
                            mvprintw(i + 1, 2, "%s", message.c_str()); // Öğeyi yazdır
                        }
                    }
                }

                std::string message = "Total:" + std::to_string(totalCost) + "$ ";
                mvprintw(balancedMenu.getMenuItems().size() + 2, 2, "%s", message.c_str());

                refresh();

                currentChoice = 0;
                changeScreen = false;

                menuDidYouLikeOperators(
                    changeScreen,
                    currentChoice,
                    key,
                    currentScreen,
                    totalCost,
                    balancedMenu.getMenuItems(),
                    user,
                    extraToppingChoice,
                    carbonatedChoice,
                    alcoholChoice,
                    chocolateChoice);

                /*
                attron(COLOR_PAIR(2));
                attron(A_BOLD);
                mvprintw(11, 2, "%s", "Did you like the menu?");
                attroff(COLOR_PAIR(2));
                attroff(A_BOLD);
                while (!changeScreen)
                {
                    if (currentChoice == 0)
                    {
                        attron(A_REVERSE);
                    }
                    mvprintw(12, 2, "%s", "Yes, I want to order the menu.");
                    attroff(A_REVERSE);
                    if (currentChoice == 1)
                    {
                        attron(A_REVERSE);
                    }
                    mvprintw(13, 2, "%s", "No, I want to try again.");
                    attroff(A_REVERSE);
                    if (currentChoice == 2)
                    {
                        attron(A_REVERSE);
                    }
                    mvprintw(14, 2, "%s", "Back to Main Page");
                    attroff(A_REVERSE);

                    key = getch();

                    switch (key)
                    {
                    case KEY_UP:
                        if (currentChoice > 0)
                            currentChoice--; // Yukarı git
                        break;
                    case KEY_DOWN:
                        if (currentChoice < 2)
                            currentChoice++; // Aşağı git
                        break;
                    case 10:
                        if (currentChoice == 0)
                        {
                            if (extraToppingChoice == 0)
                            {
                                user.getMenu().setTotalCost(2.25);
                            }
                            if (carbonatedChoice == 0)
                            {
                                user.getMenu().setTotalCost(0.5);
                            }
                            if (alcoholChoice == 0)
                            {
                                user.getMenu().setTotalCost(2.5);
                            }
                            if (chocolateChoice == 0)
                            {
                                user.getMenu().setTotalCost(1.5);
                            }
                            user.getMenu().setTotalCost(totalCost);
                            user.getMenu().addMenu(balancedMenu.getMenuItems());
                            showPopup("Your orders will be served in the short time. Enjoy your meal!");
                            endwin();
                            return 0;
                        }
                        if (currentChoice == 1)
                        {
                            changeScreen = true;
                        }
                        if (currentChoice == 2)
                        {
                            currentScreen = -1;
                            changeScreen = true;
                        }
                        break;
                    case 27:
                        currentScreen = -1;
                        changeScreen = true;
                        break;
                    }
                }
                */
            }
            while (currentScreen == 2)
            {
                currentChoice = 0;
                changeScreen = false;
                clear();
                int starterRandom = getRandomNumber(0, starterList.size() - 1);
                int saladRandom = getRandomNumber(0, saladList.size() - 1);
                int mainCourseRandom = getRandomNumber(0, allMainCoursesList.size() - 1);
                int appetizerRandom = getRandomNumber(0, appetizerList.size() - 1);
                int drinkRandom = getRandomNumber(0, drinkList.size() - 1);
                int dessertRandom = getRandomNumber(0, dessertList.size() - 1);

                std::vector<std::shared_ptr<MenuItem>> menuList = {
                    starterList[starterRandom],
                    saladList[saladRandom],
                    allMainCoursesList[mainCourseRandom],
                    appetizerList[appetizerRandom],
                    drinkList[drinkRandom],
                    dessertList[dessertRandom]};

                double totalCost = 0;

                for (int i = 0; i < menuList.size(); i++)
                {
                    totalCost = totalCost + menuList[i]->getFoodPrice();
                    std::string message = std::to_string(menuList[i]->getFoodPrice()) + "$ " + menuList[i]->getFoodName();
                    mvprintw(i + 1, 2, "%s", message.c_str()); // Öğeyi yazdır
                }
                std::string message = "Total: " + std::to_string(totalCost) + "$ ";
                mvprintw(menuList.size() + 2, 2, "%s", message.c_str()); // Öğeyi yazdır

                refresh();

                menuDidYouLikeOperators(changeScreen, currentChoice, key, currentScreen, totalCost, menuList, user, std::nullopt, std::nullopt, std::nullopt, std::nullopt);
            }
            while (currentScreen == 3)
            {
                choice = show_selectable_popup("Are you sure that you want to leave the restaurant?", "Yes", "No");
                refresh();
                if (choice == 0) // yes
                {
                    endwin();
                    return 0;
                }
                break;
            }

            //currentChoice = 0;
            currentScreen = 0;
            changeScreen = false;
            break;

        case 27:
            choice = show_selectable_popup("Are you sure that you want to leave the restaurant?", "Yes", "No");
            refresh();
            if (choice == 0) // yes
            {
                endwin();
                return 0;
            }
            break;
        }
    }
    getch();

    endwin();

    return 0;
}
