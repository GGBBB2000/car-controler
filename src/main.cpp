#include "../include/carController.hpp"
#include <memory>

int main()
{
    auto controller = new CarController();
    controller->run();
    return 0;
}
