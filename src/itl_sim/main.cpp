#include <iostream>

#include "itl/controller.h"

int main(int argc, const char ** argv)
{
  std::cout << "Hello world!" << std::endl;
  std::cout << itl::getGreeting("MPC") << std::endl;
  return EXIT_SUCCESS;
}
