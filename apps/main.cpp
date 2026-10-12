#include "rtvm/version.hpp"
#include <iostream>

int main(){
    std::cout << "rtvm version " << rtvm::version() << std::endl;
}