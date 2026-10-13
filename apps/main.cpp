#include "rtvm/version.hpp"
#include <iostream>
#include <opencv2/core.hpp>

int main(){
    std::cout << "rtvm version " << rtvm::version() << std::endl;
    std::cout << "OpenCV " << CV_VERSION << '\n';
}