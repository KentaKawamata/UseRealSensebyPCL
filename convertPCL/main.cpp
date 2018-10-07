//
// Created by kawa on 10/6/18.
//
#include <iostream>
#include <sstream>
#include "convert.hpp"

int main(int argc, char *argv[]){

    try {

        ConvertPCL convertPCL;
        convertPCL.run();

    } catch (std::exception &ex) {
        std::cout << ex.what() << std::endl;
    }
    return 0;
}