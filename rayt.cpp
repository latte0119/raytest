#include "rayt.hpp"
#include<memory>
#include<iostream>


int main(){
    rayt::Scene s(200,100,100);
    s.build();
    s.render();
}