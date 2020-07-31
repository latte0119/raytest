#include "rayt.hpp"
#include<memory>
#include<iostream>
#include<chrono>

int main(){
    std::chrono::system_clock::time_point start,end;
    start=std::chrono::system_clock::now();

    rayt::Scene s(200,100,50);
    s.build();
    s.render();

    end=std::chrono::system_clock::now();
    double time=static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count()/1000.0);
    printf("%f\n",time);

    return 0;
}