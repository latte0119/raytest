#include "rayt.hpp"

#include <chrono>
#include <iostream>
#include <memory>

int main() {
    std::chrono::system_clock::time_point start, end;
    start = std::chrono::system_clock::now();

    rayt::Scene s(200, 200, 1000);
    s.render();

    end = std::chrono::system_clock::now();
    double time = static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / 1000.0);
    printf("%f\n", time);

    return 0;
}