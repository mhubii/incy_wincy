#include <iostream>
#include <thread> 
#include <chrono>

// Speed benchmarking :).
enum {
    START,
    STOP
};

auto Timer(int set) -> double {

    double msec;

    // Set a timer.
    switch (set)
    {
        case START:
            static auto t1 = std::chrono::high_resolution_clock::now();
            msec = 0.;
            break;
        case STOP:
            static auto t2 = std::chrono::high_resolution_clock::now();
            msec = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
            break;
        default:
            break;
    }
    return msec;
}

int main(int argc, char** argv) {

    Timer(START);
    Timer(START); // redeclaration of t1? -> no

    std::this_thread::sleep_for (std::chrono::seconds(1));

    auto msec = Timer(STOP);

    printf("msec: %f", msec);

    return 0;
};
