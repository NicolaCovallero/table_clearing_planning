#include "utilities.h"

std::stack<clock_t> tictoc_stack;
void tic() {
    tictoc_stack.push(clock());
}
void toc() {
    std::cout << "Time elapsed: "
              << ((double)(clock() - tictoc_stack.top())) / CLOCKS_PER_SEC
              << std::endl;
    tictoc_stack.pop();
}