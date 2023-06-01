#include <exception>
#include <iostream>

int main() {
  try {
  } catch (std::exception& e) {
    std::cerr << e.what();
  } catch (...) {
    std::cerr << "Unknown exception";
  }
}