cmake_minimum_required(VERSION 3.16)
project(Boids VERSION 0.2.10)

# abilita il supporto per i test, tra cui l'opzione BUILD_TESTING usata sotto
include(CTest)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# richiedi l'uso di C++17, senza estensioni non-standard offerte dal compilatore usato 
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

# abilita warning
string(APPEND CMAKE_CXX_FLAGS
" -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wcast-qual -Wformat=2"
" -Wundef -Wshadow -Wcast-align -Wunused -Wnull-dereference"
" -Wdouble-promotion -Wimplicit-fallthrough -Wextra-semi -Woverloaded-virtual"
" -Wnon-virtual-dtor -Wold-style-cast")

# abilita l'address sanitizer e l'undefined-behaviour sanitizer in debug mode
string(APPEND CMAKE_CXX_FLAGS_DEBUG " -fsanitize=address,undefined -fno-omit-frame-pointer")
string(APPEND CMAKE_EXE_LINKER_FLAGS_DEBUG " -fsanitize=address,undefined -fno-omit-frame-pointer")

# find_package(X11 REQUIRED)
find_package(SFML 2.5 COMPONENTS graphics REQUIRED)

# link_directories(${X11_LIBRARIES})

add_executable(Boids_engine main.cpp boid.cpp flock.cpp)
target_link_libraries(Boids_engine PRIVATE sfml-graphics)

# se il testing e' abilitato...
#   per disabilitare il testing, passare -DBUILD_TESTING=OFF a cmake durante la fase di configurazione
if (BUILD_TESTING)

  # aggiungi l'eseguibile statistics.t
  # add_executable(statistics.t statistics.test.cpp statistics.cpp)
  # aggiungi l'eseguibile statistics.t alla lista dei test
  add_test(NAME statistics.t COMMAND statistics.t)

endif()

