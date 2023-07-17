cmake_minimum_required(VERSION 3.16)
project(Boids VERSION 1.1.0)

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

find_package(X11 REQUIRED)
find_package(SFML 2.5 COMPONENTS graphics REQUIRED)
find_package(OpenGL 1.1 REQUIRED)
#find_package(TBB REQUIRED)


# link_directories(${X11_LIBRARIES})

add_executable(Boids_engine main.cpp simulation/boid.cpp simulation/flock.cpp graphics/bird.cpp simulation/predator.cpp graphics/animation.cpp simulation/obstacles.cpp)
target_link_libraries(Boids_engine PRIVATE sfml-graphics)
target_link_libraries(Boids_engine PRIVATE ${OPENGL_LIBRARIES} ${X11_LIBRARIES})
#target_link_libraries(Boids_engine PRIVATE TBB::tbb)

# se il testing e' abilitato...
#   per disabilitare il testing, passare -DBUILD_TESTING=OFF a cmake durante la fase di configurazione
if (BUILD_TESTING)

  # aggiungi l'eseguibile Boids.t
  add_executable(Boids.t tests/all_tests.cpp tests/boids_tests.cpp tests/flock_tests.cpp tests/predator_tests.cpp tests/obstacles_tests.cpp tests/math_tests.cpp simulation/boid.cpp simulation/flock.cpp simulation/predator.cpp simulation/obstacles.cpp )
  target_link_libraries(Boids.t PRIVATE sfml-graphics)
  #target_link_libraries(Boids.t PRIVATE TBB::tbb)
  #aggiungi l'eseguibile Boids.t alla lista dei test
  add_test(NAME Boids.t COMMAND Boids.t)

endif()

