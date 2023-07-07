# Physics Programming Project

The implemented program simulates the behavior of a flock of birds in a two-dimensional space.

## Required Libraries for Program Execution

To ensure successful program execution, the following libraries need to be installed:

- SFML: Graphics library for handling graphics and events. You can install it using the command:

```bash
$ sudo apt install libsfml-dev
```
- TBB: Library for supporting parallelism. If not already installed, you can install it using the command:

```bash
$ sudo apt-get install libtbb-dev
```
## Instructions for Compilation, Testing, and Execution

1. Make sure you are in the main project directory.

2. To compile the program 

in debug mode, execute the following command:

```bash
$ cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
```
in release mode, use the command:

```bash
$ cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
```
You can enable or disable testing during compilation by adding the following to the previous command:
- `-DBUILD_TESTING=ON` to enable testing
- `-DBUILD_TESTING=OFF` to disable testing

3. Once the compilation phase is complete, run the command:

```bash
$ cmake --build build
```

If CMake fails to find the elements belonging to the TBB library during the compilation phase, you need to remove the comments
- `find_package(TBB REQUIRED)`
- `target_link_libraries(Boids_engine PRIVATE TBB::tbb)`
- `target_link_libraries(Boids.t PRIVATE TBB::tbb)`

in the CMakeLists.txt file.

4. To execute the program, use the following command:

```bash
$ build/Boids_engine
```
5. To run the tests, testing must have been enabled during compilation. Use the command:
```bash
$ build/Boids.t
```

## Simulation

### Simulation Parameters

At the start of the simulation, the user is prompted to enter various parameters via standard input. These parameters determine the generation of entities and the behavior of the flock and predators. For some parameters, accepted minimum and maximum values are provided, as well as indicative values for a more realistic simulation. If invalid values are entered, a `std::runtime_error` exception is thrown, and the program terminates with an error message displayed on the console.

### Graphics Representation

Two different graphic representation options are available during the simulation:
- `Classic`: Birds and predators are represented by triangles, and obstacles are represented by circles.
- `Star Boids`: Birds and predators are represented as space shuttles, and obstacles are represented as artificial satellites. This representation is inspired by the "Star Wars" science fiction saga.

### User Interaction

During the program execution, the user can interact with the simulation using the following key combinations:
- <kbd>Ctrl</kbd> + <kbd>B</kbd>: Randomly generates a bird on the screen. Holding down the key generates multiple birds.
- <kbd>Ctrl</kbd> + <kbd>P</kbd>: Randomly generates a predator on the screen.
- <kbd>Ctrl</kbd> + <kbd>O</kbd>: Pauses the simulation and allows the user to place one or more obstacles on the screen by clicking the left mouse button. Pressing the combination again resumes the simulation.
- <kbd>Ctrl</kbd> + <kbd>A</kbd>: Pauses the simulation. Pressing the combination again resumes the simulation.

A dynamic text box allows the user to view the activated commands and notifications regarding the success of the allowed operations.

### Statistics

During the simulation, statistics extracted from the flock are printed to an external text file in a properly formatted manner every second.

## Credits

A.A. 2022-2023

Alberto Zaghini, Andrea Maria Varisano, Matteo Mirabelli

July 2023
