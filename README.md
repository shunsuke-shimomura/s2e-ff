# S2E-FF
An example of S2E user side repository for Formation Flying study.

## Structure
- The structure of this repository is shown as follows.
- The `s2e-ff/s2e-core` codes are managed by using `git submodule`
- The `s2e-ff/ExtLibraries` is automatically generated by using the cmake files in the `s2e-ff/s2e-core/ExtLibraries` directory.
- The main directory `s2e-ff/s2e-ff` is handled as the normal user side directory.
  <pre>
  s2e-ff (repository origin)
  ├─ExtLibraries
  ├─s2e-core (submodule)
  └─s2e-ff
  </pre>

## How to start
- Clone
- Initialize and update submodule
- Generate `ExtLibraries` directory
- Open `s2e-ff/s2e-ff`
- Build and execute

## Support Environment
- GCC 9.4.0
- Visual Studio 2022

## GoogleTest
- We use [GoogleTest](https://github.com/google/googletest) to test the codes.
- How to install the GoogleTest
  - Move to `s2e-ff/ExtLibraries` directory
  - Clone GoogleTest `release-1.12.1` with the following command
    ```
    git clone https://github.com/google/googletest.git -b release-1.12.1
    ```
  - TODO: Automatically clone the repository in CMake
- How to execute the test
  - Test codes are stored in the `s2e-ff/s2e-ff/test` directory.
  - Users need to add the test codes and test target codes in the build target list `TEST_FILES` in the `CMakeList.txt`.
  - Turn on the `BUILD_64BIT` and `GOOGLE_TEST` option in the `CMakeList.txt`
    - GoogleTest supports only 64bit build
  - Build the `S2E_FF` with test
  - Execute `S2E_FF_TEST`
