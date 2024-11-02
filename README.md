# Features
- Example project to create Python bindings for a C++ project using Pybind11 and Cmake.
- Usage of the `build` package and a *pyproject.toml* configuration file to create a Python package.
- Generates Python stubs for the C++ classes and functions.
- Ensures smooth imports through the use of an `__init__.py` file.
- Includes a small test script to test the package.
- Automatically fetches dependencies for the C++ code using Cmake.
- Uses Plog as logging library for C++ code. This is convenient and showcases how to integrate other C++ libraries.

# Instructions
1. Make sure that the *build* package of python is installed: `pip install build`
2. Run the following command from the project root directory to build the package: `python -m build`
3. The package will be created in the *dist* directory.
4. You can install the package using the following command: `pip install dist/<package-name>.tar.gz`
5. For testing, you can run `build_and_test.sh` script from the project root directory. This will build the package and install it in a virtual environment and run a test script.