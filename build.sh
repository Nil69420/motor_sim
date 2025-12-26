#!/bin/bash

set -e  

echo "================================"
echo "Motor Driver Sim - Build Script"
echo "================================"
echo ""

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Parse arguments
BUILD_TYPE="Release"
BUILD_TESTS=ON
BUILD_EXAMPLES=ON
CLEAN=0

while [[ $# -gt 0 ]]; do
    case $1 in
        --debug)
            BUILD_TYPE="Debug"
            shift
            ;;
        --no-tests)
            BUILD_TESTS=OFF
            shift
            ;;
        --no-examples)
            BUILD_EXAMPLES=OFF
            shift
            ;;
        --clean)
            CLEAN=1
            shift
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: $0 [--debug] [--no-tests] [--no-examples] [--clean]"
            exit 1
            ;;
    esac
done

PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
BUILD_DIR="$PROJECT_DIR/build"

echo "Project directory: $PROJECT_DIR"
echo "Build type: $BUILD_TYPE"
echo "Build tests: $BUILD_TESTS"
echo "Build examples: $BUILD_EXAMPLES"
echo ""

if [ $CLEAN -eq 1 ]; then
    echo -e "${YELLOW}Cleaning build directory...${NC}"
    rm -rf "$BUILD_DIR"
    echo -e "${GREEN}✓ Clean complete${NC}"
    echo ""
fi

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

echo -e "${YELLOW}Configuring CMake...${NC}"
cmake .. \
    -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
    -DBUILD_TESTS=$BUILD_TESTS \
    -DBUILD_EXAMPLES=$BUILD_EXAMPLES \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Configuration successful${NC}"
else
    echo -e "${RED}✗ Configuration failed${NC}"
    exit 1
fi
echo ""

echo -e "${YELLOW}Building...${NC}"
NUM_CORES=$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)
echo "Using $NUM_CORES cores"

make -j$NUM_CORES

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Build successful${NC}"
else
    echo -e "${RED}✗ Build failed${NC}"
    exit 1
fi
echo ""

if [ "$BUILD_TESTS" == "ON" ]; then
    echo -e "${YELLOW}Running tests...${NC}"
    ctest --output-on-failure
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓ All tests passed${NC}"
    else
        echo -e "${RED}✗ Some tests failed${NC}"
        exit 1
    fi
    echo ""
fi

echo "================================"
echo -e "${GREEN}Build Complete!${NC}"
echo "================================"
echo ""
echo "Binaries are in: $BUILD_DIR"

if [ "$BUILD_EXAMPLES" == "ON" ]; then
    echo ""
    echo "Run examples:"
    echo "  ./build/basic_control"
    echo "  ./build/position_tracking"
    echo "  ./build/multi_motor"
fi

if [ "$BUILD_TESTS" == "ON" ]; then
    echo ""
    echo "Run tests:"
    echo "  cd build && ctest"
fi

echo ""
