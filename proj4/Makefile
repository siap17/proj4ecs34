# Directories
BIN_DIR = bin
OBJ_DIR = obj
SRC_DIR = src
TEST_DIR = testsrc
INCLUDE_DIR = include
TARGET = $(BIN_DIR)/teststrutils

# Google Test flags
GTEST_FLAGS = -I/opt/homebrew/opt/googletest/include -L/opt/homebrew/opt/googletest/lib -lgtest -lgtest_main -pthread

# Source files
SRC = $(SRC_DIR)/StringUtils.cpp $(TEST_DIR)/StringUtilsTest.cpp

# Object files (placing them in OBJ_DIR)
OBJ = $(OBJ_DIR)/StringUtils.o $(OBJ_DIR)/StringUtilsTest.o

all: directories $(TARGET) runtests

# Compile source files into object files
$(OBJ_DIR)/StringUtils.o: $(SRC_DIR)/StringUtils.cpp $(INCLUDE_DIR)/StringUtils.h
	g++ -c -std=c++17 -I $(INCLUDE_DIR) -o $@ $<

$(OBJ_DIR)/StringUtilsTest.o: $(TEST_DIR)/StringUtilsTest.cpp $(INCLUDE_DIR)/StringUtils.h
	g++ -c -std=c++17 -I $(INCLUDE_DIR) -o $@ $< $(GTEST_FLAGS)

# Link object files into the final executable
$(TARGET): $(OBJ)
	g++ $(OBJ) -o $(TARGET) -std=c++17 $(GTEST_FLAGS)

# Run tests
runtests: $(TARGET)
	$(TARGET)

# Create necessary directories
directories:
	mkdir -p $(BIN_DIR)
	mkdir -p $(OBJ_DIR)

# Clean up object files and binaries
clean:
	rm -rf $(BIN_DIR) $(OBJ_DIR)
