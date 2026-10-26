CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2
TARGET = send_cpu_temp_cpp
SRC = send_cpu_temp_cpp.cpp

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(SRC) -o $(TARGET)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET)

.PHONY: all run clean
