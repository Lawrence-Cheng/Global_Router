# Compiler and Flags
CXX      = g++
# CHANGED: -O3 to -O2 as per TA instructions
CXXFLAGS = -std=c++17 -O2 -Wall -Wextra -Isrc

# Directories
SRCDIR   = src
BINDIR   = bin
TARGET   = $(BINDIR)/router

# Source Files
SRCS     = $(wildcard $(SRCDIR)/*.cpp)

# Default Target: Create bin dir and compile
all: $(TARGET)

$(TARGET): $(SRCS)
	@mkdir -p $(BINDIR)
	$(CXX) $(CXXFLAGS) $(SRCS) -o $(TARGET)

# Clean Target
clean:
	rm -f $(TARGET)
	rm -rf $(BINDIR)