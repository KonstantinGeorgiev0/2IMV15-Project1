INC_WINDOWS = include/windows
INC_LINUX   = include/linux
INC_COMMON  = include/common

# Base flags used by all platforms
BASE_CXXFLAGS = -g -O2 -Wall -Wno-sign-compare -DHAVE_CONFIG_H

ifeq ($(OS),Windows_NT)
	RM = rm -vf
	INCLUDE = $(INC_WINDOWS)
	CXXFLAGS = $(BASE_CXXFLAGS) -I$(INCLUDE) -I$(INC_COMMON)
	CXX_EXTRA_FLAGS = -Llib -lfreeglut -lglu32 -lopengl32 -lpng12
else
	RM = rm -vf
	# On Mac, we need the headers from the linux folder for math/gfx libs
	INCLUDE = $(INC_LINUX)
	
	# Combine all paths: linux headers, common headers, and Homebrew headers
	CXXFLAGS = $(BASE_CXXFLAGS) -I$(INCLUDE) -I$(INC_COMMON) -I/opt/homebrew/include -DGL_SILENCE_DEPRECATION
	
	# Link against macOS Frameworks and Homebrew's libpng
	CXX_EXTRA_FLAGS = -framework GLUT -framework OpenGL -L/opt/homebrew/lib -lpng
endif

CXX = g++
SRC_DIR = src
OBJ_DIR = obj
BIN_DIR = bin

SOURCES = $(wildcard $(SRC_DIR)/*cpp)
OBJECTS = $(patsubst %.cpp, $(OBJ_DIR)/%.o, $(notdir $(SOURCES)))
EXECUTABLE = $(BIN_DIR)/project1.exe

all: $(OBJ_DIR) $(BIN_DIR) $(EXECUTABLE)

$(OBJ_DIR):
	mkdir -v $@

$(BIN_DIR):
	mkdir -v $@

obj/%.o: $(SRC_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) -o $@ -c $<

$(EXECUTABLE): $(OBJECTS)
	$(CXX) -o $@ $^ $(CXX_EXTRA_FLAGS)

clean:
	@$(RM) $(OBJ_DIR)/*
	@$(RM) $(BIN_DIR)/*exe