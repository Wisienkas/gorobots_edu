
# directory to build object files 
BUILD_DIR      := build
# directories to check for source files
DIRS           := utils controllers
# directories containing unit tests
TEST_DIRS      := tests
# name of the generated library files
STATLIB        := libgorobots.a 
DYNLIB         := libgorobots.so
# name of the generated test executable
TEST_EXEC_NAME := run_test
# cpp files to exclude from the gorobots library
# These files are excluded because they introduce a bunch of global variables
# and defines which is not compatible with creating a library
#EXCLUDES = controllers/nimm4ii/ngnet.cpp \
#           controllers/nimm4ii/acicorcactorcontroller.cpp \
#           controllers/nimm4ii/acicorccriticcontroller.cpp \
#           controllers/amosii/reinforcement_learning_control/*

# Routine to discover source files and generate .o targets
# find_files discovers all cpp files in a directory and all its subdirectories
find_cppfiles     = $(shell find $(dir) -name '*.cpp')

# CPPFILES contains all c++ source files that should be compiled
CPPFILES      := $(filter-out $(EXCLUDES),$(foreach dir,$(DIRS),$(find_cppfiles)))
# OFILES contains the names of the object files to generate from the sources
OFILES        := $(patsubst %.cpp,${BUILD_DIR}/%.o, $(CPPFILES))
# DEP_FILES contains the names of the dependency files that give information
# about which files need to be recompiled of certain files changed.
DEP_FILES     := $(OFILES:.o=.d)

# source, object and dependency files belonging to unit tests
TEST_CPPFILES  := $(foreach dir,$(TEST_DIRS),$(find_cppfiles))
TEST_OFILES    := $(patsubst %.cpp,${BUILD_DIR}/%.o, $(TEST_CPPFILES))
TEST_DEP_FILES  := $(TEST_OFILES:.o=.d)

# the test executable should end up in the build directory:
TEST_EXEC := $(BUILD_DIR)/$(TEST_EXEC_NAME)

# include directories
INC += -I.

# libraries needed to link statically to ode_robots
STATLIBS += $(shell ode_robots-config --static --libs)
# libraries needed to link statically to selforg
STATLIBS += $(shell selforg-config --static --libs)

# libraries needed to link dynamically to ode_robots
DYNLIBS += $(shell ode_robots-config --libs)
# libraries needed to link dynamically to selforg
DYNLIBS += $(shell selforg-config --libs)
DYNLIBS += -ltinyxml2

# libraries to include for testing
# -lgtest      : includes google test library
# -lgtest_main : use the standard main method provided with google test
# $(STATLIB)   : the static gorobots library
# $(STATLIBS)  : dependencies of the static gorobots build
TEST_LIBS += -lgtest -lgtest_main $(STATLIB) $(STATLIBS)

# flags for the c++ compiler
# -Wall             : enable all warnings
# -pipe             : Use pipes rather than temporary files for communication 
#                     between the various stages of compilation
# -fPIC             : Generate position-independent code (PIC) suitable for use
#                     in a shared library
# selforg-config    : configuration script for selforg
#   --cflags        : generate flags for the gcc compiler
# ode_robots_config :
#   --intern        : add debug symbols and light optimization
#   --cflags        : generate flags for the gcc compiler
CXXFLAGS = -Wall \
           -pipe \
           -fPIC \
           $(shell selforg-config --cflags) \
           $(shell ode_robots-config --intern --cflags)

# flags for ar (used to create static lib)
# -r : Insert files into the archive by replacing existing files with same name
# -c : Create the archive if it is not yet existing
# -s : Add an index to the archvive or update if it already exists
AFLAGS := -rcs

# Flags for the linker
# -shared           : Produce a shared object which can then be linked with 
#                     other objects to form an executable.
# -Wl,-rpath,$(shell selforg-config --srcprefix) : add the path in which the
#                     selforg library is produced to the list of paths to check
#                     at runtime for needed libraries
# -Wl,-rpath,$(shell ode_robots-config --prefix)/lib : add the path in which 
#                     the ode_robots library is produced to the list of paths
#                     to check at runtime for needed libraries
LDFLAGS += -shared \
           -Wl,-rpath,$(shell selforg-config --srcprefix) \
           -Wl,-rpath,$(shell ode_robots-config --prefix)/lib

# the normal build target: create the static and dynamic gorobots library
normal: statlib dynlib

# build all: includes static and dynamic builds and the test excecutable
all: normal test

# target defining that "statlib" means build the static library file
statlib: $(STATLIB)

# target defining that "dynlib" means build the dynamic library file
dynlib: $(DYNLIB)

# target defining how to build the static library file
$(STATLIB): $(OFILES)
	# packs all object files into the library file
	$(AR) $(ARFLAGS) $(STATLIB) $(OFILES) 

# target defining how to build the dynamic library file
$(DYNLIB): $(OFILES)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) $(OFILES) $(DYNLIBS) -o $(DYNLIB)

# target defining that test means build the test executable
test: $(TEST_EXEC)

# target defining that run_test means run the text executable
run_test: $(TEST_EXEC)
	./$(TEST_EXEC)

# target defining how to build the test executable
$(TEST_EXEC): $(TEST_OFILES) $(STATLIB)
	$(CXX) $(CPPFLAGS) $(TEST_OFILES) $(TEST_LIBS) -o $(TEST_EXEC)

# target defining generic rule how to build object files from source files
# 1. creates output directory if not existent
#     $@        : gives the name of the object file to be build
#     $(dir $@) : returns only the directoy part of the file name
#     -p        : no error if existing, make parent directories as needed
#     @mkdir    : The @-sign surpresses output of the command itself
# 2. actual compiler call
#     -c        : create object file
#     -MMD      : Output dependency rules (but only for user header files) 
#                 while at the same time compiling as usual
#     -MP       : This option instructs CPP to add a phony target for each 
#                 dependency other than the main file, causing each to depend 
#                 on nothing. These dummy rules work around errors make gives 
#                 if you remove header files without updating the Makefile to 
#                 match.
#     $@        : gives the name of the object file to be build
#     $<        : gives the first dependency (here: the source file)
#
${BUILD_DIR}/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) -MMD -MP -c $(CXXFLAGS) $(INC) -o $@ $<

# clean target does not depend on any files. if called, it should always be
# executed => Make it phony
.PHONY: clean
# target to remove built files
clean:
	rm -rf $(OFILES) $(DEP_FILES) $(STATLIB) $(TEST_EXEC)

# include the automatically created dependencies
-include $(DEP_FILES)
-include $(TEST_DEP_FILES)
