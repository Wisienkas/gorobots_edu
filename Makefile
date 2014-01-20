
# directory to build object files 
BUILD_DIR      := build
# directories to check for source files
DIRS           := utils controllers
# directories containing unit tests
TEST_DIRS      := tests
# name of the generated library file
LIB            := libgorobots.a 
DYNLIB         := libgorobots.so
# name of the generated test executable
TEST_EXEC_NAME := run_test
# cpp files to exclude from the gorobots library
# These files are excluded because they introduce a bunch of global variables
# and defines which is not compatible with creating a library
EXCLUDES = controllers/nimm4ii/ngnet.cpp \
           controllers/nimm4ii/acicorcactorcontroller.cpp \
           controllers/nimm4ii/acicorccriticcontroller.cpp

# Routine to discover source files and generate .o targets
find_files     = $(shell find $(dir) -name '*.cpp')
CPPFILES      := $(foreach dir,$(DIRS),$(find_files))
CPPFILES      := $(filter-out $(EXCLUDES), $(CPPFILES))
OFILES        := $(patsubst %.cpp,${BUILD_DIR}/%.o, $(CPPFILES))
DEP_FILES     := $(OFILES:.o=.d)

find_testfiles  = $(shell find $(dir) -name '*.cpp')
TEST_CPPFILES  := $(foreach dir,$(TEST_DIRS),$(find_testfiles))
TEST_OFILES    := $(patsubst %.cpp,${BUILD_DIR}/%.o, $(TEST_CPPFILES))
TEST_DEP_FILES  := $(TEST_OFILES:.o=.d)

# the test executable should end up in the build directory:
TEST_EXEC := $(BUILD_DIR)/$(TEST_EXEC_NAME)

# include directories
INC += -I.

# libraries needed by ode_robots
LIBS += $(shell ode_robots-config --static --libs)
# libraries needed by selforg
LIBS += $(shell selforg-config --static --libs)

DYNLIBS += $(shell ode_robots-config --libs)
DYNLIBS += $(shell selforg-config --libs)
DYNLIBS += -ltinyxml2

# libraries to include for testing
# -lgtest : includes google test library
TEST_LIBS += -lgtest -lgtest_main -lpthread $(LIB) 

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
LDFLAGS += -shared \
           -Wl,-rpath,$(shell selforg-config --srcprefix)

all: lib dynlib

# target defining that "lib" means build the library file
lib: $(LIB)

dynlib: $(DYNLIB)

# target defining how to build the library file
$(LIB): $(OFILES)
	# packs all object files into the library file
	$(AR) $(ARFLAGS) $(LIB) $(OFILES) 

$(DYNLIB): $(OFILES)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) $(OFILES) $(DYNLIBS) -o $(DYNLIB)

# target defining that test means build the test executable
test: $(TEST_EXEC)

# target defining that run_test means run the text executable
run_test: $(TEST_EXEC)
	./$(TEST_EXEC)

# target defining how to build the test executable
$(TEST_EXEC): $(TEST_OFILES) $(LIB)
	$(CXX) $(CPPFLAGS) $(TEST_OFILES) $(LIBS) $(TEST_LIBS) -o $(TEST_EXEC)

# target defining generic rule how to build object files from source files
# 1. creates output directory if not existent
# 2. actual compiler call
#     -c   : create object file
#     -MMD : Output dependency rules (but only for user header files) while 
#            at the same time compiling as usual
#     -MP  : This option instructs CPP to add a phony target for each 
#            dependency other than the main file, causing each to depend on 
#            nothing. These dummy rules work around errors make gives if you 
#            remove header files without updating the Makefile to match.
${BUILD_DIR}/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) -MMD -MP -c $(CXXFLAGS) $(INC) -o "$@" "$<"

# clean target does not depend on any files. if called, it should always be
# executed => Make it phony
.PHONY: clean
# target to remove built files
clean:
	rm -rf $(OFILES) $(DEP_FILES) $(LIB) $(TEST_EXEC)

# include the automatically created dependencies
-include $(DEP_FILES)
-include $(TEST_DEP_FILES)
