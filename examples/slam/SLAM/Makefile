#Global path to gorobots directory
#GOROBOTS = /home/pkesper/git/patrickkesper-gorobots-fork/

#Source directories
#SOURCE = $(GOROBOTS)utils/SLAM/ $(GOROBOTS)utils/SLAM/Models/

GOROBOTS = /home/poma/workspace/pmanoonpong-gorobots-fork/

#Source directories
SOURCE = $(GOROBOTS)utils/slam-library/ $(GOROBOTS)utils/slam-library/Models/


#Data directory
DATA = data/
#Include files
INC = $(addprefix -I, $(SOURCE))
#Libraries
LIBS = -lboost_system -lconfig++

#Get all .cpp and .o files
CPP_FILES = $(wildcard $(addsuffix *.cpp, $(SOURCE)))
OBJ_FILES = $(addsuffix .o, $(basename $(CPP_FILES)))

#Compiler flags
CC_FLAGS = -O3 -Wall 

#Phony targets for cleaning
.PHONY: clean
.PHONY: cleanAll
.PHONY: cleanData

#Prevent .o files from being automatically deleted
.PRECIOUS: '%.o'

#Get all targets by finding all .cpp files in current directory
TARGETS = $(shell for file in `find . -maxdepth 1 -name '*.cpp' -type f -printf "%f\n" | sed 's/\..*//'`; do echo "$$file "; done;)

#Make all targets
all: $(TARGETS)

#Define individual targets for each executable
#To be able to detect changes in the main .cpp file, add .cpp to prerequisites
#To do this, use second expansion
.SECONDEXPANSION:
$(TARGETS) : $(addprefix $$@,.cpp) $(OBJ_FILES)
	g++ $(CC_FLAGS) $(addprefix $@,.cpp) -o $@ $(OBJ_FILES) $(INC) $(LIBS)

#Automatic rule to generate .o files. For this get .cpp name from
#.o file name. This is done via second expansion
$(OBJ_FILES) : $$(addprefix $$(basename $$@),.cpp)
	g++ $(CC_FLAGS) -c -o $@ $< $(LIBS) $(INC)



clean:
	rm -f $(addsuffix *.o, $(SOURCE))
	find -maxdepth 1 -executable -type f -delete
	
cleanData:
	rm -f $(DATA)*

cleanAll: clean cleanData

	