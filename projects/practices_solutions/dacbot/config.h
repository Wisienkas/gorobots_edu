#pragma once

#include<string>
using std::string;

/// Unclutter main dir using data directory:
const string DATA_PATH("data/");

/// Define which array writer class to use.
// #define WRITER_CLASS cBinaryArrayWriter
// #include "src/cbinaryarraywriter.h"
#define WRITER_CLASS cTextArrayWriter
#include "ctextarraywriter.h"

#define COMEDI_POST_0_9_0
