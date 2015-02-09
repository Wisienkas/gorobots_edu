#include "ctextarraywriter.h"

#include <iostream>
using std::ostream;

#include <fstream>
using std::ofstream;


const unsigned int runbot::cTextArrayWriter::version = 0;


runbot::cTextArrayWriter::cTextArrayWriter ( std::string _file_name, int _num_cols,
                                     std::string _column_names)
        : cArrayWriter(_file_name, _num_cols, _column_names),
          file(_file_name.c_str(), std::ios::out) {

    file << "# Text dump version: " << version << std::endl;
    file << "# number of cols: " << _num_cols << std::endl;
    file << "# " << _column_names << std::endl;

    buf = new double[_num_cols];
}


runbot::cTextArrayWriter::~cTextArrayWriter () {
    file.close();
    delete buf;
}


void runbot::cTextArrayWriter::write ( std::valarray< double > a ) {
    for(unsigned int i=0; i<a.size(); ++i) {
        file << a[i];
        if (i != a.size())
            file << " ";
    }
    file << "\n";
}
