#pragma once
#ifndef CARRAYWRITER_H
#define CARRAYWRITER_H

#include <string>
#include <fstream>
#include <valarray>

namespace runbot {


/** \brief Virtual interface for writing valarrays to files.
  */
class cArrayWriter {
public:
    cArrayWriter(std::string _file_name, int _num_cols, std::string _column_names)
        : file_name(_file_name), num_cols(_num_cols), column_names(_column_names) {};
    virtual ~cArrayWriter() {};

    virtual void write(std::valarray< double > array)=0;

private:
    std::string file_name;
    int num_cols;
    std::string column_names;
};

}
#endif // CARRAYWRITER_H
