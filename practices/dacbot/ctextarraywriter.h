#pragma once
#ifndef CTEXTARRAYWRITER_H
#define CTEXTARRAYWRITER_H

#include "carraywriter.h"

namespace runbot {

class cTextArrayWriter : public cArrayWriter {
public:
    cTextArrayWriter(std::string _file_name, int _num_cols, std::string _column_names);
    virtual ~cTextArrayWriter();

    static const unsigned int version;

    virtual void write(std::valarray< double > array);

private:
    std::ofstream file;
    double*       buf;
};

}

#endif // CTEXTARRAYWRITER_H
