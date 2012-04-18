/*
 * interpolator2d.cpp
 *
 *  Created on: Mar 26, 2012
 *      Author: timo
 */

#include "interpolator2d.h"

#include <iostream>
#include <sstream>
#include <fstream>
#include <limits>

Interpolator2d::Interpolator2d()
{
    // nothing to do here
}

void Interpolator2d::load(const char * filename)
{
    minX = std::numeric_limits<double>::max( );
    maxX = std::numeric_limits<double>::min( );
    minY = std::numeric_limits<double>::max( );
    maxY = std::numeric_limits<double>::min( );

    table.clear();

    std::ifstream file(filename);
    if(!file) {
        std::cerr << "Interpolator2d: table file not found: " << filename
                  << std::endl;
        return;
    }

    char buffer[2048];
    std::stringstream stream;
    tableEntry entry;
    while (file.good()) {
        file.getline(buffer, 2048);
        if (file.eof()) break;
        if (buffer[0] == '#') continue;
        stream << buffer;
        double dummy;
        stream >> entry.x >> entry.y >> dummy;
        stream.clear();
        if (entry.x < minX) minX = entry.x;
        if (entry.x > maxX) maxX = entry.x;
        if (entry.y < minY) minY = entry.y;
        if (entry.y > maxY) maxY = entry.y;
        table.push_back(entry);
    }

    N = table.size();
}

double Interpolator2d::x(const double& y)
{
    // guess position
    int i = (y-minY)*N/(maxY-minY);
    tableEntry* A = &(table[i]);
    tableEntry* B = A;
    if (A->y == y) return A->x;
    // find surrounding frequency values. In the end it should hold
    // A->f < f < B->f
    while (B->y < y) {
        A = B;
        B = &(table[++i]);
    }
    while (A->y > y) {
        B = A;
        A = &(table[--i]);
    }
    const double dA = y - A->y;
    const double dB = B->y - y;
    const double xapprox = (dB*A->x+dA*B->x)/(dA+dB);
    return xapprox;
}

double Interpolator2d::y(const double& x)
{
    // guess position
    if (x<minX or x > maxX) return 0;
    int i = (x-minX)*N/(maxX-minX);
    tableEntry* A = &(table[i]);
    tableEntry* B = A;
    if (A->x == x) return A->y;
    // find surrounding x values. In the end it should hold
    // A->x < x < B->x
    while (B->x < x) {
        A = B;
        B = &(table[++i]);
    }
    while (A->x > x) {
        B = A;
        A = &(table[--i]);
    }
    const double dA = x - A->x;
    const double dB = B->x - x;
    const double yapprox = (dB*A->y+dA*B->y)/(dA+dB);
    return yapprox;
}
