/*
 * interpolator2d.h
 *
 *  Created on: Mar 26, 2012
 *      Author: timo
 */

#ifndef INTERPOLATOR2D_H_
#define INTERPOLATOR2D_H_

#include <vector>

class Interpolator2d
{
public:
    Interpolator2d();
    void load(const char* filename);
    double x(const double& y);
    double y(const double& x);
private:
    struct tableEntry {
        double x;
        double y;
    };
    std::vector<tableEntry> table;
    double minX;
    double maxX;
    double minY;
    double maxY;
    int    N;
};


#endif /* INTERPOLATOR2D_H_ */
