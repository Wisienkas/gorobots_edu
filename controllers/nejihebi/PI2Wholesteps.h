/***************************************************************************
 *   Copyright (C) 2012 by                                                 *
 *                                                          *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 *                                                                         *
 ***************************************************************************/

#include <map>

#include <stdlib.h>
#include <map>
#include <utility>
#include <math.h>

using namespace std;

/**
 * Example Nejihebi Controller
 *
 * This is a very simple example controller for the Nejihebi Robot. It
 * demonstrates different movement gaits. Without further adjustments it can
 * be used for the simulated as wells as for the real machine.
 */
#ifndef PI2Wholesteps_H
#define PI2Wholesteps_H

class PI2Wholesteps {
  public:
    PI2Wholesteps(bool const& verbose=false):verbose(verbose) {
      l1 = 0.103;  // distance between former joint and the center of screw [m]
      l2 = 0.123;  // distance between latter joint and the center of screw [m]
      L = l1 + l2;                // length of a unit
      l0 = 0;
      r = 0.075;                  // radius of a screw
      alp = M_PI / 4;             // inclination of a brade // pi=3.1416
      alpha[0] = -alp;
      alpha[1] = alp;
      alpha[2] = -alp;
      alpha[3] = alp;
    }

    Eigen::MatrixXd wholesteps(float y[], float t[], float phi[],
        float goal[]); //Main PI2 update rule
    // robot configuration
    float l1;      // distance between former joint and the center of screw [m]
    float l2;      // distance between latter joint and the center of screw [m]
    float L;                  // length of a unit
    float l0;
    float r;                  // radius of a screw
    float alp;             // inclination of a blade // pi=3.1416
    float alpha[4];
    bool const verbose;

};

class costfunction {
  public:
    double getcost(float a[], float b[]) {
      return sqrt(pow((a[0] - b[0]), 2) + pow((a[1] - b[1]), 2));
    }

};

#endif
