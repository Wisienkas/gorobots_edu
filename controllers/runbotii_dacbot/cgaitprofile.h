#pragma once
#ifndef CGAITPROFILE_H
#define CGAITPROFILE_H

#include <iostream>

namespace runbot {

/** cGaitProfile holds all parameters to specify a gait.
  *
  * It gives you the possibility to define everything asymmetric or
  * use a symmetric constructor for easier usage. The underlying class
  * is the same.
  *
  * As it is used as a base for the cGaitTransition, it supports arithmetic
  * operations on itself. This can be used to calculate differences or sums
  * of gaitprofiles.
  */
class cGaitProfile {
    public:
        /// Create a new symmetric gait with dummy parameters.
        cGaitProfile() :
                     m_maxhip_left(100),    m_maxhip_right(100),// set to 100
                     m_minhip_left(78),     m_minhip_right(78),
                     m_maxknee_left(175),   m_maxknee_right(175), //175
                     m_minknee_left(115),   m_minknee_right(115), // 115
                     m_gain_hip_left(2.2),  m_gain_hip_right(2.2),// set to 2.2
                     m_gain_knee_left(1.8), m_gain_knee_right(1.8) {};

        /// Create a new symmetric gait with specified parameters.
        cGaitProfile(double _minhip, double _maxhip, double _minknee, double _maxknee,
                     double _gain_hip, double _gain_knee) :
                     m_maxhip_left(_maxhip),       m_maxhip_right(_maxhip),
                     m_minhip_left(_minhip),       m_minhip_right(_minhip),
                     m_maxknee_left(_maxknee),     m_maxknee_right(_maxknee),
                     m_minknee_left(_minknee),     m_minknee_right(_minknee),
                     m_gain_hip_left(_gain_hip),   m_gain_hip_right(_gain_hip),
                     m_gain_knee_left(_gain_knee), m_gain_knee_right(_gain_knee) {};

        /// Create a new asymmetric gait.
        cGaitProfile(double _minhip_left, double _maxhip_left, double _minknee_left, double _maxknee_left,
                     double _gain_hip_left, double _gain_knee_left,
                     double _minhip_right, double _maxhip_right, double _minknee_right, double _maxknee_right,
                     double _gain_hip_right, double _gain_knee_right) :
                     m_maxhip_left(_maxhip_left),       m_maxhip_right(_maxhip_right),
                     m_minhip_left(_minhip_left),       m_minhip_right(_minhip_right),
                     m_maxknee_left(_maxknee_left),     m_maxknee_right(_maxknee_right),
                     m_minknee_left(_minknee_left),     m_minknee_right(_minknee_right),
                     m_gain_hip_left(_gain_hip_left),   m_gain_hip_right(_gain_hip_right),
                     m_gain_knee_left(_gain_knee_left), m_gain_knee_right(_gain_knee_right) {};

        void print(std::ostream& stream);

        bool operator==(cGaitProfile cmp);
        cGaitProfile operator*(double value);
        cGaitProfile& operator=(cGaitProfile);
        cGaitProfile& operator+=(cGaitProfile);
        cGaitProfile& operator-=(cGaitProfile);
        cGaitProfile operator+(cGaitProfile);
        cGaitProfile operator-(cGaitProfile);

        static void copy_to(cGaitProfile& to, cGaitProfile from);

        double maxhip_left()   { return m_maxhip_left; };
        double minhip_left()   { return m_minhip_left; };
        double maxhip_right()  { return m_maxhip_right; };
        double minhip_right()  { return m_minhip_right; };

        double maxknee_left()  { return m_maxknee_left; };
        double minknee_left()  { return m_minknee_left; };
        double maxknee_right() { return m_maxknee_right; };
        double minknee_right() { return m_minknee_right; };

        double gain_hl_ext()  { return m_gain_hip_left; };
        double gain_hl_flex() { return m_gain_hip_left; };
        double gain_hr_ext()  { return m_gain_hip_right; };
        double gain_hr_flex() { return m_gain_hip_right; };

        double gain_kl_ext()  { return m_gain_knee_left; };
        double gain_kl_flex() { return m_gain_knee_left; };
        double gain_kr_ext()  { return m_gain_knee_right; };
        double gain_kr_flex() { return m_gain_knee_right; };

    private:
        static const double precision;

        double m_maxhip_left, m_maxhip_right;       /// maximum permitted angle of hip
        double m_minhip_left, m_minhip_right;       /// minimum permitted angle of hip joints
        double m_maxknee_left, m_maxknee_right;     /// maximum permitted angle of knee joints
        double m_minknee_left, m_minknee_right;     /// minimum permitted angle of knee joints
        double m_gain_hip_left, m_gain_hip_right;
        double m_gain_knee_left, m_gain_knee_right;
};


}

#endif // CGAITPROFILE_H
