#include "cgaitprofile.h"

#include <cmath>

using namespace runbot;

const double cGaitProfile::precision = 1e-8;


bool cGaitProfile::operator== (cGaitProfile cmp) {

    return
        (m_minhip_left    == cmp.m_minhip_left)  &&
        (m_maxhip_left    == cmp.m_maxhip_left)  &&
        (m_minknee_left   == cmp.m_minknee_left) &&
        (m_maxknee_left   == cmp.m_maxknee_left) &&
        (fabs(m_gain_hip_left  - cmp.m_gain_hip_left ) <= precision) &&
        (fabs(m_gain_knee_left - cmp.m_gain_knee_left) <= precision) &&
        (m_minhip_right    == cmp.m_minhip_right)  &&
        (m_maxhip_right    == cmp.m_maxhip_right)  &&
        (m_minknee_right   == cmp.m_minknee_right) &&
        (m_maxknee_right   == cmp.m_maxknee_right) &&
        (fabs(m_gain_hip_right  - cmp.m_gain_hip_right ) <= precision) &&
        (fabs(m_gain_knee_right - cmp.m_gain_knee_right) <= precision);
}


cGaitProfile cGaitProfile::operator* (double value) {
    cGaitProfile result(*this);

    result.m_minhip_left    *= value;
    result.m_maxhip_left    *= value;
    result.m_minknee_left   *= value;
    result.m_maxknee_left   *= value;
    result.m_gain_hip_left  *= value;
    result.m_gain_knee_left *= value;

    result.m_minhip_right    *= value;
    result.m_maxhip_right    *= value;
    result.m_minknee_right   *= value;
    result.m_maxknee_right   *= value;
    result.m_gain_hip_right  *= value;
    result.m_gain_knee_right *= value;

    return result;
}


cGaitProfile& cGaitProfile::operator+= (cGaitProfile _addend) {

    m_minhip_left    += _addend.m_minhip_left;
    m_maxhip_left    += _addend.m_maxhip_left;
    m_minknee_left   += _addend.m_minknee_left;
    m_maxknee_left   += _addend.m_maxknee_left;
    m_gain_hip_left  += _addend.m_gain_hip_left;
    m_gain_knee_left += _addend.m_gain_knee_left;

    m_minhip_right    += _addend.m_minhip_right;
    m_maxhip_right    += _addend.m_maxhip_right;
    m_minknee_right   += _addend.m_minknee_right;
    m_maxknee_right   += _addend.m_maxknee_right;
    m_gain_hip_right  += _addend.m_gain_hip_right;
    m_gain_knee_right += _addend.m_gain_knee_right;

    return *this;
}


cGaitProfile cGaitProfile::operator+ (cGaitProfile _addend) {
    cGaitProfile result(*this);

    result.m_minhip_left    += _addend.m_minhip_left;
    result.m_maxhip_left    += _addend.m_maxhip_left;
    result.m_minknee_left   += _addend.m_minknee_left;
    result.m_maxknee_left   += _addend.m_maxknee_left;
    result.m_gain_hip_left  += _addend.m_gain_hip_left;
    result.m_gain_knee_left += _addend.m_gain_knee_left;

    result.m_minhip_right    += _addend.m_minhip_right;
    result.m_maxhip_right    += _addend.m_maxhip_right;
    result.m_minknee_right   += _addend.m_minknee_right;
    result.m_maxknee_right   += _addend.m_maxknee_right;
    result.m_gain_hip_right  += _addend.m_gain_hip_right;
    result.m_gain_knee_right += _addend.m_gain_knee_right;

    return result;
}


cGaitProfile& cGaitProfile::operator-= ( cGaitProfile _addend ) {

    m_minhip_left    -= _addend.m_minhip_left;
    m_maxhip_left    -= _addend.m_maxhip_left;
    m_minknee_left   -= _addend.m_minknee_left;
    m_maxknee_left   -= _addend.m_maxknee_left;
    m_gain_hip_left  -= _addend.m_gain_hip_left;
    m_gain_knee_left -= _addend.m_gain_knee_left;

    m_minhip_right    -= _addend.m_minhip_right;
    m_maxhip_right    -= _addend.m_maxhip_right;
    m_minknee_right   -= _addend.m_minknee_right;
    m_maxknee_right   -= _addend.m_maxknee_right;
    m_gain_hip_right  -= _addend.m_gain_hip_right;
    m_gain_knee_right -= _addend.m_gain_knee_right;

    return *this;
}


cGaitProfile cGaitProfile::operator- ( cGaitProfile _addend ) {
    cGaitProfile result(*this);

    result.m_minhip_left    -= _addend.m_minhip_left;
    result.m_maxhip_left    -= _addend.m_maxhip_left;
    result.m_minknee_left   -= _addend.m_minknee_left;
    result.m_maxknee_left   -= _addend.m_maxknee_left;
    result.m_gain_hip_left  -= _addend.m_gain_hip_left;
    result.m_gain_knee_left -= _addend.m_gain_knee_left;

    result.m_minhip_right    -= _addend.m_minhip_right;
    result.m_maxhip_right    -= _addend.m_maxhip_right;
    result.m_minknee_right   -= _addend.m_minknee_right;
    result.m_maxknee_right   -= _addend.m_maxknee_right;
    result.m_gain_hip_right  -= _addend.m_gain_hip_right;
    result.m_gain_knee_right -= _addend.m_gain_knee_right;

    return result;
}


cGaitProfile& cGaitProfile::operator= ( cGaitProfile _addend ) {
    copy_to(*this, _addend);
    return *this;
}


void cGaitProfile::copy_to ( cGaitProfile& to, cGaitProfile from ) {

    to.m_minhip_left    = from.m_minhip_left;
    to.m_maxhip_left    = from.m_maxhip_left;
    to.m_minknee_left   = from.m_minknee_left;
    to.m_maxknee_left   = from.m_maxknee_left;
    to.m_gain_hip_left  = from.m_gain_hip_left;
    to.m_gain_knee_left = from.m_gain_knee_left;

    to.m_minhip_right    = from.m_minhip_right;
    to.m_maxhip_right    = from.m_maxhip_right;
    to.m_minknee_right   = from.m_minknee_right;
    to.m_maxknee_right   = from.m_maxknee_right;
    to.m_gain_hip_right  = from.m_gain_hip_right;
    to.m_gain_knee_right = from.m_gain_knee_right;

}


void cGaitProfile::print(std::ostream& stream) {
    stream << "Hip  min  : " << minhip_left()  << "  " << minhip_right()  << "\n"
           << "     max  : " << maxhip_left()  << "  " << maxhip_right()  << "\n"
           << "     gain : " << gain_hl_flex() << "  " << gain_hr_flex() << "\n"
           << "Knee min  : " << minknee_left() << "  " << minknee_right() << "\n"
           << "     max  : " << maxknee_left() << "  " << maxknee_right() << "\n"
           << "     gain : " << gain_kl_flex() << "  " << gain_kr_flex() << "\n";
}
