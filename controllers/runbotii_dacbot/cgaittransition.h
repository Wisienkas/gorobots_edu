#ifndef CGAITTRANSITION_H
#define CGAITTRANSITION_H

#include "cgaitprofile.h"

//#include <QtCore/QMutex> We shoudlnt need those here
//#include <QtCore/QMutexLocker> We shouldnt need those here

namespace runbot {

/** General idea: The default constructor should get a profile and
  * fill *this and old_gait.
  * If a new gait is introduced by transition_to(), then the difference between
  * old and new gait is used to determine transition speed with
  * transition_steps.
  */
class cGaitTransition : public cGaitProfile {
    public:
        /// Copy constructor won't copy the Mutex!
        cGaitTransition(const cGaitTransition& _trans)
                : cGaitProfile(_trans) {
            old_gait = _trans.old_gait;
            new_gait = _trans.new_gait;
            num_steps = _trans.num_steps;
            actual_step = 0;
            transition_in_progress = false;
        };

        cGaitTransition(cGaitProfile _profile)
            : cGaitProfile(_profile), old_gait(_profile),
              num_steps(1000), transition_in_progress(false) {};

        cGaitTransition(cGaitProfile _profile, int _num_steps)
            : cGaitProfile(_profile), old_gait(_profile),
              num_steps(_num_steps), transition_in_progress(false) {};


        /// Operator= won't copy the Mutex!
        cGaitTransition& operator=(const cGaitTransition& _trans) {
            cGaitProfile::copy_to(*this, _trans);
            old_gait = _trans.old_gait;
            new_gait = _trans.new_gait;
            num_steps = _trans.num_steps;
            actual_step = 0;
            transition_in_progress = false;
            return *this;
        };


        cGaitProfile& operator=(const cGaitProfile& _prof) {
            cGaitProfile::copy_to(*this, _prof);
            return *this;
        }


        void set_num_transition_steps(int _num_steps) {
            // TODO: Extra handling, if transition in progress;
            //       Exception?
            num_steps = _num_steps;
        };

        void transition_to(cGaitProfile _profile) {
          //  QMutexLocker mutex_locker(&transition_mutex);

            if(! (new_gait == _profile)) {
                old_gait = *this;       // In case of switch inbetween switch
                new_gait = _profile;
                transition_in_progress = true;
                actual_step = 0;
            }
        };

        void do_transition_step() {
         //   QMutexLocker mutex_locker(&transition_mutex);

            if (transition_in_progress) {
                if (++actual_step == num_steps) {
                    *this = new_gait;
                    old_gait = new_gait;
                    transition_in_progress = false;
                } else {
                    *this = old_gait + (new_gait - old_gait) * ((double)actual_step / (double)num_steps);
                }
            }
        }

        cGaitProfile get_old_gait(){return old_gait;}
        cGaitProfile get_new_gait(){return new_gait;}
        int get_num_steps(){return num_steps;}

    private:
        cGaitProfile old_gait, new_gait;
        int num_steps, actual_step;
        bool transition_in_progress;
     //   QMutex transition_mutex;
};

}

#endif // CGAITTRANSITION_H
