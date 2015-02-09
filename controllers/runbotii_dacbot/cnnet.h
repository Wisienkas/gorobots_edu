#pragma once
#ifndef CNNET_H
#define CNNET_H

#include <valarray>
using std::valarray;

#include <string>

#include "config.h"
#include "cgaitprofile.h"

namespace runbot {

class cNNet {
    public:
        cNNet(cGaitProfile* profile_);

        /** \brief Updating the neuronal network with input data.
         *
         * Preprocessing of sensory input and connecting to the network!
         * to get the final motor output from the network.
         */
        valarray< double > update(valarray< double > input_data, int time);


        /** \brief Return the vector of neuron outputs. */
        valarray< double > get_neuron_outputs();


        /** \brief Calculating the inner activation of a neuron for single connected neurons.
         *
         * \param y previous activiation
         * \param T time constant = (1. - \frac{\delta t}{\tau})
         * \param w input weight
         * \param r input
         *
         * Returns the new activation y_{i+1} with
         *     $y_{i+y} = y_i T + \sum w * r$
         */
        static double activation(double y, double T, double w, double r) {
            return y*T+(1-T)*(w*r);
        }

    private:
        cGaitProfile* gait;

        void update_thresholds();
        void update_nnet(valarray< double > input_data);
        void dump_output_vector(int time);

        /** \brief Post processing output signal from the network
         *
         * Actual network configuration: 8 output signals from neuronal network
         *     u_hl_em             u_hl_fm
         *     u_kl_em             u_kl_fm
         *     u_hr_em             u_hr_fm
         *     u_kr_em             u_kr_fm
         *
         * This function is to generate 4 output voltages for motors. Every pair
         * of neuron output is combined as of
         * $$
         *     gain_em * u_em - gain_fm * u_fm.
         * $$
         * As can be seen, this version defines the extensor sign to be positive
         * and the flexor sign to be negative.
         * If these signs are switched a motor-sign can be used to invert the result
         * in the io module.
         * Maximum voltage checking is done in the io module, too.
         */
        valarray<double> update_motorvoltages();

        /** \brief Calculating the inner activation of a neuron.
         *
         * \param y previous activiation
         * \param T time constant = (1. - \frac{\delta t}{\tau})
         * \param w array of weights
         * \param r array of inputs
         *
         * Returns the new activation y_{i+1} with
         *     $y_{i+y} = y_i T + \sum w * r$
         */
        double activation(double y, double T, valarray<double> w, valarray<double> r) {
            return y*T+(1-T)*((w*r).sum());
        }


        int countpiezo;     // counter for getting piezo data
        int leftpiezopre;   // piezo data of left foot at previous step
        int rightpiezopre;

        //Knee Right
        double y_kr_ei;
        double y_kr_fi;
        double y_kr_em;
        double y_kr_fm;
        double u_kr_es;
        double u_kr_fs;
        double u_kr_ei;
        double u_kr_fi;
        double u_kr_em;
        double u_kr_fm;
        double w_kr_es_em;
        double w_kr_fs_fm;
        double w_kr_ei_em;
        double w_kr_fi_fm;
        double w_kr_ei_fm;
        double w_kr_fi_em;
        double threshold_kr_es;
        double threshold_kr_fs;
        double elf_kr_es;
        double elf_kr_fs;

        double y_pre_kr_ei;
        double y_pre_kr_fi;
        double y_pre_kr_em;
        double y_pre_kr_fm;

        // double angle_hl;    // Hip Left joint angle in degree, get rom channel 4
        // double angle_hr;    // Hip Right, channel 5
        // double angle_kl;    // Knee Left, channel 6
        // double angle_kr;    // Knee Right, channel 7
        // double angle_boom;  // boom angle

        double threshold_al; //threshold of left AEA receptor
        double threshold_ar; //threshold of right AEA receptor
        double u_al; //output of left AEA receptor
        double w_al_kl_ei; //weight from left AEA receptor to left knee extensor inter-neuron
        double w_al_kl_fi; //weight from left AEA receptor to left knee flexor inter-neuron
        double u_ar; //output of right AEA receptor
        double w_ar_kr_ei; //weight from right AEA receptor to right knee extensor inter-neuron
        double w_ar_kr_fi; //weight from right AEA receptor to right knee flexor inter-neuron
        double elf_al; //time constant of left AEA receptor
        double elf_ar; //time constant of right AEA receptor

        int u_gl; // out put of left ground contact sensor neuron
        double w_gl_hl_fi ;
        double w_gl_kl_ei ;
        double w_gl_hr_ei ;
        double w_gl_kr_fi ;

        int u_gr; // output of right ground contact sensor neuron
        double w_gr_hl_ei ;
        double w_gr_kl_fi ;
        double w_gr_hr_fi ;
        double w_gr_kr_ei ;

        //Hip Right
        double y_hr_ei;
        double y_hr_fi;
        double y_hr_em;
        double y_hr_fm;
        double u_hr_es;
        double u_hr_fs;
        double u_hr_ei;
        double u_hr_fi;
        double u_hr_em;
        double u_hr_fm;
        double w_hr_es_em;
        double w_hr_fs_fm;
        double w_hr_ei_em;
        double w_hr_fi_fm;
        double w_hr_ei_fm;
        double w_hr_fi_em;
        double threshold_hr_es;
        double threshold_hr_fs;
        double elf_hr_es;
        double elf_hr_fs;

        double y_pre_hr_ei;
        double y_pre_hr_fi;
        double y_pre_hr_em;
        double y_pre_hr_fm;


        //Knee Left
        double y_kl_ei;
        double y_kl_fi;
        double y_kl_em;
        double y_kl_fm;
        double u_kl_es;
        double u_kl_fs;
        double u_kl_ei;
        double u_kl_fi;
        double u_kl_em;
        double u_kl_fm;
        double w_kl_es_em;
        double w_kl_fs_fm;
        double w_kl_ei_em;
        double w_kl_fi_fm;
        double w_kl_ei_fm;
        double w_kl_fi_em;
        double threshold_kl_es;
        double threshold_kl_fs;
        double elf_kl_es;
        double elf_kl_fs;

        double y_pre_kl_ei;
        double y_pre_kl_fi;
        double y_pre_kl_em;
        double y_pre_kl_fm;

        double elf_knee;    // time constant of sensor neurons of knee joints
        double elf_hip_es;    // time constant of hip extensor sensor neurons
        double elf_hip_fs;    // time constant of hip flexor sensor neurons

        double elf_es;  //time constant of extensor sensor neurons
        double elf_fs;  // time constant of flexor sensor neurons
        double threshold_em;  // threshold of extensor motor neurons
        double threshold_fm;  // threshold of flexor motor neurons
        double threshold_ei;  // threshold of extensor inter-neurons
        double threshold_fi;  // threshold of flexor inter-neurons

        //Hip Left joint neuron parameters
        double y_hl_ei; //membrain potential of extensor interneuron
        double y_hl_fi; //membrain potential of flexor interneuron
        double y_hl_em; //membrain potential of extensor motor-neuron
        double y_hl_fm; //membrain potential of flexor motor-neuron
        double u_hl_es; //output of extensor sensor neuron
        double u_hl_fs; // output of flexor sensor neuron
        double u_hl_ei; //output of extensor inter-neuron
        double u_hl_fi; //output of flexor inter-neuron
        double u_hl_em; //output of extensor motor-neuron
        double u_hl_fm; //output of flexor motor-neuron
        double w_hl_es_em; //weight from extensor sensor neuron to extensor motor-neuron
        double w_hl_fs_fm; //weight from flexor sensor neiron to flexor motor-neuron
        double w_hl_ei_em; //weight from extensor inter-neuron to extensor motor-neuron
        double w_hl_fi_fm; //weight from flexor inter-neuron to flexor motor-neuron
        double w_hl_ei_fm; //weight from extensor interneuron to flexor motor-neuron
        double w_hl_fi_em; //weight from flexor inter-neuron to extensor motor-neuron
        double threshold_hl_es; //threshold of extensor sensor neuron
        double threshold_hl_fs; // threshold of flexor sensor neuron
        double elf_hl_es; //time constant of extensor sensor neuron
        double elf_hl_fs; //time constant of flexor sensor neuron

        double y_pre_hl_ei; //membrain potential of extensor interneuron at last step
        double y_pre_hl_fi; //membrain potential of flexor interneuron at last step
        double y_pre_hl_em; //membrain potential of extensor motor-neuron at last step
        double y_pre_hl_fm; //membrain potential of flexor motor-neuron at last step

        double angle_hl_pre; // angle of left hip joint at some steps before
        double angle_hr_pre;
        double angle_hl_now;
        double angle_hr_now;

        static const double expp;   // 1 ms .. means xep(-1/tau) ,for knee neurons,
                                    // tau is bigger than that of hips, in order to
                                    // eliminate the noise of motor neuron output.
        static const double exppp;  //  1 ms.. for hip neurons

        static const double eps;    // Precision for comparisons of floats.

        WRITER_CLASS    array_writer;
        static const unsigned int writer_columns;
        static const std::string writer_column_names;

};

}

#endif // CNNET_H
