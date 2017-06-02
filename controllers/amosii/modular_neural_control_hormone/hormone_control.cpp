#include "hormone_control.h"
#include <ode_robots/amosiisensormotordefinition.h>

//for cout
#include <selforg/controller_misc.h>

using namespace matrix;

using namespace Hormone;

using namespace std;

hormone_control::hormone_control() {
  t = 0;
  BufferSize = 50;
  LegSize = 6;
  JointSize = LegSize * 3;
  FootSize = 6;
  m_pre_1.resize(AMOSII_MOTOR_MAX);
  m_pre_edit.resize(AMOSII_MOTOR_MAX);
  m_pre_edit_p.resize(AMOSII_MOTOR_MAX);

  /////////////////////////////////

  //hormone size relative LegSize
  //@ if you want to add number of size plz plus parameter
  GlandSize = JointSize + FootSize;
  HormoneSize = LegSize + 1;
  ReceptorSize = LegSize + 1;
  /////////////////////////////////

  //prepare input and output vector size
  in0.resize(AMOSII_SENSOR_MAX);
  for (unsigned int i = 0; i < in0.size(); i++) {
    in0[i].resize(BufferSize);
  }

  out0.resize(JointSize);
  for (unsigned int i = 0; i < out0.size(); i++) {
    out0[i].resize(BufferSize);
  }
  /*
   Gait.resize(LegSize);
   for (unsigned int i = 0; i < Gait.size(); i++) {
   Gait[i].resize(BufferSize);
   }
   */
  //create Artificial hormone network
  HGland = new gland[GlandSize]; //Address 0-17 for all joint for AmosII
  HTank = new hormone[HormoneSize]; //Address 0-5 for leg for amosII
  HReceptor = new receptor[ReceptorSize]; //Address 0-5 for leg for amosII

  //config parameter hormone
  unsigned int param = 3;
  unsigned int param2 = 6;

  for (unsigned int i = 0; i < LegSize; i++) {

    //Leg
    // gland TR0 + gland CR0 +gland FR0 = hormone tank => receptor[0]

	HGland[TR0_m + i].setALPHA(0.15);
    HGland[TR0_m + i].LinkToHormone(&HTank[TR0_m + i]);
    HGland[TR0_m + i].data.setInputType(HM_SD, HM_RECORD_FIFO);
    HGland[TR0_m + i].data.setBufferSize(param);

    /*
     HGland[CR0_m+i].setALPHA(0.03415);
     HGland[CR0_m+i].LinkToHormone(&HTank[TR0_m+i]);
     HGland[CR0_m+i].data.setInputType(HM_Mean, HM_RECORD_FIFO);
     HGland[CR0_m+i].data.setBufferSize(param);

     HGland[FR0_m+i].setALPHA(0.03415);
     HGland[FR0_m+i].LinkToHormone(&HTank[TR0_m+i]);
     HGland[FR0_m+i].data.setInputType(HM_Mean, HM_RECORD_FIFO);
     HGland[FR0_m+i].data.setBufferSize(param);
     */
    HTank[TR0_m + i].setBETA(0.0105);
    HReceptor[i].AddHormoneReceptor(&HTank[TR0_m + i], 0.0005, 1);

    //Foot contact
    HGland[R0_fs-2].setALPHA(0.08);
    HGland[R0_fs-2].LinkToHormone(&HTank[6]);
    HGland[R0_fs-2].data.setInputType(HM_Mean, HM_RECORD_FIFO);
    HGland[R0_fs-2].data.setBufferSize(param2);
    //Hormone Tank for foot contact
    HTank[6].setBETA(0.0105);
    HReceptor[6].AddHormoneReceptor(&HTank[6], 0.0005, 1);

  }
  for(int i=0;i<6;i++){
  	  d_case[i][0] = 0;
  	  t_i[i] = 0;
  	  t_f[i] = 0;
  	  dt[i] = 0;
  	  check_fall[i][0] = 0;
  	  cp[i][0] = 0;
  	  cp[i][1] = 0;
  	  dt_cp[i] = 1;
  	  m_pre_edit.at(CR0_m+i) = 0;
  	  m_pre_edit_p.at(CR0_m+i) = 0;
    }

  outFilenlc1.open("hormonecontroller.txt");
  outFilenlc2.open("B-Check.txt");
  outFilenlc1 << "HG[0] HT[0] HG[1] HT[1] HG[2] HT[2] HG[3] HT[3] HG[4] HT[4] HG[5] HT[5] " << endl;
  for(int i = 0;i<6;i++){
	  outFilenlc2 << "m_pre["<< i <<"] " << "m_pre_edit["<< i <<"] " << "in0["<< i <<"] " << "error["<< i <<"] " ;
  }
  outFilenlc2 << "sum_error" << endl ;
  /*for(int w=0;w<50;w++){
	  outFilenlc2 << "CR[" << w <<"]" << ' ' ;
  }
  outFilenlc2 << endl;*/
}
;

hormone_control::~hormone_control() {
  //Save files
  outFilenlc1.close();
  outFilenlc2.close();
}

void hormone_control::setBuffersize(unsigned int size) {
  BufferSize = size;
}

void hormone_control::StimulateInput(const std::vector<double> x, const std::vector<double> y) {
  unsigned int count = 0;
  t++ ; //t count
  for (unsigned int i = 0; i < 115; i++) {
//      if((i != TR1_as) && (i != TL1_as) && (i != CR1_as) && (i != CL1_as) && (i != FR1_as) && (i != FL1_as))
    {
//        std::cout << "count_in =" << count << endl;
      in0[count].push_back(x.at(i));

      if (in0[count].size() > BufferSize) {
        in0[count].erase(in0[count].begin());
      }
      count++;

    }
  }

  count = 0;

  for (int i = 0; i < 18; i++) {
	  int check = 0;
	  for(int j = 0;j < 6;j++){ if(CR0_m+j == i){ check = 1; } }
	  if(check ==1){ out0[count].push_back(m_pre_1.at(i)); }
	  else{ out0[count].push_back(y.at(i)); }
    if (out0[count].size() > BufferSize) {
      out0[count].erase(out0[count].begin());
    }
    count++;
  }
  count = 0;

  	  //outFilenlc2 << dt[i] << ' ' << dt[i]-t_ref[i] << ' ' << ((out0[CR0_m+i].at(49)*-1)+1)*0.25 << ' '<< (in0[R0_fs+i].at(49)) << ' ' << dt_cp[i] << ' ';

  double w = 1.3 ;
    for(int i = 0;i < 6 ; i++){
  	 m_pre_edit.at(CR0_m+i) = (((out0[CR0_m+i].at(49)*-1)+1)*w) + (m_pre_edit_p.at(CR0_m+i)*(w-1)) ;

  	 //Convert to 0..1 if the sensory data is the raw data without
  	 //But if you have raw sensor data between 0...1  the comment this part!!!
  	 //in0[R0_fs+i].at(49) = in0[R0_fs+i].at(49)/100.0 ;


  	 double offset_mpre = 0.22; // set to 0.22 if the raw data is not converted to 0...1 but set to xxx if the raw foot sensor is between 0..1
  	 m_pre_edit.at(CR0_m+i) = m_pre_edit.at(CR0_m+i)*offset_mpre ;
  	 error[i] = m_pre_edit.at(CR0_m+i) - in0[R0_fs+i].at(49);
  	 outFilenlc2 << m_pre_1.at(CR0_m+i) << ' ' << m_pre_edit.at(CR0_m+i)*offset_mpre << ' ' << in0[R0_fs+i].at(49) << ' ' << error[i]*0.4 <<' ';
    }
    double sum=0 ;
    for(int i =0;i<6;i++){
  	  sum = sum+error[i];
    }
    sum = sum/6.0 ;
    outFilenlc2 << sum << endl;
    UpdateInput();
    for(int i =0;i<6;i++){
  	  m_pre_edit_p.at(CR0_m+i) = m_pre_edit.at(CR0_m+i) ;
    }


  /*for(int w=0;w<50;w++){
  		  outFilenlc2 << out0[CR0_m].at(w) << ' ' ;
  	}
  	outFilenlc2 << endl ; //test for understand how to get value*/

}

void hormone_control::StimulateInput(const vector<double> x, const vector<double> y, const vector<double> MI) {

  for (unsigned int i = 0; i < AMOSII_SENSOR_MAX; i++) {

    //load data all sensor
    in0[i].push_back(x.at(i));

    if (in0[i].size() > BufferSize) {
      in0[i].erase(in0[i].begin());
    }

    if (i < JointSize) {
      //load command motor
      out0[i].push_back(y.at(i));
      if (out0[i].size() > BufferSize) {
        out0[i].erase(out0[i].begin());
      }
    }
    if (i < LegSize) {
      Gait[i].push_back(MI.at(i));
      if (Gait[i].size() > BufferSize) {
        Gait[i].erase(Gait[i].begin());
      }
    }

  }
  UpdateInput();

}

void hormone_control::UpdateInput() {
  // add data to gland
  /*
   HGland[TR0_m].data.Add(Correlation(in0[TR0_as],out0[TR0_m]));
   HGland[TL0_m].data.Add(Correlation(in0[TL0_as],out0[TL0_m]));
   HGland[CR0_m].data.Add(Correlation(in0[CR0_as],out0[CR0_m]));
   HGland[CL0_m].data.Add(Correlation(in0[CL0_as],out0[CL0_m]));
   HGland[FR0_m].data.Add(Correlation(in0[FR0_as],out0[FR0_m]));
   HGland[FL0_m].data.Add(Correlation(in0[FL0_as],out0[FL0_m]));

   HGland[TR1_m].data.Add(Correlation(in0[TR1_as],out0[TR1_m]));
   HGland[TL1_m].data.Add(Correlation(in0[TL1_as],out0[TL1_m]));
   HGland[CR1_m].data.Add(Correlation(in0[CR1_as],out0[CR1_m]));
   HGland[CL1_m].data.Add(Correlation(in0[CL1_as],out0[CL1_m]));
   HGland[FR1_m].data.Add(Correlation(in0[FR1_as],out0[FR1_m]));
   HGland[FL1_m].data.Add(Correlation(in0[FL1_as],out0[FL1_m]));

   HGland[TR2_m].data.Add(Correlation(in0[TR2_as],out0[TR2_m]));
   HGland[TL2_m].data.Add(Correlation(in0[TL2_as],out0[TL2_m]));
   HGland[CR2_m].data.Add(Correlation(in0[CR2_as],out0[CR2_m]));
   HGland[CL2_m].data.Add(Correlation(in0[CL2_as],out0[CL2_m]));
   HGland[FR2_m].data.Add(Correlation(in0[FR2_as],out0[FR2_m]));
   HGland[FL2_m].data.Add(Correlation(in0[FL2_as],out0[FL2_m]));
   */

  HGland[TR0_m].data.Add(Correlation(in0[TR0_as], out0[TR0_m]));
  HGland[TL0_m].data.Add(Correlation(in0[TL0_as], out0[TL0_m]));
  HGland[TR0_m].data.Add(Correlation(in0[CR0_as], out0[CR0_m]));
  HGland[TL0_m].data.Add(Correlation(in0[CL0_as], out0[CL0_m]));
  HGland[TR0_m].data.Add(Correlation(in0[FR0_as], out0[FR0_m]));
  HGland[TL0_m].data.Add(Correlation(in0[FL0_as], out0[FL0_m]));

  HGland[TR1_m].data.Add(Correlation(in0[TR1_as], out0[TR1_m]));
  HGland[TL1_m].data.Add(Correlation(in0[TL1_as], out0[TL1_m]));
  HGland[TR1_m].data.Add(Correlation(in0[CR1_as], out0[CR1_m]));
  HGland[TL1_m].data.Add(Correlation(in0[CL1_as], out0[CL1_m]));
  HGland[TR1_m].data.Add(Correlation(in0[FR1_as], out0[FR1_m]));
  HGland[TL1_m].data.Add(Correlation(in0[FL1_as], out0[FL1_m]));

  HGland[TR2_m].data.Add(Correlation(in0[TR2_as], out0[TR2_m]));
  HGland[TL2_m].data.Add(Correlation(in0[TL2_as], out0[TL2_m]));
  HGland[TR2_m].data.Add(Correlation(in0[CR2_as], out0[CR2_m]));
  HGland[TL2_m].data.Add(Correlation(in0[CL2_as], out0[CL2_m]));
  HGland[TR2_m].data.Add(Correlation(in0[FR2_as], out0[FR2_m]));
  HGland[TL2_m].data.Add(Correlation(in0[FL2_as], out0[FL2_m]));

  //foot contact
  for(int i =0;i<6;i++){
	  HGland[R0_fs-2].data.Add(error[i]*0.4);
  }
  HGland[R0_fs-2].produceHormone();
  HTank[6].Active();
  for (unsigned int i = 0; i < LegSize; i++) {
    //save data hormone all
    //outFilenlc1 << ........

    //update gland
    HGland[TR0_m + i].produceHormone();
    //   HGland[CR0_m+ i].produceHormone();
    //   HGland[FR0_m+ i].produceHormone();

    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    //debug
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    // if (HGland[i].data.getBufferNow() % 120 == 0)
    // {
    //    HGland[TR0_m + i].produceHormone();
    // }
    /*
     std::cout <<"corr_TR["<<i <<"] = " << Correlation(in0[TR0_m+i],out0[TR0_m+i]) << endl;
     std::cout <<"corr_CR["<<i <<"] = " << Correlation(in0[CR0_m+i],out0[CR0_m+i]) << endl;
     std::cout <<"corr_FR["<<i <<"] = " << Correlation(in0[FR0_m+i],out0[FR0_m+i]) << endl;
     */
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    // outFilenlc1 << HGland[i].getHOR() << ' ';
    outFilenlc1 << HTank[i].G << ' ';

    //update hormone concentration
    HTank[i].Active();

    outFilenlc1 << HTank[i].hormoneValue << ' ';

  }

  outFilenlc1 << endl;

  /*
   // save data input
   outFilenlc1 << ' ' << Correlation(in0[TR0_as],out0[TR0_m]) << ' ' << Correlation(in0[TL0_as],out0[TL0_m])\
                << ' ' << Correlation(in0[CR0_as],out0[CR0_m]) << ' ' << Correlation(in0[CL0_as],out0[CL0_m])\
                << ' ' << Correlation(in0[FR0_as],out0[FR0_m]) << ' ' << Correlation(in0[FL0_as],out0[FL0_m])\
   ////comment for test 4leg

   << ' ' << Correlation(in0[TR1_as],out0[TR1_m]) << ' ' << Correlation(in0[TL1_as],out0[TL1_m])\
                << ' ' << Correlation(in0[CR1_as],out0[CR1_m]) << ' ' << Correlation(in0[CL1_as],out0[CL1_m])\
                << ' ' << Correlation(in0[FR1_as],out0[FR1_m]) << ' ' << Correlation(in0[FL1_as],out0[FL1_m])
   << ' ' << Correlation(in0[TR2_as],out0[TR2_m]) << ' ' << Correlation(in0[TL2_as],out0[TL2_m])\
                << ' ' << Correlation(in0[CR2_as],out0[CR2_m]) << ' ' << Correlation(in0[CL2_as],out0[CL2_m])\
                << ' ' << Correlation(in0[FR2_as],out0[FR2_m]) << ' ' << Correlation(in0[FL2_as],out0[FL2_m])\
                ;//<< endl;
   */

  sum_error = 0.0;

  unsigned int count = 0;
  for (unsigned int i = 0; i < JointSize; i++) {
    //if((i != TR1_as) && (i != TL1_as) && (i != CR1_as) && (i != CL1_as) && (i != FR1_as) && (i != FL1_as))
    {
      count++;
      sum_error += Correlation(in0[i], out0[i]);
    }
  }
  sum_error = (count - sum_error) / (double) count;

  //std::cout << "hormone_value1 = " << h1->hormoneValue << "\n";

  //addInspectableValue("H1",&h1->hormoneValue,"_0"); //to_string() is C++11
  //Inspectable::addInspectableValue("CPG[" + to_string(0) + "]",&h1->hormoneValue, "R"+ to_string(0) + "_0"); //to_string() is C++11
  /*
   std::cout << "conv[TR] " << Convolution(in0[TR0_as],out0[TR0_m]) << "\n";
   std::cout << "conv[TL] " << Convolution(in0[TL0_as],out0[TL0_m]) << "\n";
   std::cout << "conv[CR] " << Convolution(in0[CR0_as],out0[CR0_m]) << "\n";
   std::cout << "conv[CL] " << Convolution(in0[CL0_as],out0[CL0_m]) << "\n";
   std::cout << "conv[FR] " << Convolution(in0[FR0_as],out0[FR0_m]) << "\n";
   std::cout << "conv[FL] " << Convolution(in0[FL0_as],out0[FL0_m]) << "\n";
   */

  /*
   std::cout << "corr[TR] " << Correlation(in0[TR0_as],out0[TR0_m]) << "\n";

   std::cout << "corr[TL] " << Correlation(in0[TL0_as],out0[TL0_m]) << "\n";
   std::cout << "corr[CR] " << Correlation(in0[CR0_as],out0[CR0_m]) << "\n";
   std::cout << "corr[CL] " << Correlation(in0[CL0_as],out0[CL0_m]) << "\n";
   std::cout << "corr[FR] " << Correlation(in0[FR0_as],out0[FR0_m]) << "\n";
   std::cout << "corr[FL] " << Correlation(in0[FL0_as],out0[FL0_m]) << "\n";
   */

}

DATATYPE hormone_control::getReceptor(unsigned int index) {
  DATATYPE output = 0.0;
  DATATYPE reduce = 0.0;
  switch (index) {
    case 0:
      reduce = HReceptor[0].reduceHormone();
      //reduce = r11->reduceHormone();
      //outFilenlc1 << ' ' << reduce ;

      //std::cout << "recptor = " << output << "\n";
      output = RangeCast<DATATYPE>(reduce, 0, 0.0005, 0.985, 1.025);
      //outFilenlc1 << ' ' << output ;

      output = RangeCast<DATATYPE>(reduce, 0, 0.0005, 0.999, 1.001);
      //outFilenlc1 << ' ' << output ;

      output = RangeCast<DATATYPE>(reduce, 0, 0.0005, 0.998, 1.002);

      //outFilenlc1 << ' ' << output << endl;
      //output = RangeCast<DATATYPE>(output, 0, 0.0005, 1.002,0.998);
      //std::cout << "output(cast) = " << output << "\n";
      break;

    case 1:
      reduce = HReceptor[1].reduceHormone();
      break;
    case 2:
      reduce = HReceptor[2].reduceHormone();
      break;
    case 3:
      reduce = HReceptor[3].reduceHormone();
      break;
    case 4:
      reduce = HReceptor[4].reduceHormone();
      break;
    case 5:
      reduce = HReceptor[5].reduceHormone();
      break;
    case 6:
      reduce = HReceptor[6].reduceHormone();

  }
  output = RangeCast<DATATYPE>(reduce, 0, 0.0005, 0.0 , 1.2);
  return output;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

DATATYPE hormone_control::Convolution(const vector<double> x, const vector<double> y) {
  std::vector<long double> conv;
  double norm = 0;
  DATATYPE out_conv;

  conv.resize(x.size());

  //Convolution
  for (unsigned int i = 0; i < x.size(); i++) {
    conv.at(i) = 0; // set to zero before sum
    for (unsigned int j = 0; j < y.size(); j++) {
      if (i - j >= 0) {
        conv.at(i) += x[i - j] * y[j]; // convolve: multiply and accumulate
      }
    }
    //Normalize
    if (conv.at(i) != 0) {
      norm += pow(conv.at(i), 2);
    }
  }

  double max_conv = *std::max_element(std::begin(conv), std::end(conv));

  if (norm != 0) {
    out_conv = max_conv / sqrt(norm);
  } else {
    out_conv = 0;
  }

  return out_conv;

}

DATATYPE hormone_control::Correlation(const vector<double> x, const vector<double> y) {
  DATATYPE correlation;
  DATATYPE cov_xy = calcCov(x, y);
  DATATYPE sd_x = calcSD(x);
  DATATYPE sd_y = calcSD(y);

  if (cov_xy == 0) {
    correlation = 0;
  } else {
    correlation = cov_xy / sqrt(sd_x * sd_y);
  }

  if (correlation < 0.0) {
    correlation = -correlation;
  }

  return correlation;
}

DATATYPE hormone_control::calcSD(vector<double> inputVector) {
  double Xbar = calcMean(inputVector);
  double sum = 0;
  for (unsigned int i = 0; i < inputVector.size(); i++) {
    sum += pow(inputVector[i] - Xbar, 2);
  }
  sum /= inputVector.size();
  return (DATATYPE) sum;
}
DATATYPE hormone_control::calcMean(vector<double> inputVector) {
  DATATYPE sum = 0;
  for (unsigned int i = 0; i < inputVector.size(); i++) {
    sum += inputVector[i];
  }
  sum /= inputVector.size();
  return sum;
}

DATATYPE hormone_control::calcCov(const vector<double> x, const vector<double> y) {

  DATATYPE Xbar = calcMean(x);
  DATATYPE Ybar = calcMean(y);

  DATATYPE sum = 0;

  for (unsigned int i = 0; i < x.size(); i++) {
    sum += (x[i] - Xbar) * (y[i] - Ybar);
  }

  return sum / x.size();
}
