#include "hormone.h"

//for cout
#include <selforg/controller_misc.h>

namespace Hormone {
  //PRECLASS
  class gland;
  class hormone;
  class Data;
  class receptor;
}
;

//GLAND FUNCTION////////////////////////////////////////////////////////
//Data class////////////////////////////////////////////////////////
Hormone::gland::Data::Data() {
  BufferType = 0;
  BufferNow = 0;
  DataType = 0;
  Buffersize = 0;
}
void Hormone::gland::Data::Add(DATATYPE RawData) {

  DataBuffer.push_back(RawData);
  BufferNow++;
  switch (DataType) {
    case 0:
      OUTPUT = RawData;
      break;
    case 1:
      if (BufferNow >= Buffersize) {
        OUTPUT = calcSD(DataBuffer);
        switch (BufferType) {
          case 0:
            break;
          case 1:
            DataBuffer.erase(DataBuffer.begin());
            break;
          case 2:
            DataBuffer.clear();
            BufferNow = 0;
            break;
        }
      }

      break;
    case 2:
      if (BufferNow >= Buffersize) {
        OUTPUT = calcMean(DataBuffer);

        switch (BufferType) {
          case 0:
            break;
          case 1:
            DataBuffer.erase(DataBuffer.begin());
            break;
          case 2:
            DataBuffer.clear();
            BufferNow = 0;
            break;
        }
      }

      break;

  }

}
void Hormone::gland::Data::setBufferSize(int _Buffersize) {
  Buffersize = _Buffersize;
  BufferNow = 0;
  DataBuffer.clear();
}

unsigned int Hormone::gland::Data::getBufferNow() {
  return BufferNow;
}

void Hormone::gland::Data::setInputType(int Type = HM_Normal, int RecordType = HM_Normal) {
  DataType = Type;
  BufferType = RecordType;
  BufferNow = 0;
  DataBuffer.clear();
}
DATATYPE Hormone::gland::Data::calcSD(std::vector<DATATYPE> inputVector) {
  double Xbar = calcMean(inputVector);
  double sum = 0;
  for (unsigned int i = 0; i < inputVector.size(); i++) {
    sum += pow(inputVector[i] - Xbar, 2);
  }
  sum /= inputVector.size();
  return (DATATYPE) sum;
}
DATATYPE Hormone::gland::Data::calcMean(std::vector<DATATYPE> inputVector) {
  DATATYPE sum = 0;
  for (unsigned int i = 0; i < inputVector.size(); i++) {
    sum += inputVector[i];
  }
  sum /= inputVector.size();
  return sum;
}
void Hormone::gland::Data::clearBuffer() {
  DataBuffer.clear();
}

//class gland///////////////////////////////////////////////////
Hormone::gland::gland() {
  SI = 0;
  CI = 0;
  HOR = 0;
  ALPHA = 1.0;
}
void Hormone::gland::setSI(DATATYPE input) {
  SI = input;
}
void Hormone::gland::setCI(float feedback) {
  if (feedback > 0 && feedback < 1) {
    CI = feedback;
  }

}

DATATYPE Hormone::gland::getCI() {
  return CI;
}

DATATYPE Hormone::gland::getHOR() {
  return HOR;
}

void Hormone::gland::setALPHA(float rate) {
  if (rate >= 0 && rate <= 1) {
    ALPHA = rate;
  }
}
void Hormone::gland::LinkToHormone(hormone *Hormone) {
  Hptr = Hormone;
  Hptr->hormoneset.push_back(this);
}
void Hormone::gland::produceHormone() {
  //DATATYPE CI = 0;
  CI = 0;
  int sum = 0;

  //std::cout << "feeed.size " << feedset.size() <<"\n";

  for (int i = 0; i < feedset.size(); i++) {
    //if (i==0) CI=0; //reset hormone in CI
    //  std::cout << "feedset.feedbackmode =" << feedset[i].feedbackMode << "\n";

    switch (feedset[i].feedbackMode) {
      case 1: //inhibit
        if (feedset[i].Feedback->hormoneValue >= feedset[i].parameter) {
          CI += 0.0 * feedset[i].Weight;
        } else {
          CI += 1.0 * feedset[i].Weight;
        }
        break;
      case 2: //stimulate
        if (feedset[i].Feedback->hormoneValue < feedset[i].parameter) {
          CI += 0.0 * feedset[i].Weight;
        } else {
          CI += 1.0 * feedset[i].Weight;
        }
        break;
      case 3: //positive feeedback
        if (feedset[i].Feedback->hormoneValue != 0) {
          //CI += (1.0 / (feedset[i].Feedback->hormoneValue))* feedset[i].Weight;
          CI += (1.0 / (feedset[i].Feedback->hormoneValue)) * feedset[i].Weight;
        }
        break;
      case 4: //negative feedback
        //CI += (1.0 / (1.0 - feedset[i].Feedback->hormoneValue)) * feedset[i].Weight;
        CI += ((1.0 - feedset[i].Feedback->hormoneValue)) * feedset[i].Weight;
        break;

    }

    sum += feedset[i].Weight;
    //error check config wegiht hormone feedback
    if (sum > 1) {
      //error
      std::cout << "sum =" << sum << " error more then 1\n";

//      CI /=sum;

    } else if (sum < 0) {
      //error
      std::cout << "sum error less then 0\n";
    }

  }

  //std::cout << "CI = " << CI << "\n";

  if (feedset.size() != 0) {
    //std::cout << "CI = " << CI << "\n";
    HOR = (ALPHA * data.OUTPUT * CI) / (sqrt(1 + (pow(data.OUTPUT, 2))));
    //std::cout << "HOR = " << HOR <<"\n";
    //HOR = (ALPHA*1.0) / (1.0 + exp(-(data.OUTPUT)))*(CI);
    //HOR = (ALPHA/CI) * (exp(-(data.OUTPUT)) / pow((1.0 + exp(-(data.OUTPUT))),2));
  } else {

    HOR = (ALPHA * data.OUTPUT) / (sqrt(1 + (pow(data.OUTPUT, 2))));

    // HOR = (ALPHA*1.0) / (1.0 + exp(-(data.OUTPUT)));

    //HOR = (ALPHA) * (exp(-(data.OUTPUT)) / pow((1.0 + exp(-(data.OUTPUT))),2));
  }

  Hptr->G += HOR;
  //CI=0;
  //std::cout << "data.out = " << data.OUTPUT << "\n";
  //std::cout << "HOR = " << HOR <<"\n";
}
void Hormone::gland::setOutputRate(float output) {
  HOR = output;
}
void Hormone::gland::setFeedBack(hormone* feedhormone, int Mode, DATATYPE parameter, DATATYPE weight) {
  struct feedbackSet temp = { feedhormone, Mode, parameter, weight };
  feedset.push_back(temp);

}
//HORMONE FUNCTION//////////////////////////////////////////////////////
Hormone::hormone::hormone() {
  hormoneValue = 0;
}
void Hormone::hormone::AddHormoneValue(DATATYPE value) {
  hormoneValue += value;
}

void Hormone::hormone::setBETA(DATATYPE value) {
  if (value > 1) {
    BETA = 1;
  } else {
    if (value >= 0) {
      BETA = value;
    } else {
      BETA = 0;
    }
  }
}
void Hormone::hormone::Active() {
  hormoneValue += (G + R - (BETA * hormoneValue));

  // std::cout << "G = " << G <<"\n";
  /*
   std::cout << "R = " << R <<"\n";
   std::cout << "G = " << G <<"\n";
   std::cout << "Metabolism = " << (BETA * hormoneValue) <<"\n";
   std::cout << "hormone_value = " << hormoneValue << "  \n";
   */
  if (hormoneValue < 0) {
    hormoneValue = 0.0;
    //  std::cout <<"lowwer\n";
  } else if (hormoneValue > 1) {
    hormoneValue = 1.0;
    //  std::cout <<"higer\n";
  }
  G = 0;
  R = 0;
}
//RECEPTOR FUNCTION/////////////////////////////////////////////////////
Hormone::receptor::receptor() {
  Reaction = 0;
}
void Hormone::receptor::AddHormoneReceptor(hormone* in, DATATYPE rate, DATATYPE Weight) {
  struct HormoneSet temp = { in, rate, Weight };

  in->vectoresep.push_back(this);
  HormoneIn.push_back(temp);
}

DATATYPE Hormone::receptor::reduceHormone() {
  DATATYPE sum = 0, WeightSum = 0, dR = 0;
  for (int i = 0; i < HormoneIn.size(); i++) {
    dR = HormoneIn[i].IN->hormoneValue * HormoneIn[i].Rate;
    HormoneIn[i].IN->R -= dR;

    sum += dR * HormoneIn[i].Weight;
    WeightSum += HormoneIn[i].Weight;

  }
  if (sum > 0) {
    sum /= WeightSum;
    return sum;
  }
  return 0;

}
//ORTHER////////////////////////////////////////////////////////////////

