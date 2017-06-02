/*
 * hormone.h
 *
 *  Created on: Oct 1, 2015
 *      Author: soulx
 */

#ifndef HORMONE_H_
#define HORMONE_H_

#include <vector>
#include <algorithm>
#include <cmath>
#include <thread>//Need parallal programming
#include <ctime>


//#define DATATYPE long double
#define DATATYPE  double
namespace Hormone
{
  //PRECLASS
  class gland;
  class hormone;
  class Data;
  class receptor;

  //HORMONE///////////////////////////////////////////////////////////////
  class hormone
  {
  public:
    hormone();
    void AddHormoneValue(DATATYPE);
    DATATYPE DelHormoneValue(DATATYPE);
    DATATYPE hormoneValue;
    DATATYPE G=0;
    DATATYPE R=0;
    DATATYPE BETA=0;
    void Active();
    void setBETA(DATATYPE);
    std::vector<gland*>hormoneset;
  private:
    //std::vector<gland*> vecToGrand;
    std::vector<receptor*> vectoresep;
    friend class gland;
    friend class receptor;
  };
  //GLAND/////////////////////////////////////////////////////////////////

  class gland
  {


  public:

#define HM_AddWhenLess 0
#define HM_AddWhenMore 1


    gland();
    class Data
    {
    public:

#define HM_Normal 0
#define HM_RECORD_FIFO 1
#define HM_RECORD_Shift 2

#define HM_SD 1
#define HM_Mean 2

      Data();
      DATATYPE OUTPUT;
      void Add(DATATYPE);
      void setBufferSize(int);
      unsigned int getBufferNow();
      void setInputType(int Type, int RecordType);
      DATATYPE calcSD(std::vector<DATATYPE>);
      DATATYPE calcMean(std::vector<DATATYPE>);
      void clearBuffer();

    private:
      std::vector<DATATYPE> DataBuffer;
      int DataType;
      int BufferType;
      unsigned int BufferNow;
      unsigned int Buffersize;

    } data;
    void setSI(DATATYPE);
    void setCI(float);
    void setALPHA(float);
    void setOutputRate(float);//for debug
    void LinkToHormone(hormone*);
    void produceHormone();
    //setFeedBack( hormone , Mode , Parameter);
    void setFeedBack(hormone* ,int,DATATYPE=0,DATATYPE=1);

    DATATYPE getCI();
    DATATYPE getHOR();


  private:
    DATATYPE SI;//Input from data source
    DATATYPE CI;//Feedback from hormone
    DATATYPE HOR;//Hormone rate Output
    float ALPHA;// stimulate rate
    hormone *Hptr;
    struct  feedbackSet
    {
      hormone* Feedback;
      int feedbackMode;
        //mode
        //1 = inhibit
        //2 = stimulate
        //3 = positive feedback
        //4 = negative feedback
      DATATYPE parameter;
      DATATYPE Weight;
    };
    std::vector< struct feedbackSet > feedset;
    friend class hormone;
  };

  //RECEPTOR//////////////////////////////////////////////////////////////
  class receptor
  {
  public:

    receptor();
    void AddHormoneReceptor(hormone*,DATATYPE,DATATYPE);
    DATATYPE reduceHormone();
    DATATYPE OUTPUT;

  private:
    float Reaction;//need Name
    struct  HormoneSet
    {
      hormone* IN;
      DATATYPE Rate;
      DATATYPE Weight;
    };
    std::vector<HormoneSet> HormoneIn;
    friend class hormone;
  };
  //KIDNEY////////////////////////////////////////////////////////////////
  class kidney
  {
  public:
    kidney();
  private:

  };
  //timer
  //OTHER//////////////////////////////////////////////////////////////////
  template <class T> inline T RangeCast(T INPUT, T IN_MIN, T IN_MAX, T OUT_MIN, T OUT_MAX)
  {
    return OUT_MIN + ((INPUT - IN_MIN) / (IN_MAX - IN_MIN)*(OUT_MAX - OUT_MIN));
  }
  class timer
  {
  public:
    timer();
    void setcycle(int);
  };
}



#endif /* HORMONE_H_ */
