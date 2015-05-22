#ifndef FEATUREVECTOR_H
#define FEATUREVECTOR_H

using namespace std;

class FeatureVector : public vector<double> {
private:
	double sum = 0;
	double mean;
	double min = 1E5, max = -1E5;

public:
	void add(double data) {
		this->push_back(data);
		sum += data;

		if(data < min)
			min = data;
		if(data > max)
			max = data;
	}

	FeatureVector featureNormalization(std::vector<double> minimums, std::vector<double> maximums)
	{
	  FeatureVector normalized;
	  double temp;
	  for (int i =0; i< this->size(); i++)
	  {
		temp=this->at(i)-minimums[i]/(maximums[i]-minimums[i]);
	    normalized.insert(normalized.end(),temp);
	  }

	  return normalized;
	}


};

#endif
