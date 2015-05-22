#ifndef DATAVECTOR_H
#define DATAVECTOR_H

#include <stdio.h>
#include <string>
#include <stdlib.h>
#include <iostream>
#include <cstdio>
#include <numeric>
#include <math.h>
#include <complex>
#include <valarray>
#include <vector>

using namespace std;

typedef std::complex<double> Complex;
typedef std::valarray<Complex> CArray;

class DataVector: public vector<double> {
private:
	double sum = 0;
	double mean;
	double variance = 0;
	double min = 1E5, max = -1E5;
	double dt = 0;
	double data_old;
	double sample_frequency = 530;
	double delta = 1 / sample_frequency;
	double meandt = 0;
	double PI = 3.141592653589793238460;

public:
	void add(double data) {
		this->push_back(data);
		sum += data;

		if (data < min)
			min = data;
		if (data > max)
			max = data;

		dt = data - data_old / delta;
		data_old = data;

	}

	double getMin() {
		return min;
	}

	double getMax() {
		return max;
	}

	double getMean() {
		mean = sum / size();
		return mean;
	}

	double getVariance() {
		for (uint i = 0; i < this->size(); ++i) {
			variance += (mean - this->at(i)) * (mean - this->at(i));
		}
		variance /= this->size();
		return variance;
	}

	double getMobility() {
		double mobility = 0;
		DataVector m = this->vector_dt(delta);
		mobility = sqrt(abs(m.getVariance() / this->getVariance()));
		return mobility;
	}

	double getComplexity() {
		double cmplexity;
		DataVector m = this->vector_dt(delta);
		cmplexity = m.getMobility() / this->getMobility();
		return cmplexity;
	}

	double getMeanDt() {
		double meandt;
		for (uint i = 1; i < this->size(); ++i) {
			meandt += (this->at(i) - this->at(i - 1)) / delta;
		}

		meandt /= this->size();
		return meandt;
	}

	double getSkewness() {
		double skewness_sum;
		double skewness;

		for (int i = 1; i < this->size(); i++) {
			skewness_sum += pow((this->at(i) - mean), 3);
		}

		skewness = skewness_sum
				/ ((this->size() - 1) * pow((sqrt(this->getVariance())), 3));
		return skewness;
	}

	double crossDetector(double threshold) {
		int crossings;

		for (int i = 1; i < this->size(); i++) {
			if (((this->at(i) > threshold) && (this->at(i - 1) < threshold))
					|| ((this->at(i) < threshold)
							&& (this->at(i - 1) > threshold))) {
				crossings++;
			}
		}

		return crossings;
	}

	CArray fft() {
		std::vector<Complex> complexVector(this->begin(), this->end());
		CArray cmplxVal(complexVector.data(), complexVector.size());

		fft(cmplxVal);

		return cmplxVal;
	}

	void fft(CArray& x) {
		const size_t N = x.size();
		if (N <= 1)
			return;

		// divide
		CArray even = x[std::slice(0, N / 2, 2)];
		CArray odd = x[std::slice(1, N / 2, 2)];

		// conquer
		fft(even);
		fft(odd);

		// combine
		for (size_t k = 0; k < N / 2; ++k) {
			Complex t = std::polar(1.0d, (-2 * PI * k / N)) * odd[k];
			x[k] = even[k] + t;
			x[k + N / 2] = even[k] - t;
		}
	}

	DataVector freq_analysis(CArray& x, int a, int b) {
		int L = x.size();
		DataVector frqn;

		for (int i = L / 2; i < L; i++) {
			frqn.insert(frqn.end(), 2 * abs(x[i].real()));
		}
		return frqn;

	}

	DataVector vector_dt(double dt) {
		DataVector dvdt;
		double derivative;
		for (int i = 1; this->size(); i++) {
			derivative = (this->at(i) - this->at(i - 1)) / dt;
			dvdt.insert(dvdt.end(), derivative);
		}

		return dvdt;

	}

};

#endif
