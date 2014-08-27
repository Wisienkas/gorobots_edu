
#include <iostream>
#include <stdlib.h>
#include "Eigen/Dense"
#include "Eigen/Sparse"
#include "Eigen/Eigen"
#include <algorithm>

#include <stack>
#include <map>
#include <utility>

using namespace Eigen;
using namespace std;

#include "PI2Wholesteps.h"

class kinecontroller
{
public:
	inline Eigen::MatrixXd kinemodel(float y[], float u[]);

};

template<typename _Matrix_Type_>
MatrixXd pseudoInverse(const _Matrix_Type_ &a, double
		epsilon = std::numeric_limits<double>::epsilon())
{
	MatrixXd result(3,4); // change matrix dimension here if kinemodel changes
	//if(a.rows() < a.cols())
	//return result.setZero();
	Eigen::JacobiSVD< _Matrix_Type_ > svd = a.jacobiSvd(Eigen::ComputeThinU |
			Eigen::ComputeThinV);
	double tolerance = epsilon * std::max(a.cols(), a.rows()) *
			svd.singularValues().array().abs().maxCoeff();

	result = svd.matrixV() * ( (svd.singularValues().array().abs() >
	tolerance).select(svd.singularValues().array().inverse(), 0)
	).matrix().asDiagonal() * svd.matrixU().adjoint();
	// cout<<"\n pseudo of S is \n"<<result;
	return result;
}


Eigen::MatrixXd kinecontroller::kinemodel(float y[], float u_boise[])
{
	PI2Wholesteps p1;
	//cout<<"in pi2 y is "<<y[2]<<endl;
	//cout<<"in pi2"<<endl;
	float alpha1 = p1.alpha[0];
	float alpha2 = p1.alpha[1];
	float alpha3 = p1.alpha[2];
	float alpha4 = p1.alpha[3];
	float L0 = p1.l0;
	float Lg = p1.l1;
	float L = p1.L;

	// kinematic model without joint movements
	// fix the angular velocity of each joint to 0
	MatrixXd A(4,3);

	A(0,0)=cos(alpha1+y[2]);
	A(0,1)=sin(alpha1+y[2]);
	A(0,2)=(L0 + Lg)*sin(alpha1);
	//cout<<A.row(0);


	A(1,0)=cos(alpha2+y[2]+y[3]);
	A(1,1)=sin(alpha2+y[2]+y[3]);
	A(1,2)=(L0 + L)*sin(alpha2+y[3])+Lg*sin(alpha2);

	A(2,0)=cos(alpha3+y[2]+y[3]+y[4]);
	A(2,1)=sin(alpha3+y[2]+y[3]+y[4]);
	A(2,2)=(L0 + L)*sin(alpha3+y[3]+y[4])+L*sin(alpha3+y[4])+Lg*sin(alpha3);

	A(3,0)=cos(alpha4+y[2]+y[3]+y[4]+y[5]);
	A(3,1)=sin(alpha4+y[2]+y[3]+y[4]+y[5]);
	A(3,2)=(L0 + L)*sin(alpha4+y[3]+y[4]+y[5])+L*sin(alpha4+y[4]+y[5])+L*sin(alpha4+y[5])+Lg*sin(alpha4);

	//cout<<" A done"<<endl;

	MatrixXd B(4,4);
	B(0,0)= -(p1.r)*sin(alpha1);
	B(0,1)=0;
	B(0,2)=0;
	B(0,3)=0;

	B(1,0)=0;
	B(1,1)= -(p1.r)*sin(alpha2);
	B(1,2)=0;
	B(1,3)=0;

	B(2,0)=0;
	B(2,1)=0;
	B(2,2)= -(p1.r)*sin(alpha3);
	B(2,3)=0;

	B(3,0)=0;
	B(3,1)=0;
	B(3,2)=0;
	B(3,3)= -(p1.r)*sin(alpha4);

	//cout << "The solution A is:\n" << A;
	//cout << "\n The solution B is:\n" << B;
	//cout << "\n The solution s is:\n" << C;

	MatrixXd U(4,1);
	U(0,0)=u_boise[0];
	U(1,0)=u_boise[1];
	U(2,0)=u_boise[2];
	U(3,0)=u_boise[3];
	//cout<<"\n U: "<<U;
	MatrixXd B1=B.transpose();
	MatrixXd B2=B1.inverse();
	MatrixXd B3=(A.transpose())*B2;
	//float a=C.determinant();
	MatrixXd S=B3.transpose();
	//cout<<"\n S (B/A) matrix till now:\n"<<S;
	//MatrixXd D=A.transpose().colPivHouseholderQr().solve(B.transpose());
	//cout<<"\n Alternate S :\n"<<D;
	/*MatrixXd result1= pseudoInverse(A, 0.01 );
	MatrixXd result2=result1*B;
	MatrixXd dw=result2*U;*/
	MatrixXd result= pseudoInverse(S);
	//MatrixXd result= pseudoInverse(A)*B;

	/*result(0,0)= -0.0549;
	result(1,0)= 0.0120;
	result(2,0)= 0.0830;

	result(0,1)= -0.0409;
	result(1,1)= 0.0468;
	result(2,1)= 0.0830;

	result(0,2)= 0.0061;
	result(1,2)= -0.0328;
	result(2,2)= -0.0830;

	result(0,3)= 0.0201;
	result(1,3)= 0.0020;
	result(2,3)= -0.0830;*/

	MatrixXd dw=result*U;
	//cout<<"\n Final DW is :\n"<<dw;
	return dw;
}
