/*
 * kalman_filter.cpp
 *
 *  Created on: 6 May 2016
 *      Author: Roberto Santos Inoue
 */

#include "kalman_filter.h"

namespace ME {

	KalmanFilter::KalmanFilter() {
		x = x.Zero();
		xp = xp.Zero();
		F = F.Zero();
		G = G.Zero();
		H = H.Zero();
		P = P.Identity();
		Pp = P.Identity();
		Q = Q.Identity();
		Qd = Qd.Identity();
		R = R.Identity();
		S = S.Zero();
		K = K.Zero();
		opt = 1;
	}
	VectorState KalmanFilter::getState(void) {
		return x;
	}
	MatrixP KalmanFilter::getMatrixP(void)  {
		return P;
	}
	MatrixQ KalmanFilter::getMatrixQ(void)  {
		return Q;
	}
	MatrixR KalmanFilter::getMatrixR(void)  {
		return R;
	}
	void KalmanFilter::setKalmanType(short int optValue) {
		opt = optValue;
	}
	void KalmanFilter::setZeroState(void) {
		x = x.Zero();
	}
	void KalmanFilter::setObservationError(VectorObser zErrorValue) {
		zError = zErrorValue;
	}
	void KalmanFilter::setMatrixF(MatrixF FValue){
		F = FValue;
	}
	void KalmanFilter::setMatrixG(MatrixG GValue){
		G = GValue;
	}
	void KalmanFilter::setMatrixH(MatrixH HValue){
		H = HValue;
	}
	void KalmanFilter::setMatrixP(MatrixP PValue) {
		P = PValue;
	}

    void KalmanFilter::setMatrixQ(MatrixQ QValue) {
    	Q = QValue;
    }
    void KalmanFilter::setMatrixR(MatrixR RValue) {
    	R = RValue;
    }
    void KalmanFilter::prediction(void) {
		// cout << "x: \n\r" << x << endl;
    	// cout << "F = [ \n\r" << F << "];" << endl;
    	xp = F*x;
    	// cout << "G = [ \n\r" << G << "];" << endl;
        // cout << "Q = [ \n\r" << Q << "];" << endl;
        // cout << "R = [ \n\r" << R << "];" << endl;
        Qd = G*Q*G.transpose();
        // cout << "Qd = [ \n\r" << Qd << "];" << endl;
        Pp = F*P*F.transpose() + Qd;
        // cout << "P = [ \n\r" << P << "];" << endl;
        // cout << "Pp = [ \n\r" << Pp << "];" << endl;
    }
    void KalmanFilter::update(void) {
    	// cout << "H = [ \n\r" << H << "];" << endl;
    	// cout << "R = [ \n\r" << R << "];" << endl;
        S = H*Pp*H.transpose() + R;
        // cout << "S = [ \n\r" << S << "];" << endl;
        K = Pp*H.transpose()*S.inverse();
        // cout << "K = [ \n\r" << K << "];" << endl;
        xp = x;
        // cout << "xp = [ \n\r" << xp << "];" << endl;
        if (opt == 0) {
        	x = xp + K*(z-H*x);
        	// cout << "z = [ \n\r" << z << "];" << endl;
        }
        if (opt == 1) {
        	x = xp + K*zError;
        	// cout << "zError = [ \n\r" << zError << "];" << endl;
        }
        P = (P.Identity() - K*H)*Pp;
        // cout << "P = [ \n\r" << P << "];" << endl;
    }

} // namespace ME

