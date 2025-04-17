/*
 * kalman_filter.h
 *
 *  Created on: 6 May 2016
 *      Author: Roberto Santos Inoue
 */

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include "me_definitions.h"
#include "sensors.h"

using namespace ME::Types;

namespace ME{

    class KalmanFilter {
    	VectorState x;
    	VectorState xp;
    	VectorObser z;
    	VectorObser zError;
		MatrixF F;
		MatrixG G;
		MatrixH H;
		MatrixP P;
		MatrixP Pp;
		MatrixQ Q;
		MatrixQd Qd;
		MatrixR R;
		MatrixR S;
		MatrixK K;
		short int opt;
      public:
		KalmanFilter();
		VectorState getState(void);
		MatrixP getMatrixP(void);
		MatrixQ getMatrixQ(void);
		MatrixR getMatrixR(void);
		void setKalmanType(short int optValue);
		void setZeroState(void);
		void setObservationError(VectorObser zErrorValue);
		void setMatrixF(MatrixF FValue);
		void setMatrixG(MatrixG GValue);
		void setMatrixH(MatrixH HValue);
		void setMatrixP(MatrixP PValue);
		void setMatrixQ(MatrixQ QValue);
		void setMatrixR(MatrixR RValue);
	    void prediction(void);
	    void update(void);
    };

} // namespace ME

#endif // KALMAN_FILTER_H
