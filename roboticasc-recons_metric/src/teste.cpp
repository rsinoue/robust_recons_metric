/*
 * teste.cpp
 *
 *  Created on: 6 May 2016
 *      Author: Roberto Santos Inoue
 */

#include "teste/teste.h"


namespace ME {

	void Print::hello(void){
			cout << "Hello" << endl;
	}

	void Rectangle::set_values (int x, int y) {
	  width = x;
	  height = y;
	}

} // namespace ME
