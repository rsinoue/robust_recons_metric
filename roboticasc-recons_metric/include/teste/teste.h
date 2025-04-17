#ifndef TESTE_H
#define TESTE_H

#include <iostream>
#include <fstream>

using namespace std;

namespace ME {
	class Print {
	public:
		void hello(void);
	};

	class Rectangle {
	    int width, height;
	  public:
	    void set_values (int,int);
	    int area() {return width*height;}
	};
} // namespace ME

#endif // TESTE_H
