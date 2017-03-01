#ifndef CONTROLLER_H
#define CONTROLLER_H

typedef struct Controller {
	float ts;
	float output;
	float feedback;
	float error;
	float target;
}Controller_s;


#endif