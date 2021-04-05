#include <stdio.h>
#include "cfifo.h"

int main(void)
{
	struct fifo_t *myfifo = fifo_create();
	if(myfifo == NULL){
		printf("FIFO create err!\n");
		return 0;
	}
	int i;
	for(i = 0; i < 200; i++){
		myfifo->push(myfifo, i);
		if(myfifo->is_err(myfifo)){
			break;
		}
	}
	myfifo->print(myfifo);
	myfifo->clear(myfifo);
	myfifo->print(myfifo);
	for(i = 0; i < 200; i++){
		printf("<%d>: %d\n", i, myfifo->pop(myfifo));
		if(myfifo->is_err(myfifo)){
			break;
		}
	}
	myfifo->print(myfifo);
	return 0;
}
