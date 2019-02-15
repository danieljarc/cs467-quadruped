CFLAGS= -Wall -lm -std=c99

default: client

btFind: btFind.c 
	gcc -o btFind.c  $(CFLAGS) -lbluetooth
client: client.c 
	gcc -o client client.c $(CFLAGS) -lbluetooth

all: client btFind

clean:
	rm -f *.o client btFind 
