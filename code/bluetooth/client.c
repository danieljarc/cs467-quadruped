#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <stdio.h>
#include <string.h>

#define BUFFER_SIZE 64

int getLine( char* prompt ,char* rtnBuffer, int bufferSize){
	int i;
	char tmpBuffer[BUFFER_SIZE] = {'\0'};
	printf("%s", prompt);
	
	fgets(tmpBuffer, 500 , stdin);
	
	for(i=0; i < 500; i++){
		if(tmpBuffer[i]!= '\0'){
			if(tmpBuffer[i]== '\n'){
				rtnBuffer[i] = '\0';
			}else{
				rtnBuffer[i]= tmpBuffer[i];
			}
		}else
			return i; // Returns number of characters
	}
	return 0;
}
/*Heavily inspired by http://people.csail.mit.edu/albert/bluez-intro/x502.html#rfcomm-server.c*/
int main(int argc, char **argv)
{

    struct sockaddr_rc addr = { 0 };
    int sock, status, strSize;
    char dest[18] = "98:D3:61:FD:33:54";
	char buffer[BUFFER_SIZE];
    // allocate a socket
    sock = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
    // set the connection parameters (who to connect to)
    addr.rc_family = AF_BLUETOOTH;
    addr.rc_channel = (uint8_t) 1;
    str2ba( dest, &addr.rc_bdaddr );

    // connect to server
    status = connect(sock, (struct sockaddr *)&addr, sizeof(addr));
	if(status <= 0){ //If socket connected continue else abort

   		status = write(sock, "Connected.", 10);
		while(true){
			memset(buffer, 0 ,sizeof(buffer));
			strSize = getLine("Input a message to send: ", buffer, BUFFER_SIZE);
		
			if(strcmp(buffer, "exit\n")==0)
				break;

//			printf("%d\n", strSize);
//			printf("(%s)", buffer);
//    	   	status = send(sock, buffer, sizeof(buffer), 0);
    	   	status = send(sock, buffer, strlen(buffer), 0); // Sending strlen sends only chars not null
		}
	}else printf("Error: Connection error. status %d\n", status);	
	printf("Exit bluetooth client\n");
    close(sock);
    return 0;
}

