#ifdef WIN32
#include <windows.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include "hidapi.h"
#include <string.h>

#define MAX_STR 255
#define PTS 500

int main(int argc, char* argv[])
{

	int res;
	unsigned char buf[65];
	wchar_t wstr[MAX_STR];
	hid_device *handle;
	int i;
	int j;

	// Initialize the hidapi library
	res = hid_init();

	// Open the device using the VID, PID,
	// and optionally the Serial number.
	handle = hid_open(0x4d8, 0x3f, NULL);

	// Read the Manufacturer String
	res = hid_get_manufacturer_string(handle, wstr, MAX_STR);
	wprintf(L"Manufacturer String: %s\n", wstr);

	// Read the Product String
	res = hid_get_product_string(handle, wstr, MAX_STR);
	wprintf(L"Product String: %s\n", wstr);

	// Read the Serial Number String
	res = hid_get_serial_number_string(handle, wstr, MAX_STR);
	wprintf(L"Serial Number String: (%d) %s\n", wstr[0], wstr);

	// Read Indexed String 1
	res = hid_get_indexed_string(handle, 1, wstr, MAX_STR);
	wprintf(L"Indexed String 1: %s\n", wstr);

	
	////////////// SEND DATA /////////////////////////////////
    //Get message and row 
	printf("Write a message\n");
	char message[MAX_STR];
	scanf("%[^\n]s",message);
	int len = strlen(message);
	int row;
	printf("Pick a row\n");
	scanf("%d", &row);
	
	//clear buffer
	for (i = 0; i<65; i++)
		buf[i] = 0;
		
	// Toggle LED (cmd 0x80) and set send	
	buf[0] = 0x0;
	buf[1] = 0x80;
    buf[2] = row;

	//Write message to buffer
	for (i = 0; i<len; i++){
		buf[i+3] = (int) message[i];
	}
	res = hid_write(handle, buf, 65);
	
	//Print out Sent buffer
	for (i = 0; i < len ; i++)
		printf("buf[%d]: %d\n", i, buf[i]);
		

	////////////// READ DATA //////////////////////////////////////
	
	//short x[PTS];
	//short y[PTS];
	short z[PTS];
	short zMAF[PTS];
	short zFIR[PTS];
	int num = 0;
	
	printf("\nI am collecting data\n");
	
	while(num < PTS){
		// Request state (cmd 0x81). The first byte is the report number (0x0).
		buf[0] = 0x0;
		buf[1] = 0x81;
		res = hid_write(handle, buf, 65);
		//Read requested state
		res = hid_read(handle, buf, 65);

		// Print out the returned buffer.
		// for (i = 0; i < 4 ; i++)
			// printf("buf[%d]: %d\n", i, buf[i]);
		
		if ((buf[2] != 0) && (buf[3] != 0)){ 
			//x[num] = (short)((buf[2] << 8) | (buf[3]));
			//y[num] = (short)((buf[4] << 8) | (buf[5]));
			z[num] = (short)((buf[2] << 8) | (buf[3]));
			if (num % 25 ==0)(printf("time: %d\n", num/25)); //Optional timer
			num++;
			for (i = 0; i<65; i++)
			buf[i] = 0;
		}
		}
	//create MAF filtered data
	int numel = 5; //num of elements for filter
	short elsum=0; 
	for(i = numel; i<PTS-numel; i++){
		elsum = 0;
		for(j=numel-1;j>=0;j--){
			elsum += z[i-j]/numel;
			//printf("%d\n",elsum);
		}
		zMAF[i] = elsum;
		//printf("%d\n\n", zMAF[i]);
	}
	
	//create FIR filtered data
	double FIR[13] = {.0086, .0714, .0423, .0797, 0.1199, .1509, .1625, .1509, .1199, .0797, .0423, .0174, .0086};
	numel = 12;
	
	for(i = numel; i<PTS-numel; i++){
		elsum = 0;
		for(j=numel-1;j>=0;j--){
			elsum += (double)FIR[11-j]*z[i-j];
			//printf("%d\n",elsum);
		}
		zFIR[i] = elsum;
		//printf("%d\n\n", zMAF[i]);
	}
	
	
	
	printf("Done Collecting\n");
	FILE *acc;
	
	acc = fopen("accs.txt", "w");
	int line;
	for(line = 0; line<PTS; line++)
		fprintf(acc, "%d, %d, %d\n", z[line], zMAF[line], zFIR[line]);
	fclose(acc);

	// Finalize the hidapi library
	res = hid_exit();

	//clear buffer
	for (i = 0; i<65; i++)
		buf[i] = 0;

	return 0;
}