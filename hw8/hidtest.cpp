#ifdef WIN32
#include <windows.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include "hidapi.h"
#include <string.h>

#define MAX_STR 255
#define PTS 400

int main(int argc, char* argv[])
{

	int res;
	unsigned char buf[65];
	wchar_t wstr[MAX_STR];
	hid_device *handle;
	int i;

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
	
	short x[PTS];
	short y[PTS];
	short z[PTS];
	int num = 0;
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
		
		if ((buf[2] != 0) && (buf[4] != 0) && (buf[6] != 0)){ 
			x[num] = (short)((buf[2] << 8) | (buf[3]));
			y[num] = (short)((buf[4] << 8) | (buf[5]));
			z[num] = (short)((buf[6] << 8) | (buf[7]));
			num++;
			for (i = 0; i<65; i++)
			buf[i] = 0;
		}
		}
	
	printf("Done Collecting\n");
	FILE *acc;
	
	acc = fopen("accs.txt", "w");
	int line;
	for(line = 0; line<PTS; line++)
		fprintf(acc, "%d %d %d\n", x[line], y[line], z[line]);
	fclose(acc);

	// Finalize the hidapi library
	res = hid_exit();

	//clear buffer
	for (i = 0; i<65; i++)
		buf[i] = 0;

	return 0;
}