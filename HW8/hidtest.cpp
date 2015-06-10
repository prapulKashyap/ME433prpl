
#ifdef WIN32
#include <windows.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include "hidapi.h"

#define MAX_STR 255

int main(int argc, char* argv[])
{
	int res;
	//row=23; expand for hard coded version
	//str_count=0; expand for hard coded version 
	//char message[]="hello!"; expand for hard coded version
	unsigned char buf[65];
	wchar_t wstr[MAX_STR];
	hid_device *handle;
	int i; int iter; double Z[30]; short z[30]; FILE *readings;

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

	// Toggle LED (cmd 0x80). The first byte is the report number (0x0).
	buf[0] = 0x0;
	buf[1] = 0x80;
	
	// user input message into buffer
	
	wprintf(L"Enter message \n"); // using same wprintf format from above 
	
	char c;
	i=2;
	while ((buf[i++]=getchar())!= '\n' && i < 27)
	;
	while (i<28){
	buf[i-1]=' ';
	i++;
	}
	wprintf(L"%s", buf);
	res = hid_write(handle, buf, 65); 







	for (iter = 0; iter < 20; iter++)
	{


	// Request state (cmd 0x81). The first byte is the report number (0x0)

	buf[0] = 0x0;
	buf[1] = 0x81;
	res = hid_write(handle, buf, 65);

	// Read requested state
	res = hid_read(handle, buf, 65);

/*
	// Print out the returned buffer.
	for (i = 0; i < 4; i++)
		printf("buf[%d]: %d\n", i, buf[i]);


hard coded for testing 
*/

	//Convert char (1 byte) to shorts (2 bytes) --not using buf because it is a char

		z[iter] = (buf[4] << 8) | (buf[3] & 0xff);

		//Print the accelerations as FLOATS 

		/* printf("z accel is: %.2f\n", ((float)z[iter])/16000); */
		Z[iter] = (float)z[iter]/16000; 
	}
		
// saving data on a text file - this can be used for plotting in matlab

	readings = fopen("accels.txt", "w");
	 for (i=0; i<20; i++)
	  fprintf(readings,"%.3f\n",Z[i]);
  	fclose(readings); 



	// Finalize the hidapi library
	res = hid_exit();

	return 0;
}