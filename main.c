/* 
 * Copyright (C) 2009 Chris McClelland
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *  
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <string.h>
#include "usbwrap.h"
#include "argtable2.h"
#include "arg_uint.h"
#ifdef WIN32
#include <fcntl.h>
#include <io.h>
#endif

#define VID 0x1443
#define PID 0x0005
#define BUFFER_SIZE 4096

/*
 * Perform a read of the control endpoint. Return zero if the result matched the expected result.
 * Return -1 if there was some problem reading. Return -2 if the correct number of bytes was read,
 * but their values differed from what was expected.
 */
static int controlRead(UsbDeviceHandle *deviceHandle, uint8 bRequest, uint16 wValue, uint16 wIndex,
		const uint8 *expected, uint16 wLength)
{
	char buffer[BUFFER_SIZE];
	int returnCode;
	returnCode = usb_control_msg(
		deviceHandle,
		USB_ENDPOINT_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
		bRequest, wValue, wIndex, buffer, wLength, 5000
	);
	if ( returnCode != wLength ) {
		return -1;
	}
	if ( memcmp(buffer, expected, wLength) ) {
		return -2;
	}
	return 0;
}

/*
 * Perform a bulk write to the specified endpoint. Return 0 if all is well, else -1.
 */
static int bulkWrite(UsbDeviceHandle *deviceHandle, int endpoint, const uint8 *buffer, uint16 wLength) {
	int returnCode;
	returnCode = usb_bulk_write(deviceHandle, USB_ENDPOINT_OUT | endpoint, (char*)buffer, wLength, 5000);
	if ( returnCode != wLength ) {
		return -1;
	}
	return 0;
}

/*
 * Perform a bulk read from the specified endpoint. Return zero if the result matched the expected
 * result. Return -1 if there was some problem reading. Return -2 if the correct number of bytes was
 * read, but their values differed from what was expected.
 */
static int bulkRead(UsbDeviceHandle *deviceHandle, int endpoint, const uint8 *expected, uint16 wLength) {
	char buffer[BUFFER_SIZE];
	int returnCode;
	returnCode = usb_bulk_read(deviceHandle, USB_ENDPOINT_IN | endpoint, buffer, wLength, 5000);
	if ( returnCode != wLength ) {
		return -1;
	}
	if ( memcmp(buffer, expected, wLength) ) {
		return -2;
	}
	return 0;
}

/*
 * Do one or more reads from the specified EPP register, writing the result to the supplied buffer.
 * Return zero on success, negative otherwise.
 */
int readRegister(UsbDeviceHandle *deviceHandle, uint8 reg, int nReads, uint8 *buffer) {
	const uint8 MSG_00[] = {0x05, 0x00, 0x10, 0x00};
	const uint8 MSG_01[] = {0x03, 0x03};
	const uint8 MSG_02[] = {0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	const uint8 MSG_03[] = {0x07, 0x00, 0x03, 0x00, 0x71, 0x7F, 0x12, 0x01};
	const uint8 MSG_04[] = {0x05, 0x00, 0x09, 0x81, 0xED, 0xFE};
	const uint8 MSG_05[] = {0x03, 0x04, 0x00, 0x00};
	const uint8 MSG_06[] = {0x01, 0x00};
	const uint8 MSG_09[] = {0x03, 0x04, 0x85, 0x00};
	const uint8 MSG_11[] = {0x03, 0x04, 0x01, 0x00};
	uint8 command[9];
	uint8 status[6];
	int returnCode;

	command[0] = 0x08;
	command[1] = 0x04;
	command[2] = 0x05; // Read
	command[3] = 0x00;
	command[4] = reg;
	command[5] = nReads & 0xFF;
	command[6] = (nReads >> 8) & 0xFF;
	command[7] = (nReads >> 16) & 0xFF;
	command[8] = (nReads >> 24) & 0xFF;

	// R0: C0 E9 0000 0000 -> 05 00 10 00
	// R0: C0 E6 0000 0000 -> 03 03
	// R0: C0 E7 0000 0000 -> 0D 00 00 00 00 00 00 00
	// R0: C0 E9 0000 0000 -> 05 00 10 00
	if ( controlRead(deviceHandle, 0xE9, 0x0000, 0x0000, MSG_00, 4) ) return -1;
	if ( controlRead(deviceHandle, 0xE6, 0x0000, 0x0000, MSG_01, 2) ) return -2;
	if ( controlRead(deviceHandle, 0xE7, 0x0000, 0x0000, MSG_02, 8) ) return -3;
	if ( controlRead(deviceHandle, 0xE9, 0x0000, 0x0000, MSG_00, 4) ) return -4;

	// W1: 07 00 03 00 71 7F 12 01
	// R1: 05 00 09 81 ED FE
	if ( bulkWrite(deviceHandle, 1, MSG_03, 8) ) return -5;
	if ( bulkRead(deviceHandle, 1, MSG_04, 6) ) return -6;

	// R0: C0 E9 0000 0000 -> 05 00 10 00
	// R0: C0 E6 0000 0000 -> 03 03
	// R0: C0 E7 0000 0000 -> 0D 00 00 00 00 00 00 00
	// R0: C0 E7 0000 0000 -> 0D 00 00 00 00 00 00 00
	if ( controlRead(deviceHandle, 0xE9, 0x0000, 0x0000, MSG_00, 4) ) return -7;
	if ( controlRead(deviceHandle, 0xE6, 0x0000, 0x0000, MSG_01, 2) ) return -8;
	if ( controlRead(deviceHandle, 0xE7, 0x0000, 0x0000, MSG_02, 8) ) return -9;
	if ( controlRead(deviceHandle, 0xE7, 0x0000, 0x0000, MSG_02, 8) ) return -10;

	// W1: 03 04 00 00
	// R1: 01 00
	if ( bulkWrite(deviceHandle, 1, MSG_05, 4) ) return -11;
	if ( bulkRead(deviceHandle, 1, MSG_06, 2) ) return -12;

	// W1: 08 04 05 00 RR NN 00 00 00
	// R1: 01 00
	if ( bulkWrite(deviceHandle, 1, command, 9) ) return -13;
	if ( bulkRead(deviceHandle, 1, MSG_06, 2) ) return -14;

	// R6: 15
	returnCode = usb_bulk_read(deviceHandle, USB_ENDPOINT_IN | 6, (char*)buffer, nReads, 5000);
	if ( returnCode != nReads ) {
		return -15;
	}

	// W1: 03 04 85 00
	// R1: 05 40 NN 00 00 00
	if ( bulkWrite(deviceHandle, 1, MSG_09, 4) ) return -16;
	returnCode = usb_bulk_read(deviceHandle, USB_ENDPOINT_IN | 1, (char*)status, 6, 5000);
	if ( returnCode != 6 ) {
		return -17;
	}
	if ( status[0] != 0x05 || status[1] != 0x40 || memcmp(status+2, command+5, 4) ) {
		return -17;
	}
	
	// W1: 03 04 01 00
	// R1: 01 00
	if ( bulkWrite(deviceHandle, 1, MSG_11, 4) ) return -18;
	if ( bulkRead(deviceHandle, 1, MSG_06, 2) ) return -19;

	return 0;
}

/*
 * Write data from the supplied buffer to the specified EPP register.
 * Return zero on success, negative otherwise.
 */
int writeRegister(UsbDeviceHandle *deviceHandle, uint8 reg, int nWrites, const uint8 *buffer) {
	const uint8 MSG_00[] = {0x05, 0x00, 0x10, 0x00};
	const uint8 MSG_01[] = {0x03, 0x03};
	const uint8 MSG_02[] = {0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	const uint8 MSG_03[] = {0x07, 0x00, 0x03, 0x00, 0x71, 0x7F, 0x12, 0x01};
	const uint8 MSG_04[] = {0x05, 0x00, 0x09, 0x81, 0xED, 0xFE};
	const uint8 MSG_05[] = {0x03, 0x04, 0x00, 0x00};
	const uint8 MSG_06[] = {0x01, 0x00};
	const uint8 MSG_09[] = {0x03, 0x04, 0x84, 0x00};
	const uint8 MSG_11[] = {0x03, 0x04, 0x01, 0x00};
	uint8 command[9];
	uint8 status[6];
	int returnCode;
	command[0] = 0x08;
	command[1] = 0x04;
	command[2] = 0x04; // Write
	command[3] = 0x00;
	command[4] = reg;
	command[5] = nWrites & 0xFF;
	command[6] = (nWrites >> 8) & 0xFF;
	command[7] = (nWrites >> 16) & 0xFF;
	command[8] = (nWrites >> 24) & 0xFF;

	// R0: C0 E9 0000 0000 -> 05 00 10 00
	// R0: C0 E6 0000 0000 -> 03 03
	// R0: C0 E7 0000 0000 -> 0D 00 00 00 00 00 00 00
	// R0: C0 E9 0000 0000 -> 05 00 10 00
	if ( controlRead(deviceHandle, 0xE9, 0x0000, 0x0000, MSG_00, 4) ) return -1;
	if ( controlRead(deviceHandle, 0xE6, 0x0000, 0x0000, MSG_01, 2) ) return -2;
	if ( controlRead(deviceHandle, 0xE7, 0x0000, 0x0000, MSG_02, 8) ) return -3;
	if ( controlRead(deviceHandle, 0xE9, 0x0000, 0x0000, MSG_00, 4) ) return -4;

	// W1: 07 00 03 00 71 7F 12 01
	// R1: 05 00 09 81 ED FE
	if ( bulkWrite(deviceHandle, 1, MSG_03, 8) ) return -5;
	if ( bulkRead(deviceHandle, 1, MSG_04, 6) ) return -6;

	// R0: C0 E9 0000 0000 -> 05 00 10 00
	// R0: C0 E6 0000 0000 -> 03 03
	// R0: C0 E7 0000 0000 -> 0D 00 00 00 00 00 00 00
	// R0: C0 E7 0000 0000 -> 0D 00 00 00 00 00 00 00
	if ( controlRead(deviceHandle, 0xE9, 0x0000, 0x0000, MSG_00, 4) ) return -7;
	if ( controlRead(deviceHandle, 0xE6, 0x0000, 0x0000, MSG_01, 2) ) return -8;
	if ( controlRead(deviceHandle, 0xE7, 0x0000, 0x0000, MSG_02, 8) ) return -9;
	if ( controlRead(deviceHandle, 0xE7, 0x0000, 0x0000, MSG_02, 8) ) return -10;

	// W1: 03 04 00 00
	// R1: 01 00
	if ( bulkWrite(deviceHandle, 1, MSG_05, 4) ) return -11;
	if ( bulkRead(deviceHandle, 1, MSG_06, 2) ) return -12;

	// W1: 08 04 04 00 02 01 00 00 00
	// R1: 01 00
	if ( bulkWrite(deviceHandle, 1, command, 9) ) return -13;
	if ( bulkRead(deviceHandle, 1, MSG_06, 2) ) return -14;

	// W2: A7
	returnCode = usb_bulk_write(deviceHandle, USB_ENDPOINT_OUT | 2, (char*)buffer, nWrites, 5000);
	if ( returnCode != nWrites ) {
		return -15;
	}

	// W1: 03 04 84 00
	// R1: 05 80 NN 00 00 00
	if ( bulkWrite(deviceHandle, 1, MSG_09, 4) ) return -16;
	returnCode = usb_bulk_read(deviceHandle, USB_ENDPOINT_IN | 1, (char*)status, 6, 5000);
	if ( returnCode != 6 ) {
		return -17;
	}
	if ( status[0] != 0x05 || status[1] != 0x80 || memcmp(status+2, command+5, 4) ) {
		return -17;
	}
	
	// W1: 03 04 01 00
	// R1: 01 00
	if ( bulkWrite(deviceHandle, 1, MSG_11, 4) ) return -18;
	if ( bulkRead(deviceHandle, 1, MSG_06, 2) ) return -19;

	return 0;
}

int main(int argc, char* argv[]) {

	struct arg_uint *vidOpt  = arg_uint0("v", "vid",   "<vendorID>",  "  vendor ID (default 0x1443)");
	struct arg_uint *pidOpt  = arg_uint0("p", "pid",   "<productID>", " product ID (default 0x0005)");
	struct arg_lit *readOpt  = arg_lit0( "r", "read",                 "            read from the device");
	struct arg_lit *writeOpt = arg_lit0( "w", "write",                "           write to the device");
	struct arg_file *fileOpt = arg_file0("f", "file",  "<fileName>",  " file to read from or write to (default stdin/stdout)");
	struct arg_uint *lenOpt  = arg_uint0("l", "len",   "<length>",    "    the number of bytes to read or write (or guess from input file length)");
	struct arg_uint *addrOpt = arg_uint1("a", "addr",  "<address>",   "  register to read from or write to");
	struct arg_lit *helpOpt  = arg_lit0( "h", "help",                 "            print this help and exit\n");
	struct arg_end *endOpt   = arg_end(20);
	void* argTable[] = {vidOpt, pidOpt, readOpt, writeOpt, fileOpt, addrOpt, lenOpt, helpOpt, endOpt};
	const char *progName = "nexys2epp";
	uint32 exitCode = 0;
	int numErrors;

	uint16 vid, pid;
	uint32 reg, len;
	uint8 *buffer;
	bool isWrite = false;
	FILE *file = NULL;
	UsbDeviceHandle *deviceHandle;
	int returnCode;

	if ( arg_nullcheck(argTable) != 0 ) {
		printf("%s: insufficient memory\n", progName);
		exitCode = 1;
		goto cleanupArgtable;
	}

	numErrors = arg_parse(argc, argv, argTable);

	if ( helpOpt->count > 0 ) {
		printf("Nexys2 EPP Comms Tool Copyright (C) 2010 Chris McClelland\n\nUsage: %s", progName);
		arg_print_syntax(stdout, argTable, "\n");
		printf("\nInteract with a Nexys2 programmed with dpimref.vhd (or similar).\n\n");
		arg_print_glossary(stdout, argTable,"  %-10s %s\n");
		exitCode = 0;
		goto cleanupArgtable;
	}

	if ( numErrors > 0 ) {
		arg_print_errors(stdout, endOpt, progName);
		printf("Try '%s --help' for more information.\n", progName);
		exitCode = 2;
		goto cleanupArgtable;
	}

	if ( readOpt->count && writeOpt->count ) {
		fprintf(stderr, "You cannot supply both -r and -w\n");
		exitCode = 3;
		goto cleanupArgtable;
	} else if ( readOpt->count ) {
		isWrite = false;
	} else if ( writeOpt->count ) {
		isWrite = true;
	} else {
		fprintf(stderr, "You must supply either -r or -w\n");
		exitCode = 4;
		goto cleanupArgtable;
	}

	if ( fileOpt->count ) {
		file = fopen(fileOpt->filename[0], isWrite?"rb":"wb");
		if ( file == NULL ) {
			fprintf(stderr, "Cannot open file %s for %s\n", fileOpt->filename[0], isWrite?"reading":"writing");
			exitCode = 6;
			goto cleanupArgtable;
		}
	}

	vid = vidOpt->count ? (uint16)vidOpt->ival[0] : VID;
	pid = pidOpt->count ? (uint16)pidOpt->ival[0] : PID;
	reg = (uint32)addrOpt->ival[0];
	if ( lenOpt->count ) {
		// We have the length explicitly - no need to infer it
		len = (uint32)lenOpt->ival[0];
	} else {
		if ( !isWrite ) {
			// Reading from device, writing to file
			fprintf(stderr, "You must specify how many bytes you wish to read!");
			exitCode=90;
			goto cleanupFile;
		} else {
			// Reading from file, writing to device
			if ( !file ) {
				// stdin - length must be specified
				fprintf(stderr, "You must specify how many bytes you wish to write!");
				exitCode=91;
				goto cleanupFile;
			} else {
				// file - get the length
				fseek(file, 0, SEEK_END);
				len = ftell(file);
				fseek(file, 0, SEEK_SET);
			}
		}
	}

	buffer = malloc(len);
	if ( !buffer ) {
		fprintf(stderr, "Cannot allocate %ld bytes!\n", len);
		exitCode = 5;
		goto cleanupFile;
	}

	if ( isWrite ) {
		if ( file ) {
			// Read data from specified file
			uint32 bytesRead;
			bytesRead = fread(buffer, 1, len, file);
			if ( bytesRead != len ) {
				fprintf(stderr, "Whilst reading from \"%s\", expected %lu bytes but got %lu\n", fileOpt->filename[0], len, bytesRead);
				exitCode = 7;
				goto cleanupBuffer;
			}
		} else {
			// Read data from stdin
			uint32 bytesRead;
			#ifdef WIN32
				_setmode(_fileno(stdin), O_BINARY);
			#endif
			bytesRead = fread(buffer, 1, len, stdin);
			if ( bytesRead != len ) {
				exitCode = 8;
				goto cleanupBuffer;
			}
		}
	} else {
		if ( !file ) {
			// Write data to stdout
			#ifdef WIN32
				_setmode(_fileno(stdout), O_BINARY);
			#endif
			file = stdout;
		}
   }

	usbInitialise();
	returnCode = usbOpenDevice(vid, pid, 1, 0, 0, &deviceHandle);
	if ( returnCode ) {
		fprintf(stderr, "usbOpenDevice() failed: %s\n", usbStrError());
		exitCode = 9;
		goto cleanupBuffer;
	}

	if ( isWrite ) {
		returnCode = writeRegister(deviceHandle, reg, len, buffer);
		if ( returnCode != 0 ) {
			fprintf(stderr, "writeRegister() failed returnCode %d\n", returnCode);
			goto cleanupUsb;
		}
	} else {
		returnCode = readRegister(deviceHandle, reg, len, buffer);
		if ( returnCode != 0 ) {
			fprintf(stderr, "writeRegister() failed returnCode %d\n", returnCode);
			goto cleanupUsb;
		}
		fwrite(buffer, 1, len, file);
	}
	
cleanupUsb:
	usb_release_interface(deviceHandle, 0);
	usb_close(deviceHandle);

cleanupBuffer:
	free(buffer);

cleanupFile:
	if ( file != NULL && file != stdout ) {
		fclose(file);
	}

cleanupArgtable:
	arg_freetable(argTable, sizeof(argTable)/sizeof(argTable[0]));

	return exitCode;
}
