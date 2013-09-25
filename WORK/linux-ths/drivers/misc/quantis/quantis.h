/*
 * Copyright (c) 2004, 2005, 2006, id Quantique SA, Switzerland
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * Neither the name of id Quantique nor the names of its contributors may be
 * used to endorse or promote products derived from this software without
 * specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**# Introduction
 **
 ** The Quantis library can be used to access Quantis Quantum Random Number
 ** Generator. The library API is the same for the PCI and the USB library
 ** and is OS independent. This documentation has been automatically generated
 ** from the library sourcecode, and contains the complete header for the
 ** library, as well as examples of using the library.
 **/

/**# Quantis library **/

/**## Preamble
 ** We define the `QUANTIS_H' symbol to avoid multiple inclusion of
 ** the header file, and export the symbols as `C' symbols when the
 ** library is used in C++ mode.
 **/

#ifndef QUANTIS_H
#define QUANTIS_H

/** We are at version 1.1. **/
#define QUANTIS_LIB_VERSION 11

#ifdef __cplusplus
extern "C" {
#endif

/**# Informative functions
 **
 ** These functions return information about the number of cards
 ** present, the version of the hardware and of the
 ** software, as well as number of modules present on a board.
 **
 ** If the PCI library is used, the card number parameter refers
 ** to the number of the Quantis PCI card and if the USB library
 ** is used the card number parameter refers to the number of the
 ** Quantis USB device.
 **
 ** The Quantis PCI library is named `libquantis'. The Quantis USB
 ** library is named `libquantis-usb'.
 **/

/**## quantisCount - return the number of Quantis boards
 **
 ** Returns the number of cards installed in the system.
 **
 ** If an error occured, the function returns 0 (we assume that no
 ** card are installed in the system).
 **/
int quantisCount(void);

/**## quantisBoardVersion - get the Quantis board version
 **
 ** This function takes the card number as a parameter and returns
 ** the version of the board, or 0 (which is an invalid version number)
 ** when an error occurred. The version is a 4 bytes number in the format:
 **
 ** `[year][month][day][r]'
 **
 **  `31..24 23..16 15..8 7..0'
 **
 ** where `r' indicates the release number of the day.
 **/
int quantisBoardVersion(int cardNumber);

/**## quantisLibVersion - get the Quantis library version
 **
 ** This function returns the version of the library. The version is a
 ** decimal number composed of the major number and the minor version:
 ** `version = major * 10 + minor'.
 **/
int quantisLibVersion(void);

/**## quantisDriverVersion - get the version of the Quantis driver
 **
 ** This function returns the version of the Quantis driver. The
 ** version is a decimal number composed of the major number and the
 ** minor version: `version = major * 10 + minor'.
 **
 ** It returns 0 (which is an invalid version number) when an error
 ** occured or no card could be found.
 **/
int quantisDriverVersion(void);

/**# Board functions
 **
 ** These functions are used to reset, enable, disable and get the
 ** status of modules on a Quantis board.
 **/

/**## quantisBoardReset (lowlevel) - reset the Quantis board
 **
 ** This function takes the card number as a parameter.  It resets the
 ** board, setting all values to their initial state and enabling all
 ** the modules present on the card.
 **
 ** The function returns 0 on success, and a negative value on error.
 **
 ** Normally, this function does not need to be called, as the board
 ** is reset on driver loading (on unix systems) or dll loading (on
 ** windows systems).
 **
 **/
int quantisBoardReset(int cardNumber);

/**## quantisGetModules - detect the modules on a board
 **
 ** This function takes the card number as a parameter. The return
 ** value is a bitmask of the modules present on the card. Bit 0 is
 ** set when module 0 is present, bit 1 when module 1 is present, bit
 ** 2 when module 2 is present and bit 3 when module 3 is present. For
 ** example, a return value of `101' in binary (5 in decimal) shows
 ** that module 0 and module 2 are present.
 **
 ** If the card is not present or an error occured, a bitmask of 0 is
 ** returned (no modules installed).
 **/
int quantisGetModules(int cardNumber);

/**## quantisModulesStatus (lowlevel) - get the status of the modules on a board
 **
 ** This function takes the card number as first parameter.
 **
 ** This function returns the status of the modules on the card as a
 ** bitmask similar to the second argument to `quantisModulesReset'. A
 ** set bit signals the module is in a correct state. It returns -1 on
 ** error.
 **
 ** This function returns only the status of the modules that are
 ** currently enabled.
 **/
int quantisModulesStatus(int cardNumber);

/**## quantisModulesReset (lowlevel) - reset some modules on a board
 **
 ** This function takes the card number as first parameter, and the
 ** mask of the modules to reset as second parameter. The mask is
 ** similar to the bitmask returned by the `quantisGetModules' function,
 ** bit 0 corresponds to module 0, bit 1 to module 1, bit 2 to module 2,
 ** bit 3 to module 3. The modules for which the bitmask is set are
 ** reset.
 **
 ** This function returns 0 on success, and -1 if an error occured.
 **
 ** This function first disables the modules by calling
 ** `quantisModulesDisable' and then enables the modules by calling
 ** `quantisModulesEnable'. Thus, only the reset modules will be
 ** enabled after a call to `quantisModulesReset'.
 **/
int quantisModulesReset(int cardNumber, int moduleMask);

/**## quantisModulesEnable (lowlevel) - enable some modules on a board
 **
 ** This function takes the card number as first parameter, and the
 ** mask of the modules to enable as second parameter. The mask is
 ** similar to the bitmask given as second argument to
 ** `quantisModulesReset'.
 **
 ** This function returns 0 on success, and -1 if an error occured.
 **
 ** After a call to `quantisModulesEnable', reading from the Quantis
 ** board using `quantisRead' will also read from the enabled
 ** modules in addition to the already enabled modules.
 **/
int quantisModulesEnable(int cardNumber, int moduleMask);

/**## quantisModulesDisable (lowlevel) - disable some modules on a board
 **
 ** This function takes the card number as first parameter, and the
 ** mask of the modules to disable as second parameter. The mask is
 ** similar to the bitmask given as second argument to `quantisModulesReset'.
 **
 ** This function returns 0 on success, and -1 if an error occured.
 **
 ** After a call to `quantisModulesDisable', reading from the Quantis
 ** board using `quantisRed' will not read from the disabled
 ** modules anymore.
 **/
int quantisModulesDisable(int cardNumber, int moduleMask);

/**# Read function
 **
 ** This is the most important function that reads random data from
 ** the Quantis board.
 **/

/**## quantisRead - read from a Quantis board
 **
 ** This function takes the card number as first parameter, a pointer
 ** to a destination buffer in memory as second argument, and the number
 ** of bytes to read from the board as third parameter.
 **
 ** This function returns the number of bytes read on success, and -1
 ** on error. In case of success, it always returns the exact number
 ** of bytes requested.
 **
 ** Please note with PCI version that reading a single random byte is
 ** inefficient, as the FIFO of the PCI board is read in 32 bytes chunk. For
 ** high-throughput, multiples of 4 bytes should be read from the PCI
 ** board or the USB device.
 **/
int quantisRead(int cardNumber, void *buffer, unsigned int size);

/**## Postamble
 **
 ** Close the C++ `extern "C"' environment.
 **/
#ifdef __cplusplus
}
#endif

#endif // QUANTIS_H

