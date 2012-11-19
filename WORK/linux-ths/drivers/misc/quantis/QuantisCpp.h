/*
 * C++ wrapper for quantis driver
 * Copyright (c) 2004, 2005, id Quantique SA, Switzerland
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

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <string>
#include <iostream>
#if defined(__GNUC__) && (__GNUC__ <= 2) && (_GNUC_MINOR <= 95)
#include <limits.h>
#else
#include <limits>
#endif
#include <exception>

#include "quantisio.h"

namespace Quantis {
  using std::string;

  /** We are at version 1.1. **/
  static const int QuantisLibVersion = 11;
  
  /*
   * Error class used by all the Quantis API. Basically just a wrapper
   * for strerror message.
   */
  class QuantisError : public std::exception {
    string errorMessage;

  public:
    QuantisError(string errorMessage = "unknown error") :
      std::exception(), errorMessage(errorMessage) {}
    ~QuantisError() throw () {}

    const string getErrorMessage() { return errorMessage; }
  };

  /*
   * Class for automatic file descriptor resource management, and for
   * unsigned int ioctl reading and writing.
   */
  class FilePtr {
  private:
    int fd;
    void openFile(string filename, int mode);

  public:
    FilePtr(string filename, int mode = O_RDONLY);
    FilePtr(string baseName, int cardNumber, int mode = O_RDONLY);
    ~FilePtr() { if (fd > 0) close(fd); }

    operator int() const { return fd; }
    unsigned int readIoctl(int request);
    void writeIoctl(int request, unsigned int value);
    unsigned int read(unsigned char *buffer, unsigned int length);
    void readBlocking(unsigned char *buffer, unsigned int length);
  };

  /*
   * The Quantis wrapper class.
   */
  const string defaultBaseName = "/dev/qrandom/";

  class Quantis {
    int cardNumber;
    int baseName;

    FilePtr fd;

  public:

    static const unsigned int Module1 = 0x1;
    static const unsigned int Module2 = 0x1 << 1;
    static const unsigned int Module3 = 0x1 << 2;
    static const unsigned int Module4 = 0x1 << 3;
    static const unsigned int AllModules = Module1 | Module2 | Module3 | Module4;
    static const unsigned int MaxCards = 10;
    
    static int getLibVersion() { return QuantisLibVersion; }
    
    static int getCardCount(string baseName = defaultBaseName);
    static int getDriverVersion(string baseName = defaultBaseName);
    
    Quantis(int cardNumber = 0, string baseName = defaultBaseName) :
      fd(baseName, cardNumber, O_RDONLY) {}
    ~Quantis() {}
    
    int getModulesMask() {
      return fd.readIoctl(QUANTIS_IOCTL_GET_MODULES_MASK);
    }
    int getModulesStatus() {
      return fd.readIoctl(QUANTIS_IOCTL_GET_MODULES_STATUS);
    }
    int getBoardVersion() {
      return fd.readIoctl(QUANTIS_IOCTL_GET_BOARD_VERSION);
    }

    void reset() {
      fd.writeIoctl(QUANTIS_IOCTL_RESET_BOARD, 0);
    }

    void enableModules (int moduleMask) {
      fd.writeIoctl(QUANTIS_IOCTL_ENABLE_MODULE, moduleMask);
    }
    void disableModules(int moduleMask) {
      fd.writeIoctl(QUANTIS_IOCTL_DISABLE_MODULE, moduleMask);
    }
    void setEnabledModules(int moduleMask) {
      disableModules(AllModules);
      enableModules(moduleMask);
    }
    void resetModules  (int moduleMask) {
      enableModules(moduleMask);
    }      
    
    unsigned char  readByte();
    unsigned short readShort();
    unsigned long  readLong();
    unsigned long  readBoundedLong(unsigned long max) {
#if defined(__GNUC__) && (__GNUC__ <= 2) && (_GNUC_MINOR <= 95)
      return (unsigned long)
	((double)(((double)max + 1.0) * (double)readLong()) /
	 (ULONG_MAX + 1.0));
#else
      return (unsigned long)
	((double)(((double)max + 1.0) * (double)readLong()) /
	 (std::numeric_limits<unsigned long>::max() + 1.0));
#endif
    }

    unsigned int read(unsigned char *buffer, unsigned int length);
    void readBlocking(unsigned char *buffer, unsigned int length);
  };
}
