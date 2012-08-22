/* * IAP - Interactive Perception System - Segment visually observable environment
 * into rigid bodies and estimate type and properties of joints between them by
 * means of interaction.
 * Copyright (C) 2012 Technische Universitaet Berlin - RBO
 * <robotics@robotics.tu-berlin.de>
 * 
 * This file is part of IAP.
 * 
 * IAP is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * IAP is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with IAP.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
 * DataLogger.h
 *
 * A simple class for logging double, vector<double> string etc into a simple text file.
 * Useful for plotting, e.g. you can load such a file directly with Matlab or numpy, e.g.:
 * <code>myData = numpy.loadtxt("yourfile.txt")</code>
 *
 * The class is basically a convenient wrapper for fstream.
 *
 * Usage: Instantiate a class and pass the name+path of your file in the constructor.
 *
 *  Created on: Apr 28, 2011
 *      Author: shoefer
 */

#ifndef DATALOGGER_H_
#define DATALOGGER_H_

#include <iostream>
#include <fstream>
#include <string>
#include "assert.h"
#include <stdexcept>
#include <cstdlib>
#include <string.h>
#include <vector>

#define MAX_LINE_LENGTH 1024

//#define VERBOSE_LOGGING

namespace iap_common
{

class DataLogger
{
  std::fstream handle;
  std::string filename;
  char* linebuffer;

public:
  DataLogger(std::string f) :
    filename(f)
  {
    linebuffer = NULL;
#ifdef VERBOSE_LOGGING
    std::cout << "[DATALOG] Creating: " << filename << std::endl;
#endif
  }

  virtual ~DataLogger()
  {
    if (isOpen())
    {
      close();
    }
  }

  bool isOpen()
  {
    return handle.is_open();
  }

  virtual void openForWrite()
  {
#ifdef VERBOSE_LOGGING
    std::cout << "[DATALOG] Opening for write" << std::endl;
#endif
    handle.open(filename.c_str(), std::fstream::out);
    if (!isOpen())
    {
      std::cout << filename << std::endl;
      throw std::runtime_error("[DATALOG] Opening file for write failed");
    }
  }

  virtual void openForRead()
  {
    handle.open(filename.c_str(), std::fstream::in);
    if (!isOpen())
    {
      std::cout << filename << std::endl;
      throw std::runtime_error("[DATALOG] Opening file for read failed");
    }
  }

  void deleteFile()
  {
    if (remove(filename.c_str()) == -1)
    {
      std::cout << filename << std::endl;
      throw std::runtime_error("[DATALOG] Removing file failed");
    }
  }

  virtual double readLineAsDouble()
  {
    return atof(readLine());
  }

  virtual int readLineAsInt()
  {
    return atoi(readLine());
  }

  virtual char* readLine()
  {
    if (!isOpen())
    {
      openForRead();
    }

    char* ret = new char[MAX_LINE_LENGTH];
    char* buffer = new char[MAX_LINE_LENGTH];
    handle.getline(buffer, MAX_LINE_LENGTH);

    if (linebuffer != NULL)
    {
      // nth read: return linebuffer, store buffer
      strcpy(ret, linebuffer);
      if (*buffer == '\0')
      {
        linebuffer = NULL;
      }
      else
      {
        strcpy(linebuffer, buffer);
      }
    }
    else
    {
      // first read: return buffer, lookahead to store linebuffer
      strcpy(ret, buffer);
      if (!handle.eof())
      {
        linebuffer = new char[MAX_LINE_LENGTH];
        handle.getline(linebuffer, MAX_LINE_LENGTH);
      }
    }

    return ret;
  }

  virtual bool eof()
  {
    return handle.eof() && linebuffer == NULL;
  }

  virtual void write(double data)
  {
#ifdef VERBOSE_LOGGING
    std::cout << "[DATALOG] Writing " << data << std::endl;
#endif
    if (!isOpen())
    {
      openForWrite();
    }
    // always add endl!
    handle << data << std::endl;
  }

  virtual void write(std::string data)
  {
#ifdef VERBOSE_LOGGING
    std::cout << "[DATALOG] Writing " << data << std::endl;
#endif
    if (!isOpen())
    {
      openForWrite();
    }
    handle << data << std::endl;
  }

  virtual void write(std::vector<double> data)
  {
#ifdef VERBOSE_LOGGING
    std::cout << "[DATALOG] Writing vector of size " << data.size() << std::endl;
#endif
    if (!isOpen())
    {
      openForWrite();
    }
    for (std::vector<double>::iterator it = data.begin(); it != data.end(); it++)
    {
      handle << *it << "\t";
    }
    // always add endl!
    handle << std::endl;
  }

  virtual void close()
  {
#ifdef VERBOSE_LOGGING
    std::cout << "[DATALOG] Close " << filename << std::endl;
#endif
    handle.close();
  }
};

}

#endif /* DATALOGGER_H_ */
