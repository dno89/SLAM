/**
 * \file DMDebug.h
 * \author Daniele Molinari
 * \version 2.3
 */

/*
 *  Some useful debug macro
 *  Copyright (C) 2011  <copyright holder> <email>
 * 
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 * 
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 * 
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef	_DMDEBUG_H
#define	_DMDEBUG_H

////std libs
#include <fstream>
#include <string>
// #include <chrono>
////boost
#include <boost/date_time/posix_time/posix_time.hpp>	///FIXME remove boost

#ifndef NDEBUG

/*
 * 	CREATE OR IMPORT THE DEBUG FILE
 */

#define CREATE_PUBLIC_DEBUG_LOG(FNAME,FTOKEN)\
std::ofstream _dm_log##FTOKEN(FNAME, std::ios_base::trunc);\
static std::ofstream& _log_alias = _dm_log##FTOKEN;\
static int _verbose_level = 3;\
static int _nested = 0;

#define CREATE_PRIVATE_DEBUG_LOG(FNAME,FTOKEN)\
static std::ofstream _dm_log##FTOKEN(FNAME, std::ios_base::trunc);\
static std::ofstream& _log_alias = _dm_log##FTOKEN;\
static int _verbose_level = 3;\
static int _nested = 0;

//ctime(&chrono::high_resolution_clock::to_time_t(high_resolution_clock::now()));

#define CREATE_PUBLIC_DEBUG_LOG_WTIME(FNAME,FTOKEN)\
std::ofstream _dm_log##FTOKEN((std::string(FNAME)+boost::posix_time::to_iso_extended_string(boost::posix_time::second_clock::local_time())+".log").c_str(), std::ios_base::trunc);\
static std::ofstream& _log_alias = _dm_log##FTOKEN;\
static int _verbose_level = 3;\
static int _nested = 0;

#define CREATE_PRIVATE_DEBUG_LOG_WTIME(FNAME,FTOKEN)\
static std::ofstream _dm_log##FTOKEN((std::string(FNAME)+boost::posix_time::to_iso_extended_string(boost::posix_time::second_clock::local_time())+".log").c_str(), std::ios_base::trunc);\
static std::ofstream& _log_alias = _dm_log##FTOKEN;\
static int _verbose_level = 3;\
static int _nested = 0;

#define IMPORT_DEBUG_LOG(FTOKEN)\
extern std::ofstream _dm_log##FTOKEN;\
static std::ofstream& _log_alias = _dm_log##FTOKEN;\
static int _verbose_level = 3;\
static int _nested = 0;

/**
 * _verbose_level = 0 --> ERROR 
 * _verbose_level = 1 --> ERROR, WARNING
 * _verbose_level = 2 --> ERROR, WARNING, INFO
 */
#define INIT()\
static int _verbose_level = 3;\
static int _nested = 0;

/**
 * Setting verbose level
 */
#define DSET_VERBOSE(l)\
_verbose_level = l;

/**
 * Direct access to the log stream
 */
#define DLOG()\
_log_alias

/**
 * Open and close new context
 */
#define DOPEN_CONTEXT(C)\
_log_alias << boost::posix_time::to_iso_extended_string(boost::posix_time::microsec_clock::local_time()) + "  " << std::string(_nested, '\t') << ">> " << C << std::endl;\
++_nested;

#define DCLOSE_CONTEXT(C)\
--_nested;\
_log_alias << boost::posix_time::to_iso_extended_string(boost::posix_time::microsec_clock::local_time()) + "  " << std::string(_nested, '\t') << "<< " << C << std::endl;

/**
 * Printing generic text
 */
#define DPRINT(X)\
_log_alias << boost::posix_time::to_iso_extended_string(boost::posix_time::microsec_clock::local_time()) + "  " << std::string(_nested, '\t') << X << std::endl;

/**
 * Printing info, warning and error
 */
#define DINFO(X)\
if(_verbose_level >= 2) {\
_log_alias << boost::posix_time::to_iso_extended_string(boost::posix_time::microsec_clock::local_time()) + "  " << std::string(_nested, '\t') << "@INFO: " << X << std::endl;\
}

#define DWARNING(X)\
if(_verbose_level >= 1) {\
_log_alias << boost::posix_time::to_iso_extended_string(boost::posix_time::microsec_clock::local_time()) + "  " << std::string(_nested, '\t') << "@WARNING: " << X << std::endl;\
}

#define DERROR(X)\
if(_verbose_level >= 0) {\
_log_alias << boost::posix_time::to_iso_extended_string(boost::posix_time::microsec_clock::local_time()) + "  " << std::string(_nested, '\t') << "@ERROR: " << X << std::endl;\
}

/**
 * Tracing variable value
 */
#define DTRACE(X)\
_log_alias << boost::posix_time::to_iso_extended_string(boost::posix_time::microsec_clock::local_time()) + "  " << std::string(_nested, '\t') << #X << " = " << (X) << std::endl;

#define DTRACE_H(X)\
_log_alias << boost::to_iso_extended_string(boost::posix_time::microsec_clock::local_time()) + "  " << std::string(_nested, '\t') << #X << " = " << std::hex << (X) << std::endl;

#define DTRACE_L(X)\
_log_alias << boost::posix_time::to_iso_extended_string(boost::posix_time::microsec_clock::local_time()) + "  " << std::string(_nested, '\t') << #X << " =\n" << (X) << std::endl;

#define DTRACE_PRESERVE(X)\
_log_alias << boost::posix_time::to_iso_extended_string(boost::posix_time::microsec_clock::local_time()) + "  " << std::string(_nested, '\t') << #X << " = " << (X) << std::endl;

#else

#define CREATE_PUBLIC_DEBUG_LOG(FNAME,FTOKEN)
#define CREATE_PUBLIC_DEBUG_LOG_WTIME(FNAME,FTOKEN)
#define CREATE_PRIVATE_DEBUG_LOG(FNAME,FTOKEN)
#define CREATE_PRIVATE_DEBUG_LOG_WTIME(FNAME,FTOKEN)
#define IMPORT_DEBUG_LOG(FTOKEN)
// #define INIT()
#define DLOG
#define DSET_VERBOSE(l)
#define DOPEN_CONTEXT(C)
#define DCLOSE_CONTEXT(C)
#define DINFO(X)
#define DWARNING(X)
#define DERROR(X)
#define DPRINT(X)
#define DTRACE(X)
#define DTRACE_H(X)
#define DTRACE_L(X)
#define DTRACE_PRESERVE(X) X
#endif  //NDEBUG

#endif	//_DMDEBUG_H
