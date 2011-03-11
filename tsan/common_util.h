#ifndef TSAN_COMMON_UTIL__
#define TSAN_COMMON_UTIL__

#include "ts_util.h"

#if defined(__GNUC__)
  typedef int TS_FILE;
  #define TS_FILE_INVALID (-1)
#ifdef TS_LLVM
  #define read(fd, buf, size) __real_read(fd, buf, size)
#endif
#elif defined(_MSC_VER)
  typedef FILE *TS_FILE;
  #define TS_FILE_INVALID (NULL)
  #define read(fd, buf, size) fread(buf, 1, size, fd)
  #define close fclose
#endif

bool StringMatch(const string& wildcard, const string& text);
string ConvertToPlatformIndependentPath(const string &s);
TS_FILE OpenFileReadOnly(const string &file_name, bool die_if_failed);
string ReadFileToString(const string &file_name, bool die_if_failed);

#endif
