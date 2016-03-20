#ifndef PARAMETER_UTILS
#define PARAMETER_UTILS

#include <systemlib/param/param.h>

namespace parameter_utils
{
  float getFloatParam(const char* name);
  int getIntParam(const char* name);
  unsigned int getUIntParam(const char* name);
  unsigned short getUShortParam(const char* name);
};

#endif
