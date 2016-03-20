#include "ParameterUtils.h"

float parameter_utils::getFloatParam(const char* name)
{
  param_t h = param_find(name);
  float out;
  param_get(h, &out);
  return out;
}

int parameter_utils::getIntParam(const char* name)
{
  param_t h = param_find(name);
  int out;
  param_get(h, &out);
  return out;
}

unsigned int parameter_utils::getUIntParam(const char* name)
{
  param_t h = param_find(name);
  int out;
  param_get(h, &out);
  return (unsigned int)out;
}

unsigned short parameter_utils::getUShortParam(const char* name)
{
  param_t h = param_find(name);
  int out;
  param_get(h, &out);
  return (unsigned short)out;
}
