/*
  Copyright (c) 2014 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "itoa.h"

#include <string.h>

#ifdef __cplusplus
extern "C"
{
#endif

  extern char *itoa(int value, char *string, int radix)
  {
    return ltoa(value, string, radix);
  }

  extern char *ltoa(long value, char *string, int radix)
  {
    char tmp[33];
    char *tp = tmp;
    long i;
    unsigned long v;
    int sign;
    char *sp;

    if (string == NULL)
    {
      return 0;
    }

    if (radix > 36 || radix <= 1)
    {
      return 0;
    }

    sign = (radix == 10 && value < 0);
    if (sign)
    {
      v = -value;
    }
    else
    {
      v = (unsigned long)value;
    }

    while (v || tp == tmp)
    {
      i = v % radix;
      v = v / radix;
      if (i < 10)
        *tp++ = i + '0';
      else
        *tp++ = i + 'a' - 10;
    }

    sp = string;

    if (sign)
      *sp++ = '-';
    while (tp > tmp)
      *sp++ = *--tp;
    *sp = 0;

    return string;
  }

#if __GNUC__ > 4 || (__GNUC__ == 4 && (__GNUC_MINOR__ > 9 || (__GNUC_MINOR__ == 9 && __GNUC_PATCHLEVEL__ > 2)))
  extern char *utoa_(unsigned value, char *string, int radix)
#else
extern char *utoa_(unsigned long value, char *string, int radix)
#endif
  {
    return ultoa(value, string, radix);
  }

  extern char *ultoa(unsigned long value, char *string, int radix)
  {
    char tmp[33];
    char *tp = tmp;
    long i;
    unsigned long v = value;
    char *sp;

    if (string == NULL)
    {
      return 0;
    }

    if (radix > 36 || radix <= 1)
    {
      return 0;
    }

    while (v || tp == tmp)
    {
      i = v % radix;
      v = v / radix;
      if (i < 10)
        *tp++ = i + '0';
      else
        *tp++ = i + 'a' - 10;
    }

    sp = string;

    while (tp > tmp)
      *sp++ = *--tp;
    *sp = 0;

    return string;
  }

#ifdef __cplusplus
} // extern "C"
#endif
