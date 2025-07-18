/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.

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

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

  extern char *itoa(int value, char *string, int radix);
  extern char *ltoa(long value, char *string, int radix);

#if __GNUC__ > 4 || (__GNUC__ == 4 && (__GNUC_MINOR__ > 9 || (__GNUC_MINOR__ == 9 && __GNUC_PATCHLEVEL__ > 2)))
  extern char *utoa_(unsigned value, char *string, int radix);
#else
  extern char *utoa_(unsigned long value, char *string, int radix);
#endif

  extern char *ultoa(unsigned long value, char *string, int radix);

#ifdef __cplusplus
} // extern "C"
#endif
