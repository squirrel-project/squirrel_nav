// Lextor - Lexicographic Teach Optimize and Repeat
//
// Copyright (C) 2015-2016 Mladen Mazuran, Wolfram Burgard and
// Gian Diego Tipaldi
//
// This file is part of Lextor. The code in external/csmstrip is a modified
// version of the C(anonical) Scan Matcher (http://purl.org/censi/2007/csm).
// Copyright (C) 2007-2015 Andrea Censi
//
// Lextor is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Lextor is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with Lextor.  If not, see <http://www.gnu.org/licenses/>.

/* Some preprocessor magic for having fast inline functions. */
#define INLINE static inline
#ifndef INLINE_DECL
#define INLINE_DECL static inline
#endif

/* Some preprocessor magic for the "restrict" keyword:
        http://www.cellperformance.com/mike_acton/2006/05/demystifying_the_restrict_keyw.html
   */
#if __STDC_VERSION__ >= 199901
#elif defined(__GNUC__) && __GNUC__ >= 2 && __GNUC_MINOR__ >= 91
#define restrict __restrict__
#else
#define restrict
#endif

/* Some preprocessor magic for calling this library from C++ */

#ifdef __cplusplus
#define restrict /* nothing */
#endif
