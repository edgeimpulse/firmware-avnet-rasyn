/*
 * Copyright (c) 2020-2024 Syntiant Corp.  All rights reserved.
 * Contact at http://www.syntiant.com
 *
 * This software is available to you under a choice of one of two licenses.
 * You may choose to be licensed under the terms of the GNU General Public
 * License (GPL) Version 2, available from the file LICENSE in the main
 * directory of this source tree, or the OpenIB.org BSD license below.  Any
 * code involving Linux software will require selection of the GNU General
 * Public License (GPL) Version 2.
 *
 * OPENIB.ORG BSD LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * DEALINGS IN THE SOFTWARE.
 	** SDK: v110 **
*/

/*
 * This file is autogenerated based on current git repo.
 * To update, run:
 *
 *   scripts/set_ilib_version.py
 */

#ifndef SYNTIANT_NDP_ILIB_VERSION_H
#define SYNTIANT_NDP_ILIB_VERSION_H

#define SYNTIANT_NDP_ILIB_RELEASE_HASH \
    "75d39286cb33efa7da921ab899103262d1a88ad1"

#define SYNTIANT_NDP_ILIB_SYNTIANT_CORE_2_MAJOR_VERSION 1
#define SYNTIANT_NDP_ILIB_SYNTIANT_CORE_2_MINOR_VERSION 137
#define SYNTIANT_NDP_ILIB_SYNTIANT_CORE_2_PATCH_VERSION 0


#define STR_HELPER(x)   #x
#define STR(x)          STR_HELPER(x)

#define SYNTIANT_NDP_ILIB_VERSION_2 \
    "syntiant_core_2 " \
    STR(SYNTIANT_NDP_ILIB_SYNTIANT_CORE_2_MAJOR_VERSION) "." \
    STR(SYNTIANT_NDP_ILIB_SYNTIANT_CORE_2_MINOR_VERSION) "." \
    STR(SYNTIANT_NDP_ILIB_SYNTIANT_CORE_2_PATCH_VERSION)

#define SYNTIANT_NDP_ILIB_VERSION \
    SYNTIANT_NDP_ILIB_VERSION_2 \
    " " SYNTIANT_NDP_ILIB_RELEASE_HASH

#endif

