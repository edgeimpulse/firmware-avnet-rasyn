/*
 * Copyright (c) 2022 Syntiant Corp.  All rights reserved.
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
 * DEALINGS IN THE SOFTWARE.
 	** SDK: v110 **
*/

#ifndef __SYNTIANT_CSPI_
#define __SYNTIANT_CSPI_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief NDP120 Controller SPI write
 *
 * Gets the coniguration of the device like device id, device family id, firmware version etc.
 *
 * @param ndp NDP state object
 * @param ssb the SSB pin to use
 * @param num_bytes number of bytes to write
 * @param data data to write
 * @param end_packet end packet, i.e. let SSB return to high, or continue packet with SSB low
 * @return a @c SYNTIANT_NDP_ERROR_* code
 */
int syntiant_cspi_write(struct syntiant_ndp120_tiny_device_s *ndp,
    unsigned int ssb, unsigned int num_bytes, const void *data, int end_packet);

/**
 * @brief NDP120 Controller SPI read
 *
 * Gets the coniguration of the device like device id, device family id, firmware version etc.
 *
 * @param ndp NDP state object
 * @param ssb the SSB pin to use
 * @param num_bytes number of bytes to read
 * @param data data to return
 * @param end_packet end packet, i.e. let SSB return to high, or continue packet with SSB low
 * * @return a @c SYNTIANT_NDP_ERROR_* code
 */
int syntiant_cspi_read (struct syntiant_ndp120_tiny_device_s *ndp,
    unsigned int ssb, unsigned int num_bytes, void *data, int end_packet);

/**
 * @brief Initialize NDP120 Controller SPI
 *
 * Initialize for Host-based Controller SPI operation
 *
 * @param ndp NDP state object
 * @return a @c SYNTIANT_NDP_ERROR_* code
 */
int syntiant_cspi_init(struct syntiant_ndp120_tiny_device_s *ndp);

#ifdef __cplusplus
}
#endif

#endif
