/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Author(s): Marc Rittinghaus <marc.rittinghaus@kit.edu>
 *
 * Copyright (c) 2021, Karlsruhe Institute of Technology. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __UKNOFAULT_NOFAULT_H__
#define __UKNOFAULT_NOFAULT_H__

#include <uk/arch/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Probes the specified address range for read access. Page faults may be
 * resolved during the operation.
 *
 * @param addr Start of address range
 * @param len Number of bytes to check
 * @return 1 on success, 0 otherwise
 */
int uk_memprobe_r(unsigned long addr, __sz len);

/**
 * Probes the specified address range for read access. Page faults are not
 * resolved.
 *
 * @param addr Start of address range
 * @param len Number of bytes to check
 * @return 1 on success, 0 otherwise
 */
int uk_memprobe_r_isr(unsigned long addr, __sz len);

/**
 * Probes the specified address range for read/write access. Page faults may be
 * resolved during the operation.
 *
 * @param addr Start of address range
 * @param len Number of bytes to check
 * @return 1 on success, 0 otherwise
 */
int uk_memprobe_rw(unsigned long addr, __sz len);

/**
 * Probes the specified address range for read/write access. Page faults are
 * not resolved.
 *
 * @param addr Start of address range
 * @param len Number of bytes to check
 * @return 1 on success, 0 otherwise
 */
int uk_memprobe_rw_isr(unsigned long addr, __sz len);

/**
 * Copies len bytes from src to dst. The function returns an error if
 * an exception occurs (e.g., due to an invalid memory mapping). Page faults
 * may be resolved during the operation.
 *
 * @param dst Destination buffer
 * @param src Source buffer
 * @return Number of bytes copied, -1 on fault
 */
__ssz uk_memcpy_nofault(char *dst, const char *src, __sz len);

/**
 * Copies len bytes from src to dst. The function returns an error if
 * an exception occurs (e.g., due to an invalid memory mapping). Page faults
 * are not resolved.
 *
 * @param dst Destination buffer
 * @param src Source buffer
 * @return Number of bytes copied, -1 on fault
 */
__ssz uk_memcpy_nofault_isr(char *dst, const char *src, __sz len);

#ifdef __cplusplus
}
#endif

#endif /* __UKNOFAULT_NOFAULT_H__ */
