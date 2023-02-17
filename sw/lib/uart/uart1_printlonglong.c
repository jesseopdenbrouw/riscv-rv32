// #################################################################################################
// # This file is part of the THUAS RV32 processor                                                 #
// # ********************************************************************************************* #
// # BSD 3-Clause License                                                                          #
// #                                                                                               #
// # Copyright (c) 2022, Jesse op den Brouw. All rights reserved.                                  #
// #                                                                                               #
// # Redistribution and use in source and binary forms, with or without modification, are          #
// # permitted provided that the following conditions are met:                                     #
// #                                                                                               #
// # 1. Redistributions of source code must retain the above copyright notice, this list of        #
// #    conditions and the following disclaimer.                                                   #
// #                                                                                               #
// # 2. Redistributions in binary form must reproduce the above copyright notice, this list of     #
// #    conditions and the following disclaimer in the documentation and/or other materials        #
// #    provided with the distribution.                                                            #
// #                                                                                               #
// # 3. Neither the name of the copyright holder nor the names of its contributors may be used to  #
// #    endorse or promote products derived from this software without specific prior written      #
// #    permission.                                                                                #
// #                                                                                               #
// # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS   #
// # OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF               #
// # MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE    #
// # COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,     #
// # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE #
// # GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED    #
// # AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     #
// # NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED  #
// # OF THE POSSIBILITY OF SUCH DAMAGE.                                                            #
// # ********************************************************************************************* #
// # Print signed long long integers                                                               #
// #################################################################################################

#include <stdio.h>
#include <stdarg.h>

#include <thuasrv32.h>


/**
 * @file uart1_printlonglong.c
 * @brief Function to print signed long long integer
 * 
 * @param[in] v The value to print
 */
void uart1_printlonglong(int64_t v)
{
	int neg = 0;
	uint64_t uv;

	if (v < 0) {
		uv = (uint64_t) -v;
		neg = 1;
	} else {
		uv = (uint64_t) v;
	}

	uint32_t fbillion = (uint32_t) (uv % 1000000000);
	uv = uv / 1000000000;
	uint32_t nbillion = (uint32_t) (uv % 1000000000);
	uv = uv / 1000000000;

	if (neg) {
		uart1_putc('-');
	}
	if (uv > 0) {
		uart1_printf("%u", (uint32_t) uv);
		uart1_printf("%09u", nbillion);
	} else {
		if (nbillion > 0) {
			uart1_printf("%u", nbillion);
		}
	}
	if (uv > 0 || nbillion > 0) {
		uart1_printf("%09u", fbillion);
	} else {
		uart1_printf("%u", fbillion);
	}
}
