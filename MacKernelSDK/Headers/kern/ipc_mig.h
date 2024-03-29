/*
 * Copyright (c) 2000-2005 Apple Computer, Inc. All rights reserved.
 *
 * @APPLE_OSREFERENCE_LICENSE_HEADER_START@
 *
 * This file contains Original Code and/or Modifications of Original Code
 * as defined in and that are subject to the Apple Public Source License
 * Version 2.0 (the 'License'). You may not use this file except in
 * compliance with the License. The rights granted to you under the License
 * may not be used to create, or enable the creation or redistribution of,
 * unlawful or unlicensed copies of an Apple operating system, or to
 * circumvent, violate, or enable the circumvention or violation of, any
 * terms of an Apple operating system software license agreement.
 *
 * Please obtain a copy of the License at
 * http://www.opensource.apple.com/apsl/ and read it before using this file.
 *
 * The Original Code and all software distributed under the License are
 * distributed on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER
 * EXPRESS OR IMPLIED, AND APPLE HEREBY DISCLAIMS ALL SUCH WARRANTIES,
 * INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR NON-INFRINGEMENT.
 * Please see the License for the specific language governing rights and
 * limitations under the License.
 *
 * @APPLE_OSREFERENCE_LICENSE_HEADER_END@
 */
/*
 * @OSF_COPYRIGHT@
 */

#ifndef _KERN_IPC_MIG_H_
#define _KERN_IPC_MIG_H_

#include <mach/mig.h>
#include <mach/mach_types.h>
#include <mach/message.h>
#include <kern/kern_types.h>

#include <sys/cdefs.h>


__BEGIN_DECLS

/* Send a message from the kernel */

#if __MAC_OS_X_VERSION_MIN_REQUIRED >= __MAC_10_6
extern mach_msg_return_t mach_msg_send_from_kernel_proper(
	mach_msg_header_t       *msg,
	mach_msg_size_t         send_size);

#define mach_msg_send_from_kernel mach_msg_send_from_kernel_proper
#else
extern mach_msg_return_t mach_msg_send_from_kernel(
  mach_msg_header_t       *msg,
  mach_msg_size_t         send_size);
#endif

extern mach_msg_return_t
mach_msg_rpc_from_kernel_proper(
	mach_msg_header_t       *msg,
	mach_msg_size_t         send_size,
	mach_msg_size_t         rcv_size);

#define mach_msg_rpc_from_kernel mach_msg_rpc_from_kernel_proper


extern void
mach_msg_destroy_from_kernel_proper(
	mach_msg_header_t       *msg);

#define mach_msg_destroy_from_kernel mach_msg_destroy_from_kernel_proper

extern mach_msg_return_t mach_msg_send_from_kernel_with_options(
	mach_msg_header_t       *msg,
	mach_msg_size_t         send_size,
	mach_msg_option_t       option,
	mach_msg_timeout_t      timeout_val);

__END_DECLS


#endif  /* _KERN_IPC_MIG_H_ */
