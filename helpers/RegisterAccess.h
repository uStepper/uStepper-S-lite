/*
 * RegisterAccess.h
 *
 *  Created on: 12.07.2017
 *      Author: LK
 */

#ifndef TMC_HELPERS_REGISTERACCESS_H
#define TMC_HELPERS_REGISTERACCESS_H

	#define	ACCESS_NONE            0
	#define ACCESS_READ            1
	#define ACCESS_WRITE           2
	#define ACCESS_READ_AND_WRITE  3
	#define ACCESS_READ_OR_WRITE   7

	#define IS_READABLE(x)  (x & ACCESS_READ)
	#define IS_WRITEABLE(x) (x & ACCESS_WRITE)

#endif /* TMC_HELPERS_REGISTERACCESS_H */
