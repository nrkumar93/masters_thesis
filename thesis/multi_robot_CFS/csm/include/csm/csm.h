#ifndef H_CSM_H
#define H_CSM_H

/* Some preprocessor magic for calling this library from C++ */

#ifdef __cplusplus
	namespace CSM {}
	extern "C" {
#endif

#include "csm/laser_data.h"
#include "csm/laser_data_drawing.h"
#include "csm/laser_data_json.h"
#include "csm/algos.h"
#include "csm/utils.h"

#ifdef __cplusplus
	}
#endif

#endif
