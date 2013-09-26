#ifndef __c1_asbswarm_lib_h__
#define __c1_asbswarm_lib_h__

/* Include files */
#include "sfc_sf.h"
#include "sfc_mex.h"
#include "rtwtypes.h"

/* Type Definitions */
typedef struct {
  int32_T c1_sfEvent;
  boolean_T c1_isStable;
  boolean_T c1_doneDoubleBufferReInit;
  uint8_T c1_is_active_c1_asbswarm_lib;
  real_T c1_dim;
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
} SFc1_asbswarm_libInstanceStruct;

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c1_asbswarm_lib_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c1_asbswarm_lib_get_check_sum(mxArray *plhs[]);
extern void c1_asbswarm_lib_method_dispatcher(SimStruct *S, int_T method, void
  *data);

#endif
