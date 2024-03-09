#ifndef OPTICFLOW_MODULE_H
#define OPTICFLOW_MODULE_H

// Include opticflow calculator
#include "opticflow/opticflow_calculator.h"

// Needed for settings
extern struct opticflow_t opticflow[];

// Module functions
extern void opticflow_module_init(void);
extern void opticflow_module_run(void);
extern void opticflow_module_start(void);
extern void opticflow_module_stop(void);

#endif /* OPTICFLOW_MODULE_H */