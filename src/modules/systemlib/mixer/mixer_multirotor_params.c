#include "../param/param.h"

// 16 is max name length

/**
 * Use force = thrust coefficient * rpm^2 model
 *
 * Set to 1 to use, set to 0 to ignore
 *
 * @min 0
 * @max 1
 * @group Vehicle dynamics model
 */
PARAM_DEFINE_INT32(MDL_USE_PHYSICS, 0);

/**
 * Thrust coefficient
 *
 * @min 0
 * @max 1
 * @group Vehicle dynamics model
 * @group MDL Outputs
 */
PARAM_DEFINE_FLOAT(MDL_CT, 1.0f);

/**
* Maximum RPM
*
* @min 0
* @group Vehicle dynamics model
*/
PARAM_DEFINE_FLOAT(MDL_MAX_RPM, 0.0f);
