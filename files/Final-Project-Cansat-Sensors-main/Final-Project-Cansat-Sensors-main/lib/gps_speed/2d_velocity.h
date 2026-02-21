#ifndef GPS_VELOCITY_H_
#define GPS_VELOCITY_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Get 2-D ground velocity from the last NMEA data.
 *
 * Uses $GPRMC to get speed over ground (knots) and course over ground (deg),
 * converts to North/East components in m/s.
 *
 * v_north > 0 : moving north
 * v_north < 0 : moving south
 * v_east  > 0 : moving east
 * v_east  < 0 : moving west
 *
 * @param v_north  pointer to float for North component (m/s)
 * @param v_east   pointer to float for East component (m/s)
 * @return         true if parsed successfully, false otherwise
 */
bool gps_get_ground_velocity_ms(float *v_north, float *v_east);

#ifdef __cplusplus
}
#endif

#endif /* GPS_VELOCITY_H_ */
