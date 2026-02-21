#ifndef NEO6M_H_
#define NEO6M_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void gps_start(void);
void raw_nmea(void);

// NEW: give access to the last raw NMEA buffer read by raw_nmea()
const char *gps_get_last_sentence(void);

#ifdef __cplusplus
}
#endif

#endif /* NEO6M_H_ */
