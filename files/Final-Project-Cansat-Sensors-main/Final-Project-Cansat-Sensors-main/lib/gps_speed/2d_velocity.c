#include <string.h>
#include <stdio.h>
#include <math.h>

#include "esp_log.h"
#include "neo6m.h"
#include "2d_velocity.h"

static const char *TAG_VEL = "GPS_VEL";

// 1 knot ≈ 0.514444 m/s
#define KNOT_TO_MS 0.514444f
#define DEG_TO_RAD(deg) ((deg) * (3.14159265f / 180.0f))

bool gps_get_ground_velocity_ms(float *v_north, float *v_east)
{
    if (v_north == NULL || v_east == NULL) {
        return false;
    }

    const char *nmea = gps_get_last_sentence();

    if (nmea == NULL || nmea[0] == '\0') {
        ESP_LOGW(TAG_VEL, "No NMEA data available");
        return false;
    }

    // Find the start of a GPRMC sentence in the buffer
    const char *start = strstr(nmea, "$GPRMC");
    if (start == NULL) {
        ESP_LOGW(TAG_VEL, "No GPRMC sentence found in buffer");
        return false;
    }

    float speed_knots = 0.0f;
    float course_deg  = 0.0f;

    /*
     * GPRMC (simplified):
     * $GPRMC,time,status,lat,N,lon,E,speed_knots,course_deg,date,...
     *
     * Pattern explanation:
     *  %*[^,]  -> read until comma, discard (time)
     *  %*c     -> one char, discard (status)
     *  %*[^,]  -> latitude, discard
     *  %*c     -> N/S, discard
     *  %*[^,]  -> longitude, discard
     *  %*c     -> E/W, discard
     *  %f      -> speed_knots
     *  %f      -> course_deg
     */
    int matched = sscanf(
        start,
        "$GPRMC,%*[^,],%*c,%*[^,],%*c,%*[^,],%*c,%f,%f",
        &speed_knots,
        &course_deg
    );

    if (matched != 2) {
        ESP_LOGW(TAG_VEL, "Failed to parse speed/course from GPRMC");
        return false;
    }

    // Convert speed to m/s
    float speed_ms = speed_knots * KNOT_TO_MS;

    // Convert course to radians
    float theta = DEG_TO_RAD(course_deg);

    // 0° = North, 90° = East -> use cos for North, sin for East
    *v_north = speed_ms * cosf(theta);
    *v_east  = speed_ms * sinf(theta);

    return true;
}
