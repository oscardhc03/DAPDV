#ifndef HMI_H_
#define HMI_H_

#include <stdint.h>


typedef enum _feedback_priority_t {
    FEEDBACK_PRIORITY_LOW,
    FEEDBACK_PRIORITY_NORMAL,
    FEEDBACK_PRIORITY_HIGH,
    FEEDBACK_PRIORITY_MAX,
} feedback_priority_t;

typedef enum _feedback_event_id_t {
    FEEDBACK_EVENT_NONE,
    FEEDBACK_EVENT_STRAIGHT,
    FEEDBACK_EVENT_LEFT,
    FEEDBACK_EVENT_RIGHT,
    FEEDBACK_EVENT_U_TURN,
    FEEDBACK_EVENT_STOP,
    FEEDBACK_EVENT_SENSOR_ERROR,
    FEEDBACK_EVENT_LOW_BATTERY,
    FEEDBACK_EVENT_PWR_SAVE_ENTER,
    FEEDBACK_EVENT_PWR_SAVE_EXIT,
    FEEDBACK_EVENT_MAX,
} feedback_event_id_t;

typedef enum _feedback_source_t {
    FEEDBACK_SOURCE_PROXIMITY,
    FEEDBACK_SOURCE_NAVIGATION,
    FEEDBACK_SOURCE_OBJECT_DETECT,
    FEEDBACK_SOURCE_MAX,
} feedback_source_t;

typedef struct _feedback_event_t {
    feedback_event_id_t id;
    feedback_source_t source;
    feedback_priority_t priority;
} feedback_event_t;

#endif /* HMI_H_ */
