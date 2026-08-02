#ifndef __CAN_MPPT_H__
#define __CAN_MPPT_H__

#include "canmanager.h"
#include "io_management.h"

// ------------- CLASS -------------

// IDs / DLCs from embedded-pio/can_ids.h (SC2_CAN_*)
class CanMppt : public CANManager {
   public:
    CanMppt(CAN_TypeDef* canPort, CAN_PINS pins, int frequency = DEFAULT_CAN_FREQ);
    void readHandler(CAN_message_t msg) override;
    void sendMpptData();
};

#endif  // __CAN_MPPT_H__
