#ifndef __CAN_MPPT_H__
#define __CAN_MPPT_H__

#include "canmanager.h"
#include "io_management.h"

// ------------- CLASS -------------

// CAN for this board (IDs in can_ids.h)
class CanMppt : public CANManager {
   public:
    CanMppt(CAN_TypeDef* canPort, CAN_PINS pins, int frequency = DEFAULT_CAN_FREQ);
    void readHandler(CAN_message_t msg) override;
    void sendMpptData();
};

#endif  // __CAN_MPPT_H__
