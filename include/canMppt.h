#ifndef __CAN_MPPT_H__
#define __CAN_MPPT_H__

#include "canmanager.h"
#include "const.h"
#include "IOManagement.h"


class CANMPPT : public CANManager {
    public:
        CANMPPT(CAN_TypeDef* canPort, CAN_PINS pins, int frequency = DEFAULT_CAN_FREQ);
        void readHandler(CAN_message_t msg);
        void sendMPPTData();
};

#endif // __CAN_MPPT_H__