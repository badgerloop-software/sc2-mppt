#include "can_mppt.h"

#include "can_ids.h"
#include "mppt.h"

// ------------- PUBLIC FUNCTIONS -------------

CanMppt::CanMppt(CAN_TypeDef* canPort, CAN_PINS pins, int frequency)
    : CANManager(canPort, pins, frequency) {}

void CanMppt::readHandler(CAN_message_t msg) {
    uint8_t* data = msg.buf;
    switch (msg.id) {
        case CAN_MPPT_CAP_DISCHARGE:
            setCapDischarge(*data);
            break;
        case CAN_MPPT_OV_RESET:
            clearOVFaultReset(*data);
            break;
        case CAN_BMS_PACK:
            // bytes 0-1: current * 0.1 A, byte 4: SOC * 0.5 %
            packCurrent = ((data[0] << 8) + data[1]) * 0.1f;
            packSOC = (float)(data[4]) / 2.0f;
            break;
        case CAN_BMS_CHARGE_LIMIT:
            packChargeCurrentLimit = (float)(*(uint16_t*)data) * CONST_CURR_SAFETY_MULT;
            break;
        default:
            break;
    }
}

void CanMppt::sendMpptData() {
    sendMessage(CAN_MPPT_BOOST, (void*)&boostEnabled, sizeof(boostEnabled));
    sendMessage(CAN_MPPT_MODE, (void*)&chargeMode, sizeof(ChargeMode));

    for (int i = 0; i < NUM_ARRAYS; i++) {
        sendMessage(CAN_MPPT_STRING(i, CAN_MPPT_FIELD_V), (void*)&(arrayData[i].voltage),
                    sizeof(float));
        sendMessage(CAN_MPPT_STRING(i, CAN_MPPT_FIELD_I), (void*)&(arrayData[i].current),
                    sizeof(float));
        sendMessage(CAN_MPPT_STRING(i, CAN_MPPT_FIELD_TEMP), (void*)&(arrayData[i].temp),
                    sizeof(float));
        sendMessage(CAN_MPPT_STRING(i, CAN_MPPT_FIELD_DUTY),
                    (void*)&(arrayData[i].dutyCycle), sizeof(float));
        if (chargeMode == ChargeMode::MPPT) {
            sendMessage(CAN_MPPT_STRING(i, CAN_MPPT_FIELD_TARGET),
                        (void*)&(targetVoltage[i]), sizeof(float));
        } else {
            // no target in const current mode
            float none = -1.0f;
            sendMessage(CAN_MPPT_STRING(i, CAN_MPPT_FIELD_TARGET), (void*)&none,
                        sizeof(float));
        }
    }
}
