#include "canMppt.h"
#include "mppt.h"

CANMPPT::CANMPPT(CAN_TypeDef* canPort, CAN_PINS pins, int frequency /*= DEFAULT_CAN_FREQ*/) : CANManager(canPort, pins, frequency) {
    this->num_messages = 0;
};

void CANMPPT::readHandler(CAN_message_t msg) {
    this->num_messages++;
    uint8_t* data = msg.buf;
    switch (msg.id) {
        case 0x050:
            setCapDischarge(*data);
            break;
        case 0x051:
            clearOVFaultReset(*data);
            break;
        case 0x101:
            packCurrent = ((data[0] << 8) + data[1]) * 0.1;
            packSOC = (float)(data[4]) / 2;
            break;
        case 0x103:
            packChargeCurrentLimit = (float)(*(uint16_t*)data) * CONST_CURR_SAFETY_MULT;
            break;
        default:
            break;
    }
}

void CANMPPT::sendMPPTData() {
    this->sendMessage(0x400, (void*)&boostEnabled, sizeof(boostEnabled));
    this->sendMessage(0x401, (void*)&chargeMode, sizeof(ChargeMode));
    // For each array, offset by 3*i since sending 3 fields voltage, current, temperature
    for (int i = 0; i < NUM_ARRAYS; i++) {
        this->sendMessage(0x402 + 5*i, (void*)&(arrayData[i].voltage), sizeof(float));
        this->sendMessage(0x403 + 5*i, (void*)&(arrayData[i].current), sizeof(float));
        this->sendMessage(0x404 + 5*i, (void*)&(arrayData[i].temp), sizeof(float));
        this->sendMessage(0x405 + 5*i, (void*)&(arrayData[i].dutyCycle), sizeof(float));
        if (chargeMode == ChargeMode::MPPT) {
            this->sendMessage(0x406 + 5*i, (void*)&(targetVoltage[i]), sizeof(float));
        } else {
            float temp = -1;
            this->sendMessage(0x406 + 5*i, (void*)&temp, sizeof(float));
            //this->sendMessage(0x406 + 5*i, (void*)&(targetVoltage_C[i]), sizeof(float));
        }
    }
}