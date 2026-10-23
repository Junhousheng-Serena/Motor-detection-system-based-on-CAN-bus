#ifndef USER_CAN_PROTOCOL_H
#define USER_CAN_PROTOCOL_H

/* Protocol v2: classic CAN, standard IDs, DLC=8, little-endian integers.
 * There is no speed sensor: commands specify PWM duty, never RPM. */
#include <stdint.h>
#include <stdbool.h>

#define CAN_PROTOCOL_VERSION       2U
#define CAN_PROTOCOL_LENGTH        8U
#define CAN_ID_HOST_HEARTBEAT       0x100U
#define CAN_ID_MOTOR_COMMAND        0x201U
#define CAN_ID_MOTOR_TELEMETRY      0x301U
#define CAN_ID_MOTOR_HEARTBEAT      0x302U
#define CAN_ID_MOTOR_FAULT          0x3F1U
#define CAN_COMMAND_PERIOD_MS      50U
#define CAN_TELEMETRY_PERIOD_MS     50U
#define CAN_HEARTBEAT_PERIOD_MS     500U
#define CAN_COMMAND_TIMEOUT_MS     200U
#define CAN_TELEMETRY_TIMEOUT_MS    250U
#define CAN_PEER_TIMEOUT_MS         1500U
#define CAN_STARTUP_TIMEOUT_MS      1500U
#define CAN_CURRENT_LIMIT_MAX_MA    160U
#define CAN_TEMPERATURE_INVALID     INT16_MIN
#define CAN_VALUE_INVALID           UINT16_MAX

#define CAN_STATUS_STATE_MASK       0x07U
#define CAN_STATUS_ENABLED          0x08U
#define CAN_STATUS_COMMAND_SEEN     0x10U
#define CAN_STATUS_HOST_ONLINE      0x20U
#define CAN_STATUS_COMMAND_FRESH    0x40U
#define CAN_STATUS_MASK             0x7FU
#define CAN_HEARTBEAT_RX_DROPPED     0x01U
#define CAN_HEARTBEAT_TX_FAILED      0x02U
#define CAN_HEARTBEAT_COMMAND_SEEN   0x04U
#define CAN_HEARTBEAT_ENABLED        0x08U
#define CAN_HEARTBEAT_FLAGS_MASK     0x0FU

typedef enum {
    CAN_FAULT_OVERCURRENT = 1,
    CAN_FAULT_STALL = 2,       /* Reserved: no reliable detector with current hardware. */
    CAN_FAULT_OVERHEAT = 3,
    CAN_FAULT_ENCODER = 4,     /* Reserved: no encoder is installed. */
    CAN_FAULT_COMMUNICATION = 5,
    CAN_FAULT_SENSOR = 6
} CanFaultCode;

typedef struct {
    uint8_t sequence, enable, dutyPercent;
    uint16_t currentLimitMa;
} CanMotorCommand;

typedef struct {
    uint8_t commandSequence; /* Echo of the last accepted command; check COMMAND_SEEN. */
    uint16_t currentDeciMa;  /* Median display value, not PWM average/RMS. */
    int16_t temperatureDeciC;
    uint8_t appliedDuty, status;
} CanTelemetry;

typedef struct {
    uint8_t sequence, state, flags;
    uint32_t uptimeSeconds;
} CanHeartbeat;

typedef struct {
    uint8_t sequence, code, commandSequence;
    uint16_t peakDeciMa;
    int16_t temperatureDeciC;
} CanFaultEvent;

static inline void CanProtocol_PutU16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
}
static inline uint16_t CanProtocol_GetU16(const uint8_t *p) {
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
static inline int16_t CanProtocol_GetI16(const uint8_t *p) {
    uint16_t v = CanProtocol_GetU16(p);
    return (int16_t)(v <= INT16_MAX ? (int32_t)v : (int32_t)v - 65536);
}
static inline bool CanProtocol_CommandValid(const CanMotorCommand *c) {
    return c->enable <= 1U && c->dutyPercent <= 100U &&
           c->currentLimitMa > 0U && c->currentLimitMa <= CAN_CURRENT_LIMIT_MAX_MA;
}
static inline void CanProtocol_EncodeCommand(const CanMotorCommand *c, uint8_t p[8]) {
    p[0] = CAN_PROTOCOL_VERSION; p[1] = c->sequence;
    p[2] = c->enable; p[3] = c->dutyPercent;
    CanProtocol_PutU16(&p[4], c->currentLimitMa); p[6] = p[7] = 0;
}
static inline bool CanProtocol_DecodeCommand(const uint8_t p[8], CanMotorCommand *c) {
    if (p[0] != CAN_PROTOCOL_VERSION || p[6] || p[7]) return false;
    CanMotorCommand value = {.sequence=p[1], .enable=p[2], .dutyPercent=p[3],
                             .currentLimitMa=CanProtocol_GetU16(&p[4])};
    if (!CanProtocol_CommandValid(&value)) return false;
    *c = value; return true;
}
static inline bool CanProtocol_SequenceNewer(uint8_t incoming, uint8_t previous) {
    uint8_t distance = (uint8_t)(incoming - previous);
    return distance != 0U && distance < 128U;
}
static inline void CanProtocol_EncodeTelemetry(const CanTelemetry *t, uint8_t p[8]) {
    p[0] = CAN_PROTOCOL_VERSION; p[1] = t->commandSequence;
    CanProtocol_PutU16(&p[2], t->currentDeciMa);
    CanProtocol_PutU16(&p[4], (uint16_t)t->temperatureDeciC);
    p[6] = t->appliedDuty; p[7] = t->status;
}
static inline bool CanProtocol_DecodeTelemetry(const uint8_t p[8], CanTelemetry *t) {
    if (p[0] != CAN_PROTOCOL_VERSION || p[6] > 100U ||
        (p[7] & ~CAN_STATUS_MASK) || (p[7] & CAN_STATUS_STATE_MASK) > 5U) return false;
    t->commandSequence=p[1]; t->currentDeciMa=CanProtocol_GetU16(&p[2]);
    t->temperatureDeciC=CanProtocol_GetI16(&p[4]);
    t->appliedDuty=p[6]; t->status=p[7]; return true;
}
static inline void CanProtocol_EncodeHeartbeat(const CanHeartbeat *h, uint8_t p[8]) {
    p[0]=CAN_PROTOCOL_VERSION; p[1]=h->sequence; p[2]=h->state; p[3]=h->flags;
    for (unsigned i=0; i<4; i++) p[4+i]=(uint8_t)(h->uptimeSeconds >> (8U*i));
}
static inline bool CanProtocol_DecodeHeartbeat(const uint8_t p[8], CanHeartbeat *h) {
    if (p[0] != CAN_PROTOCOL_VERSION || p[2] > 5U ||
        (p[3] & ~CAN_HEARTBEAT_FLAGS_MASK)) return false;
    h->sequence=p[1]; h->state=p[2]; h->flags=p[3]; h->uptimeSeconds=0;
    for (unsigned i=0; i<4; i++) h->uptimeSeconds |= (uint32_t)p[4+i] << (8U*i);
    return true;
}
static inline void CanProtocol_EncodeFault(const CanFaultEvent *f, uint8_t p[8]) {
    p[0]=CAN_PROTOCOL_VERSION; p[1]=f->sequence; p[2]=f->code; p[3]=f->commandSequence;
    CanProtocol_PutU16(&p[4], f->peakDeciMa);
    CanProtocol_PutU16(&p[6], (uint16_t)f->temperatureDeciC);
}
static inline bool CanProtocol_DecodeFault(const uint8_t p[8], CanFaultEvent *f) {
    if (p[0] != CAN_PROTOCOL_VERSION || p[2] < CAN_FAULT_OVERCURRENT ||
        p[2] > CAN_FAULT_SENSOR) return false;
    f->sequence=p[1]; f->code=p[2]; f->commandSequence=p[3];
    f->peakDeciMa=CanProtocol_GetU16(&p[4]); f->temperatureDeciC=CanProtocol_GetI16(&p[6]);
    return true;
}
#endif /* USER_CAN_PROTOCOL_H */
