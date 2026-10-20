#ifndef USER_CAN_PROTOCOL_H
#define USER_CAN_PROTOCOL_H

/* Shared with the H743 peer. Classic CAN, 500 kbit/s, standard IDs, 8 bytes.
 * All multi-byte values are little-endian. No C struct is sent on the wire. */
#include <stdint.h>
#include <stdbool.h>

#define CAN_PROTOCOL_VERSION       1U
#define CAN_PROTOCOL_LENGTH        8U
#define CAN_ID_HOST_HEARTBEAT       0x101U
#define CAN_ID_MOTOR_MEASUREMENTS   0x201U
#define CAN_ID_MOTOR_STATUS         0x202U
#define CAN_TELEMETRY_PERIOD_MS     200U
#define CAN_HEARTBEAT_PERIOD_MS     500U
#define CAN_PEER_TIMEOUT_MS         1500U

#define CAN_TEMPERATURE_INVALID    INT16_MIN
#define CAN_VALUE_INVALID          UINT16_MAX
#define CAN_LINK_HOST_ONLINE        0x01U
#define CAN_LINK_RX_DROPPED         0x02U
#define CAN_LINK_TX_FAILED          0x04U
#define CAN_LINK_FLAGS_MASK         0x07U

typedef struct
{
    uint8_t sequence;
    uint8_t protection; /* 0 waiting, 1 ready, 2 overheat, 3 sensor fault, 4 overcurrent. */
    int16_t temperatureDeciC;
    uint16_t currentDeciMa; /* Median, not average/RMS current. */
    uint16_t peakDeciMa;    /* Peak among the 15 captured samples. */
} CanMeasurements;

typedef struct
{
    uint8_t sequence;
    uint16_t voltageMv;
    uint8_t appliedDuty;
    uint8_t requestedDuty;
    uint8_t linkFlags;
} CanMotorStatus;

static inline void CanProtocol_PutU16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
}

static inline uint16_t CanProtocol_GetU16(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static inline void CanProtocol_EncodeMeasurements(const CanMeasurements *sample,
                                                   uint8_t data[8])
{
    data[0] = (uint8_t)((CAN_PROTOCOL_VERSION << 4) | sample->protection);
    data[1] = sample->sequence;
    CanProtocol_PutU16(&data[2], (uint16_t)sample->temperatureDeciC);
    CanProtocol_PutU16(&data[4], sample->currentDeciMa);
    CanProtocol_PutU16(&data[6], sample->peakDeciMa);
}

static inline bool CanProtocol_DecodeMeasurements(const uint8_t data[8],
                                                   CanMeasurements *sample)
{
    if ((data[0] >> 4) != CAN_PROTOCOL_VERSION || (data[0] & 0x0FU) > 4U)
    {
        return false;
    }
    uint16_t rawTemperature = CanProtocol_GetU16(&data[2]);
    sample->sequence = data[1];
    sample->protection = data[0] & 0x0FU;
    sample->temperatureDeciC = (int16_t)(rawTemperature <= INT16_MAX ?
        (int32_t)rawTemperature : (int32_t)rawTemperature - 65536);
    sample->currentDeciMa = CanProtocol_GetU16(&data[4]);
    sample->peakDeciMa = CanProtocol_GetU16(&data[6]);
    return true;
}

static inline void CanProtocol_EncodeStatus(const CanMotorStatus *status,
                                             uint8_t data[8])
{
    data[0] = CAN_PROTOCOL_VERSION;
    data[1] = status->sequence;
    CanProtocol_PutU16(&data[2], status->voltageMv);
    data[4] = status->appliedDuty;
    data[5] = status->requestedDuty;
    data[6] = status->linkFlags;
    data[7] = 0U;
}

static inline bool CanProtocol_DecodeStatus(const uint8_t data[8],
                                             CanMotorStatus *status)
{
    if (data[0] != CAN_PROTOCOL_VERSION || data[4] > 100U || data[5] > 100U ||
        (data[6] & ~CAN_LINK_FLAGS_MASK) != 0U || data[7] != 0U)
    {
        return false;
    }
    status->sequence = data[1];
    status->voltageMv = CanProtocol_GetU16(&data[2]);
    status->appliedDuty = data[4];
    status->requestedDuty = data[5];
    status->linkFlags = data[6];
    return true;
}

static inline void CanProtocol_EncodeHeartbeat(uint8_t sequence, uint32_t uptime,
                                                uint8_t data[8])
{
    data[0] = CAN_PROTOCOL_VERSION;
    data[1] = sequence;
    data[2] = data[3] = 0U;
    for (unsigned i = 0; i < 4; i++) data[4 + i] = (uint8_t)(uptime >> (8U * i));
}

static inline bool CanProtocol_IsHeartbeat(const uint8_t data[8])
{
    return data[0] == CAN_PROTOCOL_VERSION && data[2] == 0U && data[3] == 0U;
}

#endif /* USER_CAN_PROTOCOL_H */
