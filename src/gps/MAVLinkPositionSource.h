#pragma once

#include "configuration.h"

#if !MESHTASTIC_EXCLUDE_GPS

#include <memory>

#include "GPSStatus.h"
#include "Observer.h"

#if defined(ARCH_RP2040)
#include <SerialUART.h>
#elif defined(ARCH_NRF52)
#include <Uart.h>
#else
#include <HardwareSerial.h>
#endif

extern "C" {
#include <common/mavlink.h>
}

class MAVLinkPositionSource
{
  public:
    Observable<const meshtastic::GPSStatus *> newStatus;

    bool begin();
    void poll();

    bool isConnected() const { return hasAcceptedPacket; }

  private:
    void publishStatus(bool hasLock);
    bool shouldAccept(uint8_t sysid, uint8_t compid);
    bool decodeGlobalPosition(const mavlink_message_t &message, meshtastic_Position &position);
    bool decodeGpsRaw(const mavlink_message_t &message, meshtastic_Position &position);
    bool decodeTime(uint64_t timeUsec, uint32_t &timeSec) const;
    void markStaleIfNeeded();
    int32_t resolveRxGpio() const;
    int32_t resolveTxGpio() const;
    uint32_t resolveBaudRate() const;
    uint32_t staleTimeoutMs() const;

    meshtastic_Position lastPosition = meshtastic_Position_init_default;
    mavlink_status_t parserStatus = {};

    uint32_t lastAcceptedPacketMs = 0;
    uint32_t lastGlobalPositionMs = 0;
    uint16_t lastParseErrorCount = 0;
    uint32_t lastRejectedLogMs = 0;

    bool waitingLogged = false;
    bool firstPacketLogged = false;
    bool hasAcceptedPacket = false;
    bool staleLogged = false;

#if defined(ARCH_RP2040)
    static SerialUART *serialPort;
#elif defined(ARCH_NRF52)
    static Uart *serialPort;
#else
    static HardwareSerial *serialPort;
#endif
};

extern std::unique_ptr<MAVLinkPositionSource> mavlinkPositionSource;

#endif
