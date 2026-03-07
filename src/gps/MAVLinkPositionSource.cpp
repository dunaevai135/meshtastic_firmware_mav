#include "configuration.h"

#if !MESHTASTIC_EXCLUDE_GPS

#include "MAVLinkPositionSource.h"

#include "GPS.h"
#include "RTC.h"
#include "Throttle.h"

#ifndef GPS_SERIAL_PORT
#define GPS_SERIAL_PORT Serial1
#endif

namespace
{
static constexpr uint32_t DEFAULT_MAVLINK_BAUD = 57600;
static constexpr uint32_t DEFAULT_MAVLINK_STALE_MS = 10000;
static constexpr uint32_t GLOBAL_POSITION_PREFERRED_MS = 2000;
static constexpr uint64_t MIN_UNIX_TIME_USEC = 1500000000000000ULL;
}

#if defined(ARCH_NRF52)
Uart *MAVLinkPositionSource::serialPort = &GPS_SERIAL_PORT;
#elif defined(ARCH_ESP32) || defined(ARCH_PORTDUINO) || defined(ARCH_STM32WL)
HardwareSerial *MAVLinkPositionSource::serialPort = &GPS_SERIAL_PORT;
#elif defined(ARCH_RP2040)
SerialUART *MAVLinkPositionSource::serialPort = &GPS_SERIAL_PORT;
#else
HardwareSerial *MAVLinkPositionSource::serialPort = nullptr;
#endif

std::unique_ptr<MAVLinkPositionSource> mavlinkPositionSource = nullptr;

bool MAVLinkPositionSource::begin()
{
    const int32_t rx = resolveRxGpio();
    const int32_t tx = resolveTxGpio();
    const uint32_t baud = resolveBaudRate();

    LOG_INFO("POSSRC[MAV]: init uart=%d baud=%u rx=%d", 1, baud, rx);

    if (!serialPort || rx < 0) {
        LOG_ERROR("POSSRC[MAV]: uart init failed rx=%d", rx);
        return false;
    }

#ifdef ARCH_ESP32
    serialPort->setRxBufferSize(SERIAL_BUFFER_SIZE);
    serialPort->begin(baud, SERIAL_8N1, rx, tx);
#elif defined(ARCH_RP2040)
    if (tx >= 0) {
        serialPort->setPinout(tx, rx);
    }
    serialPort->setFIFOSize(256);
    serialPort->begin(baud);
#elif defined(ARCH_NRF52)
    if (tx >= 0) {
        serialPort->setPins(rx, tx);
    }
    serialPort->begin(baud);
#elif defined(ARCH_STM32WL)
    if (tx >= 0) {
        serialPort->setTx(tx);
    }
    serialPort->setRx(rx);
    serialPort->begin(baud);
#elif defined(ARCH_PORTDUINO)
    serialPort->begin(baud);
#else
#error Unsupported architecture
#endif

    if (!waitingLogged) {
        LOG_INFO("POSSRC[MAV]: waiting for data");
        waitingLogged = true;
    }

    return true;
}

void MAVLinkPositionSource::poll()
{
    if (!serialPort) {
        return;
    }

    while (serialPort->available() > 0) {
        const int c = serialPort->read();
        mavlink_message_t message;

        if (mavlink_parse_char(MAVLINK_COMM_0, c, &message, &parserStatus)) {
            if (!firstPacketLogged) {
                LOG_INFO("POSSRC[MAV]: first packet sys=%u comp=%u msgid=%u", message.sysid, message.compid, message.msgid);
                firstPacketLogged = true;
            }

            if (!shouldAccept(message.sysid, message.compid)) {
                continue;
            }

            meshtastic_Position position = lastPosition;
            bool isValid = false;

            if (message.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
                isValid = decodeGlobalPosition(message, position);
                if (isValid) {
                    lastGlobalPositionMs = millis();
                }
            } else if (message.msgid == MAVLINK_MSG_ID_GPS_RAW_INT) {
                const bool primaryFresh = lastGlobalPositionMs > 0 &&
                                          Throttle::isWithinTimespanMs(lastGlobalPositionMs, GLOBAL_POSITION_PREFERRED_MS);
                if (!primaryFresh) {
                    isValid = decodeGpsRaw(message, position);
                }
            }

            if (isValid) {
                lastAcceptedPacketMs = millis();
                hasAcceptedPacket = true;
                staleLogged = false;
                lastPosition = position;
                publishStatus(true);

                LOG_INFO("POSSRC[MAV]: valid fix lat=%f lon=%f alt=%.3f", position.latitude_i * 1e-7,
                         position.longitude_i * 1e-7, position.altitude / 1000.0f);
            }
        } else if (parserStatus.parse_error != lastParseErrorCount) {
            lastParseErrorCount = parserStatus.parse_error;
            LOG_WARN("POSSRC[MAV]: parser error");
        }
    }

    markStaleIfNeeded();
}

void MAVLinkPositionSource::publishStatus(bool hasLock)
{
    meshtastic::GPSStatus status(hasLock, hasAcceptedPacket, false, lastPosition);
    newStatus.notifyObservers(&status);
}

bool MAVLinkPositionSource::shouldAccept(uint8_t sysid, uint8_t compid)
{
    const uint32_t expectedSysid = config.position.mavlink_sysid;
    const uint32_t expectedCompid = config.position.mavlink_compid;

    if ((expectedSysid != 0 && expectedSysid != sysid) || (expectedCompid != 0 && expectedCompid != compid)) {
        if (!Throttle::isWithinTimespanMs(lastRejectedLogMs, 5000)) {
            lastRejectedLogMs = millis();
            LOG_INFO("POSSRC[MAV]: rejected sys=%u comp=%u", sysid, compid);
        }
        return false;
    }

    return true;
}

bool MAVLinkPositionSource::decodeGlobalPosition(const mavlink_message_t &message, meshtastic_Position &position)
{
    mavlink_global_position_int_t packet;
    mavlink_msg_global_position_int_decode(&message, &packet);

    if (packet.lat == 0 && packet.lon == 0) {
        return false;
    }

    if (packet.lat < -900000000 || packet.lat > 900000000 || packet.lon < -1800000000 || packet.lon > 1800000000) {
        return false;
    }

    position.location_source = meshtastic_Position_LocSource_LOC_EXTERNAL;
    position.latitude_i = packet.lat;
    position.longitude_i = packet.lon;
    position.altitude = packet.alt;
    position.has_altitude = true;

    if (packet.hdg != UINT16_MAX) {
        position.ground_track = packet.hdg * 1000U;
        position.has_ground_track = true;
    }

    position.timestamp = 0;
    return true;
}

bool MAVLinkPositionSource::decodeGpsRaw(const mavlink_message_t &message, meshtastic_Position &position)
{
    mavlink_gps_raw_int_t packet;
    mavlink_msg_gps_raw_int_decode(&message, &packet);

    if (packet.fix_type < 2) {
        return false;
    }

    if (packet.lat == 0 && packet.lon == 0) {
        return false;
    }

    if (packet.lat < -900000000 || packet.lat > 900000000 || packet.lon < -1800000000 || packet.lon > 1800000000) {
        return false;
    }

    position.location_source = meshtastic_Position_LocSource_LOC_EXTERNAL;
    position.latitude_i = packet.lat;
    position.longitude_i = packet.lon;
    position.altitude = packet.alt;
    position.has_altitude = true;
    position.fix_type = packet.fix_type;
    position.fix_quality = packet.fix_type;
    position.sats_in_view = packet.satellites_visible == UINT8_MAX ? 0 : packet.satellites_visible;

    uint32_t timeSec = 0;
    if (decodeTime(packet.time_usec, timeSec)) {
        position.time = timeSec;
        position.timestamp = timeSec;

        struct timeval tv = {};
        tv.tv_sec = timeSec;
        tv.tv_usec = packet.time_usec % 1000000ULL;
        perhapsSetRTC(RTCQualityGPS, &tv);
    }

    return true;
}

bool MAVLinkPositionSource::decodeTime(uint64_t timeUsec, uint32_t &timeSec) const
{
    if (timeUsec < MIN_UNIX_TIME_USEC) {
        return false;
    }

    timeSec = timeUsec / 1000000ULL;
    return true;
}

void MAVLinkPositionSource::markStaleIfNeeded()
{
    if (!hasAcceptedPacket || lastAcceptedPacketMs == 0) {
        return;
    }

    const uint32_t ageMs = millis() - lastAcceptedPacketMs;
    if (ageMs < staleTimeoutMs()) {
        return;
    }

    if (!staleLogged) {
        LOG_INFO("POSSRC[MAV]: stale age_ms=%u", ageMs);
        staleLogged = true;
        publishStatus(false);
    }
}

int32_t MAVLinkPositionSource::resolveRxGpio() const
{
    if (config.position.rx_gpio > 0) {
        return config.position.rx_gpio;
    }
#if defined(GPS_RX_PIN)
    return GPS_RX_PIN;
#elif defined(PIN_SERIAL1_RX)
    return PIN_SERIAL1_RX;
#else
    return -1;
#endif
}

int32_t MAVLinkPositionSource::resolveTxGpio() const
{
    if (config.position.tx_gpio > 0) {
        return config.position.tx_gpio;
    }
#ifdef ARCH_ESP32
    return -1;
#elif defined(GPS_TX_PIN)
    return GPS_TX_PIN;
#elif defined(PIN_SERIAL1_TX)
    return PIN_SERIAL1_TX;
#else
    return -1;
#endif
}

uint32_t MAVLinkPositionSource::resolveBaudRate() const
{
    return config.position.mavlink_uart_baud ? config.position.mavlink_uart_baud : DEFAULT_MAVLINK_BAUD;
}

uint32_t MAVLinkPositionSource::staleTimeoutMs() const
{
    return config.position.mavlink_stale_secs ? config.position.mavlink_stale_secs * 1000U : DEFAULT_MAVLINK_STALE_MS;
}

#endif
