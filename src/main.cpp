#include <Arduino.h>
#include <WiFi.h>
#include "secrets.h"

// ============================================================
// DNP3 Configuration
// ============================================================

constexpr uint8_t BI0_PIN = 34;
constexpr uint8_t BI1_PIN = 35;
constexpr uint8_t AI0_PIN = 32;

struct DNP3Database
{
    bool bi[2];
    int32_t ai[1];
};

DNP3Database database;

constexpr uint16_t DNP3_PORT = 20000;

constexpr uint16_t OUTSTATION_ADDRESS = 4;
constexpr uint16_t MASTER_ADDRESS     = 3;

WiFiServer dnp3Server(DNP3_PORT);

// ============================================================
// DNP3 Constants
// ============================================================

constexpr uint8_t DNP3_START_1 = 0x05;
constexpr uint8_t DNP3_START_2 = 0x64;

// Application function codes
constexpr uint8_t APP_CONFIRM              = 0x00;
constexpr uint8_t APP_READ                 = 0x01;
constexpr uint8_t APP_WRITE                = 0x02;
constexpr uint8_t APP_ENABLE_UNSOLICITED   = 0x14;
constexpr uint8_t APP_DISABLE_UNSOLICITED  = 0x15;
constexpr uint8_t APP_RESPONSE             = 0x81;
constexpr uint8_t APP_UNSOLICITED_RESPONSE = 0x82;

// ============================================================
// DNP3 CRC
// Polynomial used by DNP3:
// x^16 + x^13 + x^12 + x^11 + x^10 + x^8 + x^6 + x^5 + x^2 + 1
//
// Reflected implementation polynomial = 0xA6BC
// Initial = 0x0000
// Final complement
// ============================================================

uint16_t dnp3CRC(const uint8_t* data, size_t length)
{
    uint16_t crc = 0x0000;

    for (size_t i = 0; i < length; i++)
    {
        crc ^= data[i];

        for (int bit = 0; bit < 8; bit++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA6BC;
            else
                crc >>= 1;
        }
    }

    return ~crc;
}

// ============================================================
// Debug Helpers
// ============================================================

void updateDatabase()
{
    database.bi[0] = digitalRead(BI0_PIN);
    database.bi[1] = digitalRead(BI1_PIN);

    database.ai[0] = analogRead(AI0_PIN);
}

void printHex(const uint8_t* data, size_t length)
{
    for (size_t i = 0; i < length; i++)
    {
        if (data[i] < 0x10)
            Serial.print("0");

        Serial.print(data[i], HEX);
        Serial.print(" ");
    }

    Serial.println();
}

// ============================================================
// CRC Validation
// ============================================================

bool validateCRC(
    const uint8_t* data,
    size_t length,
    uint16_t receivedCRC)
{
    uint16_t calculated = dnp3CRC(data, length);

    return calculated == receivedCRC;
}

// ============================================================
// Link Header
// ============================================================

struct DNP3LinkHeader
{
    uint8_t length;
    uint8_t control;

    uint16_t destination;
    uint16_t source;

    uint8_t functionCode;
};

bool parseLinkHeader(
    const uint8_t* frame,
    size_t frameLength,
    DNP3LinkHeader& header)
{
    if (frameLength < 10)
        return false;

    if (frame[0] != 0x05 || frame[1] != 0x64)
        return false;

    header.length  = frame[2];
    header.control = frame[3];

    header.destination =
        frame[4] |
        (static_cast<uint16_t>(frame[5]) << 8);

    header.source =
        frame[6] |
        (static_cast<uint16_t>(frame[7]) << 8);

    header.functionCode =
        header.control & 0x0F;

    // Header CRC covers bytes 0-7.
    uint16_t receivedCRC =
        frame[8] |
        (static_cast<uint16_t>(frame[9]) << 8);

    if (!validateCRC(frame, 8, receivedCRC))
    {
        Serial.println("BAD HEADER CRC");
        return false;
    }

    return true;
}

// ============================================================
// Extract DNP3 User Data
//
// DNP3 inserts a CRC after every 16 bytes of user data.
// This removes those CRC bytes.
// ============================================================

size_t extractUserData(
    const uint8_t* frame,
    size_t frameLength,
    uint8_t* output,
    size_t outputCapacity)
{
    if (frameLength < 10)
        return 0;

    uint8_t dnpLength = frame[2];

    // DNP3 LENGTH counts:
    //
    // control       = 1
    // destination   = 2
    // source        = 2
    //
    // Therefore user data = LENGTH - 5

    if (dnpLength < 5)
        return 0;

    size_t userDataLength = dnpLength - 5;

    if (userDataLength > outputCapacity)
        return 0;

    size_t framePosition = 10;
    size_t outputPosition = 0;

    while (outputPosition < userDataLength)
    {
        size_t remaining =
            userDataLength - outputPosition;

        size_t blockLength =
            (remaining > 16) ? 16 : remaining;

        // Make sure data + CRC exist.
        if (framePosition + blockLength + 2 > frameLength)
        {
            Serial.println("Incomplete DNP3 data block");
            return 0;
        }

        uint16_t receivedCRC =
            frame[framePosition + blockLength] |
            (static_cast<uint16_t>(
                frame[framePosition + blockLength + 1]) << 8);

        if (!validateCRC(
                &frame[framePosition],
                blockLength,
                receivedCRC))
        {
            Serial.println("BAD DATA CRC");
            return 0;
        }

        memcpy(
            &output[outputPosition],
            &frame[framePosition],
            blockLength);

        outputPosition += blockLength;

        framePosition += blockLength + 2;
    }

    return outputPosition;
}

// ============================================================
// Send DNP3 Link Frame
// ============================================================

void sendDNP3Frame(
    WiFiClient& client,
    const uint8_t* userData,
    size_t userDataLength)
{
    uint8_t frame[300];

    size_t pos = 0;

    // --------------------------------------------------------
    // Link header
    // --------------------------------------------------------

    frame[pos++] = 0x05;
    frame[pos++] = 0x64;

    // Length:
    // control + dest + source + user data
    frame[pos++] =
        static_cast<uint8_t>(5 + userDataLength);

    // --------------------------------------------------------
    // Link control
    //
    // DIR = 0   outstation -> master
    // PRM = 1   primary
    // FC  = 4   unconfirmed user data
    //
    // 0100 0100 = 0x44
    // --------------------------------------------------------

    frame[pos++] = 0x44;

    // Destination = master
    frame[pos++] = MASTER_ADDRESS & 0xFF;
    frame[pos++] = (MASTER_ADDRESS >> 8) & 0xFF;

    // Source = outstation
    frame[pos++] = OUTSTATION_ADDRESS & 0xFF;
    frame[pos++] = (OUTSTATION_ADDRESS >> 8) & 0xFF;

    // Header CRC
    uint16_t headerCRC =
        dnp3CRC(frame, 8);

    frame[pos++] = headerCRC & 0xFF;
    frame[pos++] = (headerCRC >> 8) & 0xFF;

    // --------------------------------------------------------
    // User data blocks
    // --------------------------------------------------------

    size_t dataPosition = 0;

    while (dataPosition < userDataLength)
    {
        size_t remaining =
            userDataLength - dataPosition;

        size_t blockLength =
            (remaining > 16) ? 16 : remaining;

        size_t blockStart = pos;

        memcpy(
            &frame[pos],
            &userData[dataPosition],
            blockLength);

        pos += blockLength;

        uint16_t blockCRC =
            dnp3CRC(
                &frame[blockStart],
                blockLength);

        frame[pos++] = blockCRC & 0xFF;
        frame[pos++] = (blockCRC >> 8) & 0xFF;

        dataPosition += blockLength;
    }

    Serial.println();
    Serial.println("TX:");
    printHex(frame, pos);

    client.write(frame, pos);
    client.flush();
}

// ============================================================
// Application Response
// ============================================================

void sendApplicationResponse(
    WiFiClient& client,
    uint8_t sequence)
{
    uint8_t data[16];

    size_t pos = 0;

    // --------------------------------------------------------
    // Transport header
    //
    // FIR = 1
    // FIN = 1
    // SEQ = transport sequence
    // --------------------------------------------------------

    uint8_t transport =
        0xC0 | (sequence & 0x3F);

    data[pos++] = transport;

    // --------------------------------------------------------
    // Application control
    //
    // FIR = 1
    // FIN = 1
    // CON = 0
    // UNS = 0
    // SEQ = application sequence
    // --------------------------------------------------------

    uint8_t applicationControl =
        0xC0 | (sequence & 0x0F);

    data[pos++] = applicationControl;

    // Function code
    data[pos++] = APP_RESPONSE;

    // --------------------------------------------------------
    // IIN
    //
    // Internal Indications = 0
    // --------------------------------------------------------

    data[pos++] = 0x00;
    data[pos++] = 0x00;

    sendDNP3Frame(
        client,
        data,
        pos);
}

void sendIntegrityResponse(
    WiFiClient& client,
    uint8_t applicationSequence)
{
    // Read the current physical ESP32 I/O
    updateDatabase();

    uint8_t data[64];
    size_t pos = 0;

    // ========================================================
    // Transport Header
    // ========================================================

    // FIR = 1
    // FIN = 1
    // Single transport fragment
    data[pos++] =
        0xC0 | (applicationSequence & 0x3F);

    // ========================================================
    // Application Header
    // ========================================================

    // FIR = 1
    // FIN = 1
    // SEQ = same application sequence as request
    data[pos++] =
        0xC0 | (applicationSequence & 0x0F);

    // Function = RESPONSE
    data[pos++] = APP_RESPONSE;   // 0x81

    // IIN1
    data[pos++] = 0x00;

    // IIN2
    data[pos++] = 0x00;


    // ========================================================
    // GROUP 1 VARIATION 2
    // Binary Input With Flags
    // ========================================================

    data[pos++] = 0x01;   // Group 1
    data[pos++] = 0x02;   // Variation 2

    // Qualifier 0x00:
    // 1-byte start and stop indexes
    data[pos++] = 0x00;

    // Start index = 0
    data[pos++] = 0x00;

    // Stop index = 1
    data[pos++] = 0x01;


    // --------------------------------------------------------
    // Binary Input 0
    //
    // bit 0 = ONLINE
    // bit 7 = binary state
    // --------------------------------------------------------

    data[pos++] =
        0x01 |
        (database.bi[0] ? 0x80 : 0x00);


    // --------------------------------------------------------
    // Binary Input 1
    // --------------------------------------------------------

    data[pos++] =
        0x01 |
        (database.bi[1] ? 0x80 : 0x00);


    // ========================================================
    // GROUP 30 VARIATION 1
    // 32-bit Analog Input With Flags
    // ========================================================

    data[pos++] = 30;     // Group 30
    data[pos++] = 1;      // Variation 1

    // 1-byte start/stop indexes
    data[pos++] = 0x00;

    // Start index = 0
    data[pos++] = 0x00;

    // Stop index = 0
    data[pos++] = 0x00;


    // --------------------------------------------------------
    // Analog Input 0 Flags
    //
    // ONLINE = 1
    // --------------------------------------------------------

    data[pos++] = 0x01;


    // --------------------------------------------------------
    // Analog Input 0 Value
    //
    // Group 30 Variation 1:
    // signed 32-bit integer
    // little endian
    // --------------------------------------------------------

    int32_t value = database.ai[0];

    data[pos++] = value & 0xFF;
    data[pos++] = (value >> 8) & 0xFF;
    data[pos++] = (value >> 16) & 0xFF;
    data[pos++] = (value >> 24) & 0xFF;


    // ========================================================
    // Debug
    // ========================================================

    Serial.println();
    Serial.println("Sending Integrity Response");

    Serial.printf(
        "BI0 = %u\n",
        database.bi[0]);

    Serial.printf(
        "BI1 = %u\n",
        database.bi[1]);

    Serial.printf(
        "AI0 = %ld\n",
        static_cast<long>(database.ai[0]));


    // ========================================================
    // Send it
    // ========================================================

    sendDNP3Frame(
        client,
        data,
        pos);
}

// ============================================================
// Process Application Layer
// ============================================================

void processApplication(
    WiFiClient& client,
    const uint8_t* userData,
    size_t length)
{
    // Need:
    // transport + app control + function
    if (length < 3)
        return;

    uint8_t transportControl =
        userData[0];

    uint8_t applicationControl =
        userData[1];

    uint8_t functionCode =
        userData[2];

    uint8_t transportSequence =
        transportControl & 0x3F;

    uint8_t applicationSequence =
        applicationControl & 0x0F;

    Serial.println();
    Serial.println("===== TRANSPORT =====");

    Serial.printf(
        "Transport Control: 0x%02X\n",
        transportControl);

    Serial.printf(
        "Transport SEQ: %u\n",
        transportSequence);

    Serial.println();
    Serial.println("===== APPLICATION =====");

    Serial.printf(
        "App Control: 0x%02X\n",
        applicationControl);

    Serial.printf(
        "App SEQ: %u\n",
        applicationSequence);

    Serial.printf(
        "Function: 0x%02X\n",
        functionCode);

    // --------------------------------------------------------
    // ENABLE UNSOLICITED
    // --------------------------------------------------------

    if (functionCode == APP_ENABLE_UNSOLICITED)
    {
        Serial.println(
            "ENABLE_UNSOLICITED received");

        Serial.println(
            "Responding with empty RESPONSE");

        sendApplicationResponse(
            client,
            applicationSequence);

        return;
    }

    // --------------------------------------------------------
    // DISABLE UNSOLICITED
    // --------------------------------------------------------

    if (functionCode == APP_DISABLE_UNSOLICITED)
    {
        Serial.println(
            "DISABLE_UNSOLICITED received");

        sendApplicationResponse(
            client,
            applicationSequence);

        return;
    }


    // --------------------------------------------------------
    // READ
    // --------------------------------------------------------

    if (functionCode == APP_READ)
    {
         Serial.println("READ received");

        Serial.println(
            "Responding with static database");

        sendIntegrityResponse(
            client,
            applicationSequence);

        return;
    }

    // --------------------------------------------------------
    // Unknown / not implemented
    // --------------------------------------------------------

    Serial.println(
        "Application function not implemented yet");
}

// ============================================================
// Process DNP3 Frame
// ============================================================

void processDNP3Frame(
    WiFiClient& client,
    const uint8_t* frame,
    size_t frameLength)
{
    Serial.println();
    Serial.println("================================");
    Serial.printf(
        "RX: %u bytes\n",
        static_cast<unsigned>(frameLength));

    printHex(
        frame,
        frameLength);

    DNP3LinkHeader header;

    if (!parseLinkHeader(
            frame,
            frameLength,
            header))
    {
        Serial.println(
            "Invalid link frame");

        return;
    }

    Serial.printf(
        "Destination: %u\n",
        header.destination);

    Serial.printf(
        "Source: %u\n",
        header.source);

    Serial.printf(
        "Link Function: %u\n",
        header.functionCode);

    if (header.destination != OUTSTATION_ADDRESS)
    {
        Serial.println(
            "Frame not addressed to us");

        return;
    }

    uint8_t userData[256];

    size_t userDataLength =
        extractUserData(
            frame,
            frameLength,
            userData,
            sizeof(userData));

    if (userDataLength == 0)
    {
        Serial.println(
            "No valid user data");

        return;
    }

    Serial.print("User Data: ");

    printHex(
        userData,
        userDataLength);

    processApplication(
        client,
        userData,
        userDataLength);
}

// ============================================================
// Calculate Physical DNP3 Frame Size
// ============================================================

size_t calculatePhysicalFrameSize(
    uint8_t dnpLength)
{
    if (dnpLength < 5)
        return 0;

    size_t userDataLength =
        dnpLength - 5;

    size_t blocks =
        (userDataLength + 15) / 16;

    // 10 byte link header
    // + user data
    // + 2 CRC bytes per block

    return
        10 +
        userDataLength +
        (blocks * 2);
}

// ============================================================
// Setup
// ============================================================

void setup()
{
    Serial.begin(115200);

    delay(1000);

    pinMode(BI0_PIN, INPUT);
    pinMode(BI1_PIN, INPUT);
    pinMode(AI0_PIN, INPUT);
    // Setting up pins

    Serial.println();
    Serial.println("==============================");
    Serial.println(" ESP32 DNP3 Outstation");
    Serial.println("==============================");

    WiFi.mode(WIFI_STA);

    WiFi.begin(
        WIFI_SSID,
        WIFI_PASSWORD);

    Serial.print(
        "Connecting to Wi-Fi");

    while (
        WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    Serial.println();

    Serial.print(
        "ESP32 IP: ");

    Serial.println(
        WiFi.localIP());

    dnp3Server.begin();

    Serial.printf(
        "Listening on TCP port %u\n",
        DNP3_PORT);
}

// ============================================================
// Main
// ============================================================

void loop()
{
    WiFiClient client =
        dnp3Server.available();

    if (!client)
        return;

    Serial.println();
    Serial.println(
        "DNP3 master connected");

    uint8_t rxBuffer[512];

    size_t rxLength = 0;

    while (client.connected())
    {
        // ----------------------------------------------------
        // Receive TCP bytes
        // ----------------------------------------------------

        while (
            client.available() &&
            rxLength < sizeof(rxBuffer))
        {
            rxBuffer[rxLength++] =
                client.read();
        }

        // ----------------------------------------------------
        // Search for DNP3 frames
        // ----------------------------------------------------

        while (rxLength >= 3)
        {
            // Synchronize to 05 64

            if (
                rxBuffer[0] != 0x05 ||
                rxBuffer[1] != 0x64)
            {
                memmove(
                    rxBuffer,
                    rxBuffer + 1,
                    rxLength - 1);

                rxLength--;

                continue;
            }

            size_t frameSize =
                calculatePhysicalFrameSize(
                    rxBuffer[2]);

            if (frameSize == 0)
            {
                rxLength = 0;
                break;
            }

            // Wait for more TCP data.

            if (rxLength < frameSize)
                break;

            processDNP3Frame(
                client,
                rxBuffer,
                frameSize);

            // Remove processed frame.

            size_t remaining =
                rxLength - frameSize;

            memmove(
                rxBuffer,
                rxBuffer + frameSize,
                remaining);

            rxLength =
                remaining;
        }

        delay(1);
    }

    client.stop();

    Serial.println(
        "DNP3 master disconnected");
}