#include <Arduino.h>
#include <WiFi.h>

// ============================================================
// Wi-Fi
// ============================================================

const char* WIFI_SSID = "VIRGIN464";
const char* WIFI_PASSWORD = "79A53D31D477";

// ============================================================
// DNP3 configuration
// ============================================================

constexpr uint16_t DNP3_PORT = 20000;

constexpr uint16_t OUTSTATION_ADDRESS = 4;
constexpr uint16_t MASTER_ADDRESS     = 3;

WiFiServer dnp3Server(DNP3_PORT);

// ============================================================
// DNP3 Link Layer Header
//
// 05 64 LEN CTRL DEST_L DEST_H SRC_L SRC_H CRC_L CRC_H
// ============================================================

struct DNP3LinkHeader
{
    uint8_t length;
    uint8_t control;

    uint16_t destination;
    uint16_t source;

    bool dir;
    bool prm;
    bool fcb;
    bool fcv_dfc;

    uint8_t functionCode;
};


// ============================================================
// Utility
// ============================================================

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
// Link Header Parser
// ============================================================

bool parseLinkHeader(
    const uint8_t* data,
    size_t size,
    DNP3LinkHeader& header)
{
    // Complete DNP3 link header is 10 bytes.
    if (size < 10)
        return false;

    // Every DNP3 frame starts with 05 64.
    if (data[0] != 0x05 || data[1] != 0x64)
    {
        Serial.println("Invalid DNP3 start bytes");
        return false;
    }

    header.length  = data[2];
    header.control = data[3];

    // DNP3 transmits addresses little-endian.
    header.destination =
        static_cast<uint16_t>(data[4]) |
        (static_cast<uint16_t>(data[5]) << 8);

    header.source =
        static_cast<uint16_t>(data[6]) |
        (static_cast<uint16_t>(data[7]) << 8);

    // Decode control byte.
    header.dir     = (header.control & 0x80) != 0;
    header.prm     = (header.control & 0x40) != 0;
    header.fcb     = (header.control & 0x20) != 0;
    header.fcv_dfc = (header.control & 0x10) != 0;

    header.functionCode = header.control & 0x0F;

    return true;
}


// ============================================================
// Debug Header
// ============================================================

void printLinkHeader(const DNP3LinkHeader& header)
{
    Serial.println();
    Serial.println("----- DNP3 LINK HEADER -----");

    Serial.printf("Length:      %u\n", header.length);
    Serial.printf("Control:     0x%02X\n", header.control);

    Serial.printf(
        "Destination: %u\n",
        header.destination);

    Serial.printf(
        "Source:      %u\n",
        header.source);

    Serial.printf("DIR:         %u\n", header.dir);
    Serial.printf("PRM:         %u\n", header.prm);
    Serial.printf("FCB:         %u\n", header.fcb);
    Serial.printf("FCV/DFC:     %u\n", header.fcv_dfc);

    Serial.printf(
        "Function:    %u\n",
        header.functionCode);

    Serial.println("----------------------------");
}


// ============================================================
// Process incoming DNP3 data
// ============================================================

void processDNP3Frame(
    const uint8_t* data,
    size_t length)
{
    Serial.println();
    Serial.printf(
        "Received %u bytes\n",
        static_cast<unsigned>(length));

    printHex(data, length);

    DNP3LinkHeader header;

    if (!parseLinkHeader(data, length, header))
    {
        Serial.println("Failed to parse DNP3 header");
        return;
    }

    printLinkHeader(header);

    // Make sure Ignition is actually talking to us.
    if (header.destination != OUTSTATION_ADDRESS)
    {
        Serial.printf(
            "Frame intended for outstation %u, not us (%u)\n",
            header.destination,
            OUTSTATION_ADDRESS);

        return;
    }

    if (header.source != MASTER_ADDRESS)
    {
        Serial.printf(
            "Unexpected master address: %u\n",
            header.source);
    }
}


// ============================================================
// Setup
// ============================================================

void setup()
{
    Serial.begin(115200);

    delay(1000);

    Serial.println();
    Serial.println("==============================");
    Serial.println(" ESP32 DNP3 Outstation");
    Serial.println("==============================");

    WiFi.mode(WIFI_STA);

    WiFi.begin(
        WIFI_SSID,
        WIFI_PASSWORD);

    Serial.print("Connecting to Wi-Fi");

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    Serial.println();
    Serial.println("Wi-Fi connected");

    Serial.print("ESP32 IP: ");
    Serial.println(WiFi.localIP());

    Serial.printf(
        "DNP3 Outstation Address: %u\n",
        OUTSTATION_ADDRESS);

    Serial.printf(
        "DNP3 Master Address: %u\n",
        MASTER_ADDRESS);

    dnp3Server.begin();

    Serial.printf(
        "Listening for DNP3 on TCP port %u\n",
        DNP3_PORT);
}


// ============================================================
// Main Loop
// ============================================================

void loop()
{
    WiFiClient client = dnp3Server.available();

    if (!client)
        return;

    Serial.println();
    Serial.println("DNP3 master connected");

    uint8_t buffer[512];

    while (client.connected())
    {
        if (client.available())
        {
            size_t bytesRead =
                client.read(
                    buffer,
                    sizeof(buffer));

            processDNP3Frame(
                buffer,
                bytesRead);
        }

        delay(1);
    }

    client.stop();

    Serial.println("DNP3 master disconnected");
}