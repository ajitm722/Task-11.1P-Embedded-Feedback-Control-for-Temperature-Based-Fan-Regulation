#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>
#include <WiFiNINA.h>
#include <cmath> // for isnan

// ===========================
// DHT11 Sensor Configuration
// ===========================
namespace sensor
{
    constexpr uint8_t kDataPin{2};
    constexpr auto kSensorType{DHT11};
    constexpr unsigned long kIntervalMs{5000};

    DHT dht{kDataPin, kSensorType};

    void begin()
    {
        dht.begin();
    }

    float readTemperature()
    {
        return dht.readTemperature();
    }
}

// ===========================
// OLED Display Configuration
// ===========================
namespace display
{
    constexpr uint8_t SCREEN_WIDTH{128};
    constexpr uint8_t SCREEN_HEIGHT{64};
    constexpr uint8_t OLED_RESET{4};
    constexpr uint8_t I2C_ADDRESS{0x3C};

    Adafruit_SSD1306 oled{SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET};

    void begin()
    {
        if (!oled.begin(SSD1306_SWITCHCAPVCC, I2C_ADDRESS))
        {
            Serial.println(F("OLED init failed"));
            while (true)
            {
                delay(100);
            }
        }

        oled.clearDisplay();
        oled.setTextColor(SSD1306_WHITE);
        oled.setTextSize(2);
        oled.setCursor(0, 0);
        oled.println(F("OLED Ready"));
        oled.display();
    }

    void showTemperature(const float temperatureC)
    {
        oled.clearDisplay();
        oled.setTextSize(2);
        oled.setCursor(0, 0);

        if (std::isnan(temperatureC))
        {
            oled.println(F("Sensor Error"));
        }
        else
        {
            oled.print(F("Temp:"));
            oled.setCursor(0, 30);
            oled.print(temperatureC, 2);
            oled.print(F(" C"));
        }

        oled.display();
    }
}

// ===========================
// WiFi + HTTP POST Logic
// ===========================
namespace network
{
    constexpr char ssid[] = "OPTUS_EF1429M";
    constexpr char password[] = "wifipassword"; // Replace with your WiFi password
    constexpr char server[] = "192.168.0.243";  // Replace with Raspberry Pi IP
    constexpr uint16_t port = 8000;

    WiFiClient client;

    void connect()
    {
        Serial.print("Connecting to WiFi");
        while (WiFi.begin(ssid, password) != WL_CONNECTED)
        {
            Serial.print(".");
            delay(1000);
        }
        Serial.println(F("\nWiFi connected"));
        Serial.print(F("IP Address: "));
        Serial.println(WiFi.localIP());
    }

    void sendTemperature(float tempC)
    {
        if (!client.connect(server, port))
        {
            Serial.println(F("HTTP connection failed"));
            return;
        }

        String payload = "{\"temperature\":" + String(tempC, 2) + "}";
        String request =
            "POST /temp HTTP/1.1\r\n"
            "Host: " +
            String(server) + "\r\n"
                             "Content-Type: application/json\r\n"
                             "Content-Length: " +
            String(payload.length()) + "\r\n\r\n" +
            payload;

        client.print(request);

        // Optional: print server response
        unsigned long timeout = millis();
        while (client.connected() && millis() - timeout < 1000)
        {
            while (client.available())
            {
                char c = client.read();
                Serial.write(c);
            }
        }
        client.stop();
    }
}

// ===========================
// Arduino Setup & Loop
// ===========================
void setup()
{
    Serial.begin(9600);
    while (!Serial)
        ;

    Serial.println(F("Starting Temperature Monitor..."));

    sensor::begin();
    display::begin();
    network::connect();

    delay(2000);
}

void loop()
{
    static unsigned long lastReadTime{0};
    const unsigned long currentTime{millis()};

    if (currentTime - lastReadTime >= sensor::kIntervalMs)
    {
        lastReadTime = currentTime;

        const float temperature = sensor::readTemperature();

        Serial.print(F("Temperature: "));
        if (std::isnan(temperature))
        {
            Serial.println(F("Read failed"));
        }
        else
        {
            Serial.print(temperature);
            Serial.println(F(" °C"));
        }

        display::showTemperature(temperature);

        if (!std::isnan(temperature))
        {
            network::sendTemperature(temperature);
        }
    }
}
