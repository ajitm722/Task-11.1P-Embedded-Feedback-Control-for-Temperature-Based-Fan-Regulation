#include <Wire.h>             // Library for I2C communication
#include <Adafruit_GFX.h>     // Core graphics library for OLED
#include <Adafruit_SSD1306.h> // Library for SSD1306 OLED display
#include <DHT.h>              // Library for DHT sensor
#include <WiFiNINA.h>         // Library for WiFi communication
#include <cmath>              // For isnan function to check if a value is NaN

// ===========================
// DHT11 Sensor Configuration
// ===========================
namespace sensor
{
    constexpr uint8_t kDataPin{2};        // Pin connected to the DHT11 sensor
    constexpr auto kSensorType{DHT11};    // Type of DHT sensor
    constexpr uint16_t kIntervalMs{2000}; // Interval between sensor readings in milliseconds

    DHT dht{kDataPin, kSensorType}; // Initialize DHT sensor object

    void begin()
    {
        dht.begin(); // Start the DHT sensor
    }

    float readTemperature()
    {
        return dht.readTemperature(); // Read temperature in Celsius from the DHT sensor
                                      // Useful for monitoring environmental conditions
    }
}

// ===========================
// OLED Display Configuration
// ===========================
namespace display
{
    constexpr uint8_t SCREEN_WIDTH{128}; // OLED display width in pixels
    constexpr uint8_t SCREEN_HEIGHT{64}; // OLED display height in pixels
    constexpr uint8_t OLED_RESET{4};     // Reset pin for OLED (not used here)
    constexpr uint8_t I2C_ADDRESS{0x3C}; // I2C address for the OLED display

    Adafruit_SSD1306 oled{SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET}; // Initialize OLED object

    void begin()
    {
        if (!oled.begin(SSD1306_SWITCHCAPVCC, I2C_ADDRESS)) // Initialize OLED with I2C address
        {
            Serial.println(F("OLED init failed")); // Print error if initialization fails
                                                   // Useful for debugging hardware issues
            while (true)
            {
                delay(100); // Halt execution to prevent further errors
            }
        }

        oled.clearDisplay();              // Clear the display buffer
        oled.setTextColor(SSD1306_WHITE); // Set text color to white
        oled.setTextSize(2);              // Set text size
        oled.setCursor(0, 0);             // Set cursor position
        oled.println(F("OLED Ready"));    // Display initialization message
        oled.display();                   // Send buffer to the display
    }

    void showTemperature(const float temperatureC)
    {
        oled.clearDisplay();  // Clear the display buffer
        oled.setTextSize(2);  // Set text size
        oled.setCursor(0, 0); // Set cursor position

        if (std::isnan(temperatureC)) // Check if temperature reading is invalid
        {
            oled.println(F("Sensor Error")); // Display error message
                                             // Useful for notifying the user of sensor issues
        }
        else
        {
            oled.print(F("Temp:"));      // Display "Temp:" label
                                         // Useful for providing context to the displayed value
            oled.setCursor(0, 30);       // Move cursor to next line
            oled.print(temperatureC, 2); // Display temperature value with 2 decimal places
                                         // Useful for showing precise temperature readings
            oled.print(F(" C"));         // Display unit (Celsius)
                                         // Useful for clarifying the measurement unit
        }

        oled.display(); // Send buffer to the display
    }
}

// ===========================
// WiFi + HTTP POST Logic
// ===========================
namespace network
{
    constexpr char ssid[] = "OPTUS_EF1429M";    // WiFi SSID
    constexpr char password[] = "wifipassword"; // WiFi password
    constexpr char server[] = "192.168.0.243";  // Server IP address
    constexpr uint16_t port = 8000;             // Server port

    WiFiClient client; // WiFi client object for HTTP communication

    void connect()
    {
        Serial.print("Connecting to WiFi");                // Print connection status
        while (WiFi.begin(ssid, password) != WL_CONNECTED) // Attempt to connect to WiFi
        {
            Serial.print("."); // Print dots while connecting
                               // Useful for providing visual feedback during connection attempts
            delay(1000);       // Wait 1 second
        }
        Serial.println(F("\nWiFi connected")); // Print success message
        Serial.print(F("IP Address: "));       // Print IP address
        Serial.println(WiFi.localIP());        // Display local IP address
    }

    void sendTemperature(float tempC)
    {
        if (!client.connect(server, port)) // Connect to server
        {
            Serial.println(F("HTTP connection failed")); // Print error if connection fails
            return;
        }

        // Create JSON payload with temperature data
        String payload = "{\"temperature\":" + String(tempC, 2) + "}";
        // `String` (capital S) is an Arduino class for handling text dynamically.
        // It is useful in embedded systems for constructing the JSON payload

        // Create HTTP POST request
        String request =
            "POST /temp HTTP/1.1\r\n"
            "Host: " +
            String(server) + "\r\n"
                             "Content-Type: application/json\r\n"
                             "Content-Length: " +
            String(payload.length()) + "\r\n\r\n" +
            payload;
        // `String` is used again to dynamically build the HTTP request string.
        // This allows flexibility in formatting and including variable data like
        // the server address and payload length.

        client.print(request); // Send HTTP request

        client.stop(); // Close connection
    }
}

// ===========================
// Arduino Setup & Loop
// ===========================
void setup()
{
    Serial.begin(9600); // Start Serial communication at 9600 baud
    while (!Serial)
        ; // Wait for Serial to initialize

    Serial.println(F("Starting Temperature Monitor...")); // Print startup message

    sensor::begin();    // Initialize DHT sensor
    display::begin();   // Initialize OLED display
    network::connect(); // Connect to WiFi

    delay(2000); // Wait 2 seconds before starting loop
}

void loop()
{
    const float temperature{sensor::readTemperature()}; // Read temperature from sensor

    Serial.print(F("Temperature: ")); // Print temperature to Serial
    if (std::isnan(temperature))      // Check if reading is invalid
    {
        Serial.println(F("Read failed")); // Print error message
    }
    else
    {
        Serial.print(temperature); // Print valid temperature
        Serial.println(F(" °C"));  // Print unit
    }

    display::showTemperature(temperature); // Display temperature on OLED

    if (!std::isnan(temperature)) // Send temperature to server if valid
    {
        network::sendTemperature(temperature);
    }

    delay(sensor::kIntervalMs); // Wait for the specified interval
}
