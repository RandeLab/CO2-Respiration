#include <Adafruit_SSD1306.h>
#include <SparkFunBME280.h>
#include <SparkFunCCS811.h>
#include <Wire.h>
#include <OneWire.h>

#define OLED_RESET 4

#define TIMEOUT 500
#define HIGH_RES_INTERVAL 2
#define LOW_RES_INTERVAL_FACT 60

#define HIGH_RES_COUNT 150
#define LOW_RES_COUNT 150

#define CCS811_ADDR 0x5A

#define PIN_START 8

CCS811 myCCS811(CCS811_ADDR);
BME280 myBME280;

// adding the DS18B20 Temperature Sensor (needs OneWire.h library
OneWire  ds(6);  // on pin 10 (a 4.7K resistor is necessary)

uint32_t displayFactor = 1000 / 48;
int32_t thresholdPPM = 450;

uint8_t CMD[] = { 0XFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79 };
uint8_t CAL[] = { 0xFF, 0x01, 0x87, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78 };

Adafruit_SSD1306 display(OLED_RESET);

struct Measurement {
    int16_t ppm;
    int16_t tvoc;
    int16_t eco2;
    int32_t temp;
    float hum;
    float press;
};

struct Measurement clearMeasurement {
    .ppm = 0,
    .tvoc = 0,
    .eco2 = 0,
    .temp = 0,
    .hum = 0,
    .press = 0,
};

struct Measurement lres[LOW_RES_COUNT] = { 0 };
struct Measurement hres[HIGH_RES_COUNT] = { 0 };

uint16_t h_head = 0;
uint16_t l_head = 0;

uint16_t high_res_counter = LOW_RES_INTERVAL_FACT; // start off to trigger
uint16_t seconds_counter = 0;
uint32_t meas_counter = 0;

volatile uint8_t next = 0;

int32_t started_idx = HIGH_RES_COUNT - 1;
unsigned long started = millis();

IntervalTimer measurementTimer;

void setup()
{

    initSensors();

    Serial.begin(115200);
    Serial1.begin(9600);
    Serial2.begin(9600);

    delay(1000);
    Serial.print("C02 Sensor MH-Z16\n");
    Serial2.print("C02 Sensor MH-Z16\n");

    measurementTimer.begin(timer, 1000000);

    initDisplay();

    pinMode(8, INPUT_PULLUP);

    OLEDdrawBackground();
}

void loop()
{

    char c = 0;
    while (Serial.available()) {
        c = Serial.read();
    }

    while (Serial2.available()) {
        c = Serial2.read();
    }

    if (c == 'a') {
        print_values();
    }

    if (c == 'c') {
        Serial1.write(CAL, sizeof(CAL));
        delay(20);
        while (Serial1.available()) {
            Serial.printf("%0x02X ", Serial1.read());
        }
    }

    if (digitalRead(PIN_START) == LOW) {
        started_idx = HIGH_RES_COUNT - 1;
        started = millis();
        meas_counter = 0;
        OLEDdrawBackground();
    }

    process();

    delay(30);
}

int32_t measure(struct Measurement* current)
{
    Serial1.write(CMD, sizeof(CMD));
    uint8_t expected = 9, count = 0;
    uint16_t concentration = 0;

    unsigned long s = millis();

    for (;;) {
        if (millis() - s > TIMEOUT) {
            current->ppm = -1;
            return -1;
        }
        while (Serial1.available()) {
            uint8_t c = Serial1.read();
            if (count == 2) {
                concentration = c << 8;
            } else if (count == 3) {
                concentration |= c;
            } else if (count == 4) {
                current->temp = c - 40;
            }

            count++;
        }

        if (count >= expected) {
            break;
        }

        delay(50);
    }

    current->ppm = concentration;
    return concentration;
}

void timer()
{
    seconds_counter++;
    if (seconds_counter % HIGH_RES_INTERVAL == 0) {
        next = 1;
        seconds_counter = 0;
    }
}

void process()
{
    static struct Measurement current {
        .ppm = 0,
        .tvoc = 0,
        .eco2 = 0,
        .temp = 0,
        .hum = 0,
        .press = 0,
    };

    if (next == 0) {
        return;
    }
    next = 0;

    current = clearMeasurement;

    high_res_counter++;
    meas_counter++;

    if (started_idx >= 0) {
        started_idx--;

        if (started_idx < 0) {
            meas_counter = 0;
            OLEDdrawBackground();
        }
    }

    measure(&current);
    measureSensors(&current);

    push_high_res(&current);

    if (high_res_counter >= LOW_RES_INTERVAL_FACT) {
        high_res_counter = 0;
        push_low_res(&current);
    }

    update_display(&current);
}

int32_t last_ppm_high_res()
{
    if (h_head == 0) {
        return hres[HIGH_RES_COUNT - 1].ppm;
    } else {
        return hres[h_head - 1].ppm;
    }
}
void push_high_res(struct Measurement* current)
{
    hres[h_head] = *current;

    h_head++;
    if (h_head >= HIGH_RES_COUNT) {
        h_head = 0;
    }
}

void push_low_res(struct Measurement* current)
{
    lres[l_head] = *current;

    l_head++;
    if (l_head >= LOW_RES_COUNT) {
        l_head = 0;
    }
}

void for_low_res(void (*cb)(uint32_t, struct Measurement*))
{
    uint32_t i = 0;
    uint32_t len = LOW_RES_COUNT;
    uint32_t idx = 0;

    uint32_t tail = l_head;

    if (tail < len) {
        for (i = tail; i < len; i++) {
            cb(idx++, &lres[i]);
        }
    }

    for (i = 0; i < l_head; i++) {
        cb(idx++, &lres[i]);
    }
}

void for_high_res(void (*cb)(uint32_t, struct Measurement*))
{
    uint32_t i = 0;
    uint32_t len = HIGH_RES_COUNT;
    uint32_t idx = 0;

    uint32_t tail = h_head;

    if (tail < len) {
        for (i = tail; i < len; i++) {
            cb(idx++, &hres[i]);
        }
    }

    for (i = 0; i < h_head; i++) {
        cb(idx++, &hres[i]);
    }
}

void reset_buffers()
{
    h_head = 0;
    l_head = 0;

    uint32_t i = 0;
    for (i = 0; i < LOW_RES_COUNT; i++) {
        lres[i] = clearMeasurement;
    }

    for (i = 0; i < HIGH_RES_COUNT; i++) {
        hres[i] = clearMeasurement;
    }
}

void print_hres_cb(uint32_t idx, struct Measurement* m)
{
    print("\n[");
    print(m->ppm);
    print(",");
    print(m->temp);
    print(",");
    print(m->tvoc);
    print(",");
    print(m->eco2);
    print(",");
    print(m->hum);
    print(",");
    print(m->press);
    print("]");

    if (idx < HIGH_RES_COUNT - 1) {
        print(",");
    }
}

void print_lrs_cb(uint32_t idx, struct Measurement* m)
{
    print("\n[");
    print(m->ppm);
    print(",");
    print(m->temp);
    print(",");
    print(m->tvoc);
    print(",");
    print(m->eco2);
    print(",");
    print(m->hum);
    print(",");
    print(m->press);
    print("]");

    if (idx < LOW_RES_COUNT - 1) {
        print(",");
    }
}

void print_high_res()
{
    print("\"high_res\":[");
    for_high_res(&print_hres_cb);
    print("]");
}

void print_low_res()
{
    print("\"low_res\":[");
    for_low_res(&print_lrs_cb);
    print("]");
}

void print(const char* data)
{
    Serial.print(data);
    Serial2.print(data);
}

void print(long v)
{
    Serial.print(v);
    Serial2.print(v);
}

void print_values()
{
    print("{\n");
    print_high_res();
    print(",\n");
    print_low_res();
    print(",\n");
    print("\"started_index\":");
    print(started_idx);
    print("}\n");
}

void update_display(struct Measurement* current)
{
    OLEDshowGraph(current);
}

void initDisplay()
{
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0, 25);
    display.println("AirQuality");
    display.setTextSize(1);
    display.setCursor(0, 46);
    display.println("BME280 Temp/Hum");
    display.setCursor(0, 56);
    display.println("CCS811 CO2 Sensor");
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.println("ABC");
    display.display();
}

void initSensors()
{
    myCCS811.begin();

    myBME280.settings.commInterface = I2C_MODE;
    myBME280.settings.I2CAddress = 0x76;
    myBME280.settings.runMode = 3; //Normal mode
    myBME280.settings.tStandby = 0;
    myBME280.settings.filter = 4;
    myBME280.settings.tempOverSample = 5;
    myBME280.settings.pressOverSample = 5;
    myBME280.settings.humidOverSample = 5;

    delay(100); //Make sure sensor had enough time to turn on. BME280 requires 2ms to start up.
    myBME280.begin();
    delay(500);
}

int16_t measureSensors(struct Measurement* current)
{
    if (myCCS811.dataAvailable()) {
        //Calling this function updates the global tVOC and eCO2 variables
        myCCS811.readAlgorithmResults();
        current->eco2 = myCCS811.getCO2();
        current->tvoc = myCCS811.getTVOC();
        //current-> = myBME280.readTempC();
        current->press = myBME280.readFloatPressure();
        current->hum = myBME280.readFloatHumidity();

        //This sends the temperature data to the CCS811
        //myCCS811.setEnvironmentalData(BMEhumid, BMEtempC);
        return 0;
    }

    return -1;
}

void OLEDshowGraph(struct Measurement* current)
{
    display.setTextSize(2);
    display.fillRect(0, 0, 128, 16, BLACK);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.print(current->ppm);
    display.println(" ppm");
    //display.setCursor(0,20);
    //display.print(ppmCCS881);
    //display.println(" CCS");
    display.setTextSize(1);
    display.setCursor(100, 0);
    display.print(current->temp);
    display.println("T");
    display.setCursor(100, 9);
    display.print((millis() - started) / 1000);
    display.println("s");

    display.setTextSize(1);
    display.fillRect(1, 46, 80, 17, BLACK);
    display.setCursor(2, 47);
    display.print(current->eco2);
    display.println(" ppm eCO2");
    display.setCursor(2, 55);
    display.print(current->hum);
    display.println(" % hum");

    if (meas_counter > 0) {
        display.drawLine(meas_counter - 1, 64 - last_ppm_high_res() / displayFactor, meas_counter, 64 - current->ppm / displayFactor, WHITE);
    }

    if (current->ppm > thresholdPPM) {
        if (last_ppm_high_res() < thresholdPPM) {
            display.setTextSize(1);
            display.setCursor(h_head + 3, thresholdPPM / displayFactor + 10);
            display.print((millis() - started) / 1000);
            display.println("s");
        }
    }

    display.display();
    delay(300);
}

void OLEDdrawBackground()
{
    display.clearDisplay();
    for (int i = 0; i <= 128; i = i + 3) {
        display.drawPixel(i, 64 - (400 / displayFactor), WHITE);
    }
    for (int i = 0; i <= 128; i = i + 5) {
        display.drawPixel(i, 64 - (1000 / displayFactor), WHITE);
    }
    for (int i = 0; i <= 128; i = i + 7) {
        display.drawPixel(i, 64 - (3000 / displayFactor), WHITE);
    }
    display.drawRect(0, 16, 128, 48, 1); //Border of the bar chart

    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(102, 19);
    display.println("1000");
    display.setCursor(108, 46);
    display.println("400");

    display.display();
}

