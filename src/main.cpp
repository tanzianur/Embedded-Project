// Embedded Challenge Spring 2024: The Tremor Challenge
// Tanzia Nur, Greyson Diaz, Saad Bensalim, Rayed Khan

#include <ArduinoFFT.h>
#include <Adafruit_CircuitPlayground.h>

const uint16_t samples = 128;
const double samplingFreq = 50.0;
const double dangerZoneIntensity = 60.0;
const int sampleInterval = 2000;  // Interval for each sample set in milliseconds
const int evaluationPeriod = 30 * 1000;  // Total period for evaluation in milliseconds
double vReal[samples], vImag[samples];
unsigned int index = 0, sampleCount = 0, dangerCount = 0;
unsigned long lastTime = 0, lastSampleSetTime = 0;
unsigned long samplingPeriod = 1000000 / samplingFreq;
bool isDeviceRunning = false;
bool isAlarmEnabled = false;

void handleButtonPress();
bool collectSamples();
void performFFT();
double analyzeFFT();
void updateFeedback(double intensity);

void setup() {
    Serial.begin(115200);
    CircuitPlayground.begin();
    CircuitPlayground.clearPixels(); // Clear the Neopixel to start fresh
    memset(vImag, 0, sizeof(vImag));
    for (int i = 0; i < samples; i++) vImag[i] = 0;
    lastSampleSetTime = millis();
}

void loop() {
    handleButtonPress();  // Handle button interactions to start/stop device and toggle alarm
    if (isDeviceRunning) {
        if (collectSamples()) {  // Collects data samples for the FFT
            performFFT();  // Performs FFT on the collected data
            double intensity = analyzeFFT();  // Analyzes the FFT data to calculate maximum intensity
            updateFeedback(intensity);  // Update Neopixel based on the calculated intensity
            // Debug output to monitor intensity values
            Serial.print("Intensity: "); Serial.println(intensity);

            if (millis() - lastSampleSetTime >= sampleInterval) {
                if (intensity >= dangerZoneIntensity) {
                    dangerCount++;  // Increment the count of dangerous samples
                }
                sampleCount++;  // Increment the total count of samples

                // Debug outputs to check the count of samples and danger occurrences
                Serial.print("Sample Count: "); Serial.println(sampleCount);
                Serial.print("Danger Count: "); Serial.println(dangerCount);

                if (millis() - lastSampleSetTime >= evaluationPeriod) {  // Check if the evaluation period is over
                    double dangerRatio = (double)dangerCount / sampleCount;
                    Serial.print("Danger Ratio: "); Serial.println(dangerRatio);
                    if (dangerRatio >= 0.6 && isAlarmEnabled) {
                        Serial.println("Alarm sounding: Danger level exceeded");
                        // Additional code to trigger an alarm
                        CircuitPlayground.playTone(1000, 500);  // Play a 1000 Hz tone for 500 milliseconds
                    } else {
                        Serial.println("Not enough danger signals to sound the alarm.");
                    }
                    // Reset counters after the evaluation period
                    dangerCount = 0;
                    sampleCount = 0;
                    lastSampleSetTime = millis();
                }
            }
        }
    }
}

bool collectSamples() {
    if (micros() - lastTime >= samplingPeriod) {
        lastTime = micros();
        double x = CircuitPlayground.motionX();
        double y = CircuitPlayground.motionY();
        double z = CircuitPlayground.motionZ();
        if (index == 0) {  // Reset vReal at the start of each new sampling set
            for (int i = 0; i < samples; i++) vReal[i] = 0;
        }
        vReal[index] = sqrt(x * x + y * y + z * z);
        index++;
        if (index >= samples) {
            index = 0;
            return true;
        }
    }
    return false;
}

void performFFT() {
    memset(vImag, 0, sizeof(vImag));
    ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, samples, samplingFreq);
    FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.compute(FFT_FORWARD);
    FFT.complexToMagnitude();
}

double analyzeFFT() {
    double maxIntensity = 0;
    for (int i = 1; i < samples / 2; i++) {
        double frequency = i * samplingFreq / samples;
        if (frequency >= 3 && frequency <= 6) {
            maxIntensity = max(maxIntensity, vReal[i]);
        }
    }
    return maxIntensity;
}

void handleButtonPress() {
    if (CircuitPlayground.leftButton()) {
        delay(200);  // Debounce delay
        CircuitPlayground.playTone(1000, 500);
        delay(200);  // Debounce delay
          // Play a 1000 Hz tone for 500 milliseconds
        CircuitPlayground.playTone(2000, 500);  // Play a 1000 Hz tone for 500 milliseconds
        CircuitPlayground.clearPixels(); // Clear the Neopixel to start fresh
        isDeviceRunning = !isDeviceRunning;
        Serial.println(isDeviceRunning ? "Device started" : "Device stopped");
    }
    if (CircuitPlayground.rightButton()) {
        delay(200);
        CircuitPlayground.playTone(2000, 500);
        isAlarmEnabled = !isAlarmEnabled;
        Serial.println(isAlarmEnabled ? "Alarm enabled" : "Alarm disabled");
    }
}

void updateFeedback(double intensity) {
    const int lowThreshold = 25;  // Adjust these values based on your calibration and testing
    const int highThreshold = 60;

    uint8_t red, green, blue;
    if (intensity < lowThreshold) {
        // Green color - low intensity
        CircuitPlayground.clearPixels();
        green = 255;
        red = 0;
        blue = 0;
        CircuitPlayground.setPixelColor(4, 0, green, 0);
        CircuitPlayground.setPixelColor(5, 0, green, 0);
    } else if (intensity >= lowThreshold && intensity < highThreshold) {
        // Yellow color - transitioning from green to red
        CircuitPlayground.clearPixels();
        green = 255;
        red = 255;
        blue = 0;
        CircuitPlayground.setPixelColor(2, red, green, blue);
        CircuitPlayground.setPixelColor(3, red, green, blue);
        CircuitPlayground.setPixelColor(4, 0, green, 0);
        CircuitPlayground.setPixelColor(5, 0, green, 0);
        CircuitPlayground.setPixelColor(6, red, green, blue);
        CircuitPlayground.setPixelColor(7, red, green, blue);
    } else {
        // Red color - high intensity
        CircuitPlayground.clearPixels();
        red = 255;
        green = 0;
        blue = 0;
        CircuitPlayground.setPixelColor(0, red, green, blue);
        CircuitPlayground.setPixelColor(1, red, green, blue);
        CircuitPlayground.setPixelColor(2, 255, 255, blue);
        CircuitPlayground.setPixelColor(3, 255, 255, blue);
        CircuitPlayground.setPixelColor(4, 0, 255, 0);
        CircuitPlayground.setPixelColor(5, 0, 255, 0);
        CircuitPlayground.setPixelColor(6, 255, 255, blue);
        CircuitPlayground.setPixelColor(7, 255, 255, blue);
        CircuitPlayground.setPixelColor(8, red, green, blue);
        CircuitPlayground.setPixelColor(9, red, green, blue);
    }
}
