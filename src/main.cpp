// Embedded Challenge Spring 2024: The Tremor Challenge
// Tanzia Nur, Greyson Diaz, Saad Bensalim, Rayed Khan

#include <ArduinoFFT.h>
#include <Adafruit_CircuitPlayground.h>

const uint16_t samples = 128;
const double samplingFreq = 50.0;
const double dangerZoneIntensity = 10.0;
const int sampleInterval = 2000;  // Interval for each sample set in milliseconds
const int evaluationPeriod = 5 * 60 * 1000;  // Total period for evaluation in milliseconds
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

void setup() {
    Serial.begin(115200);
    CircuitPlayground.begin();
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
        isDeviceRunning = !isDeviceRunning;
        Serial.println(isDeviceRunning ? "Device started" : "Device stopped");
    }

    if (CircuitPlayground.rightButton()) {
        delay(200);
        isAlarmEnabled = !isAlarmEnabled;
        Serial.println(isAlarmEnabled ? "Alarm enabled" : "Alarm disabled");
    }
}