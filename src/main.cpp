// Embedded Challenge Spring 2024: The Tremor Challenge
// Tanzia Nur, Greyson Diaz, Saad Bensalim, Rayed Khan

#include <ArduinoFFT.h>
#include <Adafruit_CircuitPlayground.h>

// TODO #1: Find danger zone threshold value
// TODO #2: Figure out button configuration
// TODO #3: Figure out Neopixel config and updateFeedback() func logistics
// TODO #4: Figure out speaker config
// TODO #5: Fix analyzeFFT()
//  - Need to store samples and have a counter to see how many of 
//   the samples are in the danger zone. Then compare to percentages,
//   less than 50% not dangerous 60+ dangerous

// put function declarations here:
const uint16_t samples = 128; // Total samples for FFT
const double samplingFreq = 50.0; // Hz

// FFT variables for magnitude
double vReal[samples], vImag[samples];
unsigned int index = 0;
unsigned long lastTime = 0;
unsigned long samplingPeriod = 1000000 / samplingFreq; // Sampling period in microseconds

void setup() {
    Serial.begin(115200);
    CircuitPlayground.begin();
    for (int i = 0; i < samples; i++) {
        vImag[i] = 0; // Imaginary parts are zero
    }
}

bool collectSamples() {
    if (micros() - lastTime >= samplingPeriod) {
        lastTime = micros();
        double x = CircuitPlayground.motionX();
        double y = CircuitPlayground.motionY();
        double z = CircuitPlayground.motionZ();
        vReal[index] = sqrt(x * x + y * y + z * z); // Calculate the magnitude and store it
        index++;
        if (index >= samples) {
            index = 0;
            return true; // Sampling complete
        }
    }
    return false; // Sampling not complete
}

void performFFT() {
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

void updateFeedback(double intensity) {
    // Update feedback logic here based on intensity
}

void loop() {
    if (collectSamples()) {
        performFFT();
        double intensity = analyzeFFT();
        updateFeedback(intensity);
    }
}