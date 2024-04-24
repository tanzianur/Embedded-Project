// Embedded Challenge Spring 2024: The Tremor Challenge
// Tanzia Nur, Greyson Diaz, Saad Bensalim, Rayed Khan

#include <ArduinoFFT.h>
#include <Adafruit_CircuitPlayground.h>

// TODO: Find danger zone threshold value
// TODO: 


// put function declarations here:
void collectData();
void performFFT(double *vReal, double *vImag);
double analyzeFFT();
void updateFeedback(double intensity);

// Constants for FFT
const uint64_t samples = 128; // Total samples for FFT, can be changed
const double samplingFreq = 50.0;
// Frequency resolution = 50 Hz / 128 = 0.39 Hz

// FFT variables
double vRealX[samples], vImagX[samples]; // x-axis
double vRealY[samples], vImagY[samples]; // y-axis
double vRealZ[samples], vImagZ[samples]; // z-axis


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  CircuitPlayground.begin();

  // Initialize all imaginary parts to zero
  for (int i = 0; i < samples; i++) {
    vImagX[i] = vImagY[i] = vImagZ[i] = 0;
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  collectData();
  performFFT(vRealX, vImagX);
  performFFT(vRealY, vImagY);
  performFFT(vRealZ, vImagZ);
}

// put function definitions here:
void collectData() {
  for (int i = 0; i < samples; i++) {
    vRealX[i] = CircuitPlayground.motionX();
    vRealY[i] = CircuitPlayground.motionY();
    vRealZ[i] = CircuitPlayground.motionZ();
    delay(1000/samplingFreq);
  }
}

void performFFT(double *vReal, double *vImag) {
  // Creating FFT object
  ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, samples, samplingFreq);
  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(FFT_FORWARD);
  FFT.complexToMagnitude();
}

double analyzeFFT() {
  double maxIntensity = 0;
  for (int axis = 0; axis < 3; axis++) {
    double *vReal = (axis == 0) ? vRealX : (axis == 1) ? vRealY : vRealZ;
    for (int i = 1; i < samples / 2; i++) {
      double frequency = i * samplingFreq / samples;
      if (frequency >= 3 && frequency <= 6) {
        maxIntensity = max(maxIntensity, vReal[i]);
      }
    }
  }
  return maxIntensity;
}

void updateFeedback(double intensity) {
    // Set the Neopixel brightness and color based on the intensity level
    if (intensity > 150) {  // Danger zone threshold
        // If in danger zone, light up at maximum brightness with a red color
        CircuitPlayground.setBrightness(255); // Max brightness
        CircuitPlayground.setPixelColor(0, CircuitPlayground.colorWheel(0)); // Red color
        tone(5, 1000, 500);  // Sound an alarm at 1000 Hz for 500 ms on pin 5
    } else {
        // If not in danger zone, light up with a dimmer green color
        CircuitPlayground.setBrightness(50); // Lower brightness
        CircuitPlayground.setPixelColor(0, CircuitPlayground.colorWheel(96)); // Green color
    }

}