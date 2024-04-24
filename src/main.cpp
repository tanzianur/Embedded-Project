// Embedded Challenge Spring 2024: The Tremor Challenge
// Tanzia Nur, Greyson Diaz, Saad Bensalim, Rayed Khan

#include <ArduinoFFT.h>
#include <Adafruit_CircuitPlayground.h>

// put function declarations here:
void collectData();
void performFFT(double *vReal, double *vImag);
double analyzeFFT();

Adafruit_CPlay_NeoPixel pixels(1, 17, NEO_GRB + NEO_KHZ800);

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
  pixels.begin();

  // Initialize all imaginary parts to zero
  for (int i = 0; i < samples; i++) {
    vImagX[i] = vImagY[i] = vImagZ[i] = 0;
  }
}

void loop() {
  // put your main code here, to run repeatedly:
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
  // Map the intensity to brightness values
  int brightness = map(intensity, 0, 200, 0, 255);  // Adjust range based on observed values

  // Set color based on intensity level
  if (intensity > 150) {  // Danger zone threshold
    pixels.setPixelColor(0, pixels.Color(255, 0, 0));  // Red color for high intensity
    tone(PC6, 1000, 500);  // Sound an alarm at 1000 Hz for 500 ms
  } else if (intensity > 100) {
    pixels.setPixelColor(0, pixels.Color(255, 165, 0));  // Orange color for medium intensity
  } else {
    pixels.setPixelColor(0, pixels.Color(0, 255, 0));  // Green color for low intensity
  }
  
  pixels.setBrightness(brightness);  // Set the brightness based on the intensity
  pixels.show();
}
// hello 