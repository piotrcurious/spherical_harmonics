/*
 * ESP32 Spherical Harmonics Visualization
 * For TFT_eSPI Library
 * 
 * This demo visualizes different spherical harmonics and
 * spherical molecular orbitals (SMOs) on an ESP32 with TFT display.
 * The visualization rotates the 3D representation of spherical harmonics.
 */

#include <TFT_eSPI.h>
#include <SPI.h>
#include <math.h>

TFT_eSPI tft = TFT_eSPI();

// Screen dimensions
#define SCREEN_WIDTH tft.width()
#define SCREEN_HEIGHT tft.height()
#define SCREEN_CENTER_X (SCREEN_WIDTH / 2)
#define SCREEN_CENTER_Y (SCREEN_HEIGHT / 2)

// Color definitions
#define RED_COLOR TFT_RED
#define BLUE_COLOR TFT_BLUE
#define GREEN_COLOR TFT_GREEN
#define YELLOW_COLOR TFT_YELLOW
#define BLACK_COLOR TFT_BLACK
#define WHITE_COLOR TFT_WHITE
#define BACKGROUND_COLOR TFT_BLACK

// Spherical harmonics parameters
#define RADIUS 70
#define POINT_SPACING 6
#define ROTATION_SPEED 0.02
#define ZOOM_FACTOR 120.0

// To store point coordinates for visualization
struct Point3D {
  float x, y, z;
  float r; // Radius from origin
  float value; // Value of the spherical harmonic
};

// Current spherical harmonic type
int current_harmonic = 0;
int max_harmonics = 6;

// Rotation variables
float angle_x = 0.0;
float angle_y = 0.0;
float angle_z = 0.0;

// Button pin for changing harmonics
const int BUTTON_PIN = 0; // Change to your button pin
int lastButtonState = HIGH; // Assuming pull-up resistor
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 200; // Debounce time in milliseconds

// Function prototypes
void drawSphericalHarmonic(int l, int m);
float sphericalHarmonic(int l, int m, float theta, float phi);
float associatedLegendre(int l, int m, float x);
float factorial(int n);

void setup() {
  Serial.begin(115200);
  
  // Initialize display
  tft.init();
  tft.setRotation(1); // Adjust as needed for your display
  tft.fillScreen(BACKGROUND_COLOR);
  
  // Setup button for changing harmonics
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  // Display initial info
  tft.setTextColor(WHITE_COLOR, BACKGROUND_COLOR);
  tft.setTextSize(1);
  tft.setCursor(5, 5);
  tft.println("ESP32 Spherical Harmonics Demo");
  tft.setCursor(5, 20);
  tft.println("Press button to change harmonic");
}

void loop() {
  // Check button for changing harmonics
  int reading = digitalRead(BUTTON_PIN);
  if (reading != lastButtonState && millis() - lastDebounceTime > debounceDelay) {
    if (reading == LOW) { // Button pressed
      current_harmonic = (current_harmonic + 1) % max_harmonics;
      // Clear screen for new harmonic
      tft.fillRect(0, 35, SCREEN_WIDTH, SCREEN_HEIGHT - 35, BACKGROUND_COLOR);
    }
    lastDebounceTime = millis();
  }
  lastButtonState = reading;

  // Update rotation angles
  angle_x += ROTATION_SPEED;
  angle_y += ROTATION_SPEED * 0.7;
  
  // Clear previous drawing, but keep the header
  tft.fillRect(0, 35, SCREEN_WIDTH, SCREEN_HEIGHT - 35, BACKGROUND_COLOR);
  
  // Display current harmonic info
  tft.setCursor(5, 35);
  
  // Different harmonics and SMOs
  switch (current_harmonic) {
    case 0:
      tft.println("Y(0,0) - s orbital");
      drawSphericalHarmonic(0, 0);
      break;
    case 1:
      tft.println("Y(1,0) - p_z orbital");
      drawSphericalHarmonic(1, 0);
      break;
    case 2:
      tft.println("Y(1,1) - p_x/p_y orbitals");
      drawSphericalHarmonic(1, 1);
      break;
    case 3:
      tft.println("Y(2,0) - d_z^2 orbital");
      drawSphericalHarmonic(2, 0);
      break;
    case 4:
      tft.println("Y(2,1) - d_xz/d_yz orbitals");
      drawSphericalHarmonic(2, 1);
      break;
    case 5:
      tft.println("Y(2,2) - d_xy/d_x^2-y^2 orbitals");
      drawSphericalHarmonic(2, 2);
      break;
  }
  
  delay(30); // Control animation speed
}

// Calculate and draw the selected spherical harmonic
void drawSphericalHarmonic(int l, int m) {
  const int numTheta = 30;
  const int numPhi = 30;
  Point3D points[numTheta * numPhi];
  
  int idx = 0;
  float min_value = 1000.0;
  float max_value = -1000.0;
  
  // Calculate spherical harmonic values at various points on a sphere
  for (int i = 0; i < numTheta; i++) {
    float theta = M_PI * i / (numTheta - 1);
    for (int j = 0; j < numPhi; j++) {
      float phi = 2.0 * M_PI * j / (numPhi - 1);
      
      // Calculate spherical harmonic value
      float value = sphericalHarmonic(l, m, theta, phi);
      
      // Store the min/max values for normalization
      if (value < min_value) min_value = value;
      if (value > max_value) max_value = value;
      
      // Convert to cartesian coordinates
      // Scale the radius by the harmonic value (amplify for visualization)
      float r = RADIUS * (0.5 + 0.5 * value);
      float sin_theta = sin(theta);
      
      points[idx].x = r * sin_theta * cos(phi);
      points[idx].y = r * sin_theta * sin(phi);
      points[idx].z = r * cos(theta);
      points[idx].value = value;
      points[idx].r = r;
      
      idx++;
    }
  }
  
  // Normalize the values for color mapping
  float range = max_value - min_value;
  if (range < 0.001) range = 1.0; // Prevent division by zero
  
  // Draw the points
  for (int i = 0; i < numTheta * numPhi; i++) {
    // Apply rotation
    float x = points[i].x;
    float y = points[i].y;
    float z = points[i].z;
    
    // Rotate around X axis
    float y1 = y * cos(angle_x) - z * sin(angle_x);
    float z1 = y * sin(angle_x) + z * cos(angle_x);
    
    // Rotate around Y axis
    float x2 = x * cos(angle_y) + z1 * sin(angle_y);
    float z2 = -x * sin(angle_y) + z1 * cos(angle_y);
    
    // Project 3D to 2D with perspective
    float scale = ZOOM_FACTOR / (ZOOM_FACTOR + z2);
    int screen_x = SCREEN_CENTER_X + (int)(x2 * scale);
    int screen_y = SCREEN_CENTER_Y + (int)(y1 * scale);
    
    // Skip if outside screen
    if (screen_x < 0 || screen_x >= SCREEN_WIDTH || screen_y < 0 || screen_y >= SCREEN_HEIGHT)
      continue;
    
    // Determine color based on value (positive = RED, negative = BLUE)
    uint16_t color;
    float normalized_value = (points[i].value - min_value) / range;
    if (points[i].value >= 0) {
      // Positive values: RED with intensity based on value
      uint8_t intensity = 128 + (uint8_t)(normalized_value * 127);
      color = tft.color565(intensity, 0, 0);
    } else {
      // Negative values: BLUE with intensity based on value
      uint8_t intensity = 128 + (uint8_t)((1.0 - normalized_value) * 127);
      color = tft.color565(0, 0, intensity);
    }
    
    // Draw point
    int radius = max(1, min(3, (int)(scale * 3)));
    tft.fillCircle(screen_x, screen_y, radius, color);
  }
  
  // Draw coordinate axes for reference
  int axis_length = 40;
  
  // X-axis (rotated)
  float x1 = axis_length;
  float y1 = 0;
  float z1 = 0;
  
  // Apply same rotation as points
  float y1_rot = y1 * cos(angle_x) - z1 * sin(angle_x);
  float z1_rot = y1 * sin(angle_x) + z1 * cos(angle_x);
  float x1_rot = x1 * cos(angle_y) + z1_rot * sin(angle_y);
  float z1_final = -x1 * sin(angle_y) + z1_rot * cos(angle_y);
  
  float scale = ZOOM_FACTOR / (ZOOM_FACTOR + z1_final);
  int x_end = SCREEN_CENTER_X + (int)(x1_rot * scale);
  int y_end = SCREEN_CENTER_Y + (int)(y1_rot * scale);
  
  tft.drawLine(SCREEN_CENTER_X, SCREEN_CENTER_Y, x_end, y_end, RED_COLOR);
  
  // Y-axis (rotated)
  float x2 = 0;
  float y2 = axis_length;
  float z2 = 0;
  
  y1_rot = y2 * cos(angle_x) - z2 * sin(angle_x);
  z1_rot = y2 * sin(angle_x) + z2 * cos(angle_x);
  x1_rot = x2 * cos(angle_y) + z1_rot * sin(angle_y);
  z1_final = -x2 * sin(angle_y) + z1_rot * cos(angle_y);
  
  scale = ZOOM_FACTOR / (ZOOM_FACTOR + z1_final);
  x_end = SCREEN_CENTER_X + (int)(x1_rot * scale);
  y_end = SCREEN_CENTER_Y + (int)(y1_rot * scale);
  
  tft.drawLine(SCREEN_CENTER_X, SCREEN_CENTER_Y, x_end, y_end, GREEN_COLOR);
  
  // Z-axis (rotated)
  float x3 = 0;
  float y3 = 0;
  float z3 = axis_length;
  
  y1_rot = y3 * cos(angle_x) - z3 * sin(angle_x);
  z1_rot = y3 * sin(angle_x) + z3 * cos(angle_x);
  x1_rot = x3 * cos(angle_y) + z1_rot * sin(angle_y);
  z1_final = -x3 * sin(angle_y) + z1_rot * cos(angle_y);
  
  scale = ZOOM_FACTOR / (ZOOM_FACTOR + z1_final);
  x_end = SCREEN_CENTER_X + (int)(x1_rot * scale);
  y_end = SCREEN_CENTER_Y + (int)(y1_rot * scale);
  
  tft.drawLine(SCREEN_CENTER_X, SCREEN_CENTER_Y, x_end, y_end, BLUE_COLOR);
}

// Calculate spherical harmonic Y_l^m(theta, phi)
float sphericalHarmonic(int l, int m, float theta, float phi) {
  // Simplified spherical harmonics for demonstration
  // Note: This is a simplified implementation for visualization
  
  float m_abs = abs(m);
  
  // Calculate the associated Legendre polynomial
  float legendre = associatedLegendre(l, m_abs, cos(theta));
  
  // Calculate normalization factor
  float norm = sqrt((2.0 * l + 1.0) * factorial(l - m_abs) / 
                   (4.0 * M_PI * factorial(l + m_abs)));
  
  // Calculate the angular part
  float angular;
  if (m == 0) {
    angular = 1.0;
  } else if (m > 0) {
    angular = cos(m * phi) * sqrt(2.0);
  } else { // m < 0
    angular = sin(m_abs * phi) * sqrt(2.0);
  }
  
  return norm * legendre * angular;
}

// Calculate associated Legendre polynomial P_l^m(x)
float associatedLegendre(int l, int m, float x) {
  // Simplified implementation for demonstration
  
  if (l == 0 && m == 0) {
    return 1.0;
  } else if (l == 1 && m == 0) {
    return x;
  } else if (l == 1 && m == 1) {
    return -sqrt(1.0 - x*x);
  } else if (l == 2 && m == 0) {
    return 0.5 * (3.0 * x * x - 1.0);
  } else if (l == 2 && m == 1) {
    return -3.0 * x * sqrt(1.0 - x*x);
  } else if (l == 2 && m == 2) {
    return 3.0 * (1.0 - x*x);
  }
  
  // Default fallback for unsupported cases
  return 0.0;
}

// Simple factorial function
float factorial(int n) {
  if (n <= 1) return 1.0;
  
  float result = 1.0;
  for (int i = 2; i <= n; i++) {
    result *= i;
  }
  return result;
}
