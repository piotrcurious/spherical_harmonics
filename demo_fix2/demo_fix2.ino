#include <TFT_eSPI.h>
#include <SPI.h>
#include <math.h>
#include <stdlib.h> // Required for atof and atoi

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

// Spherical harmonics parameters - Configurable via Serial
float RADIUS = 70.0;
float POINT_SPACING = 6.0;
float ROTATION_SPEED = 0.02;
float ZOOM_FACTOR = 120.0;

const int numTheta = 18;
const int numPhi = 18;

// To store point coordinates for visualization
struct Point3D {
  float x, y, z;
  float r; // Radius from origin
  float value; // Value of the spherical harmonic
};

// Current spherical harmonic type - Now defined by l and m
int current_l = 0;
int current_m = 0;
int max_l_display = 3; // Example: Limit display up to l=3 for demo, adjust as needed

// Rotation variables
float angle_x = 0.0;
float angle_y = 0.0;
float angle_z = 0.0;

// Button pin for (basic) harmonic change - Still functional, now cycles through l values
const int BUTTON_PIN = 22;
int lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 100;

// Function prototypes
void drawSphericalHarmonic(int l, int m);
float sphericalHarmonic(int l, int m, float theta, float phi);
float associatedLegendre(int l, int m, float x);
float factorial(int n);
void processSerialCommand(); // New function to handle serial commands

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect. Needed for native USB on some systems
  }

  // Initialize display
  tft.init();
  tft.setRotation(1); // Adjust as needed for your display
  tft.fillScreen(BACKGROUND_COLOR);

  // Setup button for basic harmonic change
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Display initial info on TFT
  tft.setTextColor(WHITE_COLOR, BACKGROUND_COLOR);
  tft.setTextSize(1);
  tft.setCursor(5, 5);
  tft.println("ESP32 Spherical Harmonics Demo");
  tft.setCursor(5, 20);
  tft.println("Press button to change L"); // Button now changes L

  // Print instructions to Serial Monitor
  Serial.println("\nESP32 Spherical Harmonics Demo - Serial Control Enabled");
  Serial.println("----------------------------------------------------");
  Serial.println("Use the serial monitor to adjust parameters.");
  Serial.println("Send commands in the format: parameter=value");
  Serial.println("Available parameters:");
  Serial.println("  l=[int]         - Spherical harmonic degree (l >= 0, e.g., l=3)");
  Serial.println("  m=[int]         - Spherical harmonic order (-l <= m <= l, e.g., m=2)");
  Serial.println("  speed=[float]     - Rotation speed (e.g., speed=0.01)");
  Serial.println("  zoom=[float]      - Zoom factor (e.g., zoom=150)");
  Serial.println("  radius=[float]    - Overall radius (e.g., radius=80)");
  Serial.println("  spacing=[float]   - Point spacing (e.g., spacing=5)");
  Serial.println("----------------------------------------------------");
  Serial.println("Current parameters:");
  Serial.print("  l="); Serial.println(current_l);
  Serial.print("  m="); Serial.println(current_m);
  Serial.print("  speed="); Serial.println(ROTATION_SPEED);
  Serial.print("  zoom="); Serial.println(ZOOM_FACTOR);
  Serial.print("  radius="); Serial.println(RADIUS);
  Serial.print("  spacing="); Serial.println(POINT_SPACING);
  Serial.println("----------------------------------------------------\n");
}

void loop() {
  // Check for serial commands and process them
  processSerialCommand();

  // Check button for basic harmonic change (cycles through L values)
  int reading = digitalRead(BUTTON_PIN);
  if (reading != lastButtonState && millis() - lastDebounceTime > debounceDelay) {
    if (reading == LOW) { // Button pressed
      current_l++;
      if (current_l > max_l_display) {
        current_l = 0;
      }
      current_m = 0; // Reset m to 0 when l changes for simplicity
      // Clear screen for new harmonic
      tft.fillRect(0, 35, SCREEN_WIDTH, SCREEN_HEIGHT - 35, BACKGROUND_COLOR);
      Serial.print("L changed by button to: "); Serial.print("l="); Serial.print(current_l); Serial.print(", m="); Serial.println(current_m);
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
  tft.print("Y("); tft.print(current_l); tft.print(","); tft.print(current_m); tft.println(")"); // Display Y(l,m)

  drawSphericalHarmonic(current_l, current_m);


  delay(30); // Control animation speed
}

// Function to process serial commands
void processSerialCommand() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim(); // Remove leading/trailing whitespace

    int equalSignIndex = command.indexOf('=');
    if (equalSignIndex != -1) {
      String parameter = command.substring(0, equalSignIndex);
      String valueStr = command.substring(equalSignIndex + 1);

      parameter.trim();
      valueStr.trim();

      if (parameter.equalsIgnoreCase("l")) {
        int value = valueStr.toInt();
        if (value >= 0 && value <= 10) { // Example upper limit for l, adjust if needed
          current_l = value;
          current_m = min(current_m, current_l); // Keep m valid if l is reduced
          tft.fillRect(0, 35, SCREEN_WIDTH, SCREEN_HEIGHT - 35, BACKGROUND_COLOR); // Clear for new harmonic
          Serial.print("Spherical harmonic degree (l) set to: "); Serial.println(current_l);
        } else {
          Serial.println("Invalid l value. Must be between 0 and 10 (example limit).");
        }
      } else if (parameter.equalsIgnoreCase("m")) {
        int value = valueStr.toInt();
        if (value >= -current_l && value <= current_l) { // Validate m based on current l
          current_m = value;
          tft.fillRect(0, 35, SCREEN_WIDTH, SCREEN_HEIGHT - 35, BACKGROUND_COLOR); // Clear for new harmonic
          Serial.print("Spherical harmonic order (m) set to: "); Serial.println(current_m);
        } else {
          Serial.println("Invalid m value. Must be between -" + String(current_l) + " and " + String(current_l) + " for current l=" + String(current_l));
        }
      }  else if (parameter.equalsIgnoreCase("speed")) {
        float value = valueStr.toFloat();
        ROTATION_SPEED = value;
        Serial.print("Rotation speed set to: "); Serial.println(ROTATION_SPEED);
      } else if (parameter.equalsIgnoreCase("zoom")) {
        float value = valueStr.toFloat();
        ZOOM_FACTOR = value;
        Serial.print("Zoom factor set to: "); Serial.println(ZOOM_FACTOR);
      } else if (parameter.equalsIgnoreCase("radius")) {
        float value = valueStr.toFloat();
        RADIUS = value;
        Serial.print("Radius set to: "); Serial.println(RADIUS);
      } else if (parameter.equalsIgnoreCase("spacing")) {
        float value = valueStr.toFloat();
        POINT_SPACING = value;
        Serial.print("Point spacing set to: "); Serial.println(POINT_SPACING);
      } else {
        Serial.print("Unknown parameter: "); Serial.println(parameter);
        Serial.println("Available parameters are: l, m, speed, zoom, radius, spacing");
      }
    } else {
      Serial.println("Invalid command format. Use parameter=value");
    }
  }
}


// Calculate and draw the selected spherical harmonic
void drawSphericalHarmonic(int l, int m) {

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
    int radius = max(1, min(3, (int)(scale * POINT_SPACING/2.0))); // radius adjusted by spacing, smaller spacing -> smaller points
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

// Calculate spherical harmonic Y_l^m(theta, phi) - REAL VERSION
float sphericalHarmonic(int l, int m, float theta, float phi) {
    float m_abs = abs(m);
    float legendre = associatedLegendre(l, m_abs, cos(theta));
    float norm = sqrt((2.0 * l + 1.0) * factorial(l - m_abs) /
                       (4.0 * M_PI * factorial(l + m_abs)));
    float angular;

    if (m == 0) {
        angular = 1.0; // Zonal harmonics (m=0) are real and azimuthally independent
    } else if (m > 0) {
        angular = sqrt(2.0) * cos(m * phi); // Real part combination
    } else { // m < 0
        angular = sqrt(2.0) * sin(m_abs * phi); // Imaginary part combination, adjusted for real SH
    }

    return norm * legendre * angular;
}


// Calculate associated Legendre polynomial P_l^m(x) for m >= 0
float associatedLegendre(int l, int m, float x) {
  if (m < 0 || m > l) {
    return 0.0; // Definition: P_l^m = 0 if m < 0 or m > l
  }
  if (m == 0) {
    if (l == 0) return 1.0;
    if (l == 1) return x;
    // Recurrence relation for P_l^0(x)
    float P_l_minus_1 = associatedLegendre(l - 1, 0, x);
    float P_l_minus_2 = associatedLegendre(l - 2, 0, x);
    return ((2.0 * l - 1.0) * x * P_l_minus_1 - (l - 1.0) * P_l_minus_2) / l;

  } else if (m == l) {
    if (l == 0) return 1.0; // Though m should be > 0 for this branch, to be safe
    if (l == 1) return -sqrt(1.0 - x*x); // or sqrt(1-x^2) depending on phase convention
    // Formula for P_l^l(x)
    return -(2.0 * l - 1.0) * sqrt(1.0 - x*x) * associatedLegendre(l - 1, l - 1, x);

  } else if (m < l) {
    // Recurrence relation for P_l^m(x)
    float P_l_minus_1_m = associatedLegendre(l - 1, m, x);
    float P_l_minus_2_m = associatedLegendre(l - 2, m, x);
    return ((2.0 * l - 1.0) * x * P_l_minus_1_m - (l + m - 1.0) * P_l_minus_2_m) / (l - m);
  }
  return 0.0; // Should not reach here, but for safety
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
