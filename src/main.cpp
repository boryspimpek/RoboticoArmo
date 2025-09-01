// The Bluetooth MAC address of the ESP32 must be uploaded 
// to the PS4 controller using the Sixaxis pairing tool.
// A0:DD:6C:85:54:9E

#include <Arduino.h>
#include <math.h>
#include <SCServo.h>
#include <PS4Controller.h>

SMS_STS st;

#define S_RXD 18
#define S_TXD 19

#define S_SCL 22
#define S_SDA 21

#define RGB_LED   23
#define NUMPIXELS 10

const float L1 = 120.0;  
const float L2 = 120.0;  
const float L3 = 110.0;  
const float DELTA_THETA = 0.0174533; 

const int SERVO_LIMITS[5][2] = {
    {0, 0},
    {1024, 3072},
    {0, 2048},
    {400, 3700},
    {600, 3500}
};

struct JointAngles {
    float theta1;
    float theta2;
    float theta3;
    float theta4;
};

struct Point3D {
    float x;
    float y;
    float z;
};

enum OrientationMode {
    ORIENTATION_DOWN,  
    ORIENTATION_FLAT   
};

// Definicje przycisków PS4
#define PS4_TRIANGLE PS4_BUTTON_TRIANGLE
#define PS4_CIRCLE PS4_BUTTON_CIRCLE
#define PS4_CROSS PS4_BUTTON_CROSS

// Zmienne globalne
String method = "wrist";
OrientationMode orientation_mode = ORIENTATION_DOWN;
Point3D current_position = {150.0, 0.0, 100.0};
const int MOVE_STEP_SMALL = 2;
const int MOVE_STEP_LARGE = 5;
int move_step = MOVE_STEP_SMALL;

String check_servo_angles(int servo_angles[4]) {
    String errors = "";
    
    for (int i = 0; i < 4; i++) {
        int servo_id = i + 1;
        int target = servo_angles[i];
        int min_angle = SERVO_LIMITS[servo_id][0];
        int max_angle = SERVO_LIMITS[servo_id][1];
        
        if (target < min_angle || target > max_angle) {
            errors += "Kąt serwa " + String(servo_id) + 
                     " poza zakresem (" + String(min_angle) + 
                     "-" + String(max_angle) + "): " + 
                     String(target) + ", ";
        }
    }
    
    if (errors.length() > 0) {
        errors.remove(errors.length() - 2);
    }
    
    return errors;
}

int rad_to_servo(float rad) {
    const int center = 2048;
    const float scale = 2048.0f / M_PI;
    return 4095 - (int)round(rad * scale + center);
}

JointAngles solve_ik_full(float x_target, float y_target, float z_target) {
    float r_target = sqrt(x_target * x_target + y_target * y_target);
    float px = r_target;
    float py = z_target;

    float d = sqrt(px * px + py * py);
    if (d > (L1 + L2 + L3)) {
        JointAngles error_angles = {NAN, NAN, NAN, NAN};
        return error_angles;
    }

    float theta1 = atan2(y_target, x_target);

    float min_cost = INFINITY;
    JointAngles best_angles;
    bool solution_found = false;

    for (float theta4_c = -M_PI; theta4_c < M_PI; theta4_c += DELTA_THETA) {
        float wrist_r = r_target - L3 * cos(theta4_c);
        float wrist_z = z_target - L3 * sin(theta4_c);

        float D = sqrt(wrist_r * wrist_r + wrist_z * wrist_z);
        if (D > (L1 + L2) || D < abs(L1 - L2)) {
            continue;
        }

        float cos_theta3 = (wrist_r * wrist_r + wrist_z * wrist_z - L1 * L1 - L2 * L2) / (2 * L1 * L2);
        if (cos_theta3 < -1.0 || cos_theta3 > 1.0) {
            continue;
        }

        float theta3 = -acos(cos_theta3);
        float k1 = L1 + L2 * cos(theta3);
        float k2 = L2 * sin(theta3);
        float theta2 = atan2(wrist_z, wrist_r) - atan2(k2, k1);
        float theta4 = theta4_c - (theta2 + theta3);

        float cost = theta3 * theta3 + theta4 * theta4;

        if (cost < min_cost) {
            min_cost = cost;
            best_angles = {theta1, theta2, theta3, theta4};
            solution_found = true;
        }
    }

    if (!solution_found) {
        JointAngles error_angles = {NAN, NAN, NAN, NAN};
        return error_angles;
    }

    return best_angles;
}

JointAngles solve_ik_wrist(float x_target, float y_target, float z_target, OrientationMode orientation_mode) {
    float r_target = sqrt(x_target * x_target + y_target * y_target);
    float px = r_target;
    float py = z_target;

    float d = sqrt(px * px + py * py);
    if (d > (L1 + L2)) {
        JointAngles error_angles = {NAN, NAN, NAN, NAN};
        return error_angles;
    }

    float theta1 = atan2(y_target, x_target);

    float cos_theta3 = (r_target * r_target + z_target * z_target - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    
    if (cos_theta3 < -1.0 || cos_theta3 > 1.0) {
        JointAngles error_angles = {NAN, NAN, NAN, NAN};
        return error_angles;
    }

    float theta3 = -acos(cos_theta3);
    
    float k1 = L1 + L2 * cos(theta3);
    float k2 = L2 * sin(theta3);
    float theta2 = atan2(z_target, r_target) - atan2(k2, k1);

    float theta4;
    if (orientation_mode == ORIENTATION_DOWN) {
        theta4 = -M_PI/2.0 - (theta2 + theta3);
    } else {
        theta4 = 0 - (theta2 + theta3);
    }

    JointAngles angles = {theta1, theta2, theta3, theta4};
    return angles;
}

Point3D find_wrist_point(float theta1, float theta2, float theta3, float theta4) {
    Point3D wrist_point;
    
    wrist_point.x = L1 * cos(theta2) * cos(theta1) + L2 * cos(theta2 + theta3) * cos(theta1);
    wrist_point.y = L1 * cos(theta2) * sin(theta1) + L2 * cos(theta2 + theta3) * sin(theta1);
    wrist_point.z = L1 * sin(theta2) + L2 * sin(theta2 + theta3);
    
    return wrist_point;
}

void moveServosSyncEx(uint8_t ids[4], int targetPos[4], u16 baseSpeed, u16 accValue) {
    s16 currentPos[4];
    u16 speed[4];
    s16 position[4];
    u8 acc[4];

    int delta[4];
    int maxDelta = 0;

    for (int i = 0; i < 4; i++) {
        currentPos[i] = st.ReadPos(ids[i]);
        delta[i] = abs(targetPos[i] - currentPos[i]);
        if (delta[i] > maxDelta) maxDelta = delta[i];
        acc[i] = (u8)accValue;
    }

    for (int i = 0; i < 4; i++) {
        if (delta[i] == 0) {
            speed[i] = 0;
        } else {
            speed[i] = (u16)((float)delta[i] / maxDelta * baseSpeed);
            if (speed[i] < 1) speed[i] = 1;
        }
        position[i] = targetPos[i];
    }

    st.SyncWritePosEx(ids, 4, position, speed, acc);
}

void move_to_point(float x, float y, float z, const String& method = "wrist", OrientationMode orientation_mode = ORIENTATION_DOWN, u16 max_speed = 1000) {
    JointAngles angles;
    
    if (method == "full") {
        angles = solve_ik_full(x, y, z);
    } else {
        angles = solve_ik_wrist(x, y, z, orientation_mode);
    }

    if (isnan(angles.theta1)) {
        Serial.println("Błąd: Nie można znaleźć rozwiązania IK dla podanych współrzędnych");
        return;
    }

    int servo_angles[4];
    servo_angles[0] = rad_to_servo(angles.theta1);
    servo_angles[1] = rad_to_servo(angles.theta2);
    servo_angles[2] = rad_to_servo(angles.theta3);
    servo_angles[3] = rad_to_servo(angles.theta4);
    
    uint8_t ids[4] = {1, 2, 3, 4};
    
    String errors = check_servo_angles(servo_angles);
    if (errors.length() > 0) {
        Serial.print("Błędy: ");
        Serial.println(errors);
        return;
    }
    
    moveServosSyncEx(ids, servo_angles, max_speed, 50);
}

Point3D process_PS4_input(Point3D current_pos) {
    Point3D new_pos = current_pos;
    
    int ly = PS4.data.analog.stick.ly; // -128 (GÓRA) do 127 (DÓŁ)
    int lx = PS4.data.analog.stick.lx; // -128 (LEWO) do 127 (PRAWO)
    int ry = PS4.data.analog.stick.ry;
    
    // Ruch w osi X (lewy analog góra/dół)
    if (abs(ly) > 20) { // Dead zone
        new_pos.x += (ly / 127.0) * move_step;
    }
    
    // Ruch w osi Y (lewy analog lewo/prawo)
    if (abs(lx) > 20) {
        new_pos.y += (-lx / 127.0) * move_step;
    }
    
    // Ruch w osi Z (prawy analog góra/dół) pozostaje bez zmian
    if (abs(ry) > 20) {
        new_pos.z += (ry / 127.0) * move_step;
    }
    
    return new_pos;
}
void onConnect() {
    Serial.println("PS4 controller connected");
}

void onDisconnect() {
    Serial.println("PS4 controller disconnected");
}

void setup() {
    Serial.begin(115200);
    Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
    st.pSerial = &Serial1;
    
    PS4.attachOnConnect(onConnect);
    PS4.attachOnDisconnect(onDisconnect);
    PS4.begin(); 
    
    delay(1000);
    Serial.println("Inicjalizacja zakończona");
    
    move_to_point(current_position.x, current_position.y, current_position.z, 
                 method.c_str(), orientation_mode, 500);
}

void loop() {
    if (!PS4.isConnected()) {
        delay(1000);
        return;
    }
    
    static unsigned long lastPrintTime = 0;
    static int lastTriangleState = 0;
    static int lastCircleState = 0;
    static int lastCrossState = 0;
    
    int triangle = PS4.data.button.triangle;
    int circle = PS4.data.button.circle;
    int cross = PS4.data.button.cross;
    
    if (triangle && !lastTriangleState) {
        orientation_mode = ORIENTATION_DOWN;
        move_to_point(current_position.x, current_position.y, current_position.z, 
                     method.c_str(), orientation_mode);
        Serial.println("Orientacja: w dół");
    }
    
    if (circle && !lastCircleState) {
        orientation_mode = ORIENTATION_FLAT;
        move_to_point(current_position.x, current_position.y, current_position.z, 
                     method.c_str(), orientation_mode);
        Serial.println("Orientacja: pozioma");
    }
    
    if (cross && !lastCrossState) {
        method = (method == "full") ? "wrist" : "full";
        move_step = (method == "wrist") ? MOVE_STEP_SMALL : MOVE_STEP_LARGE;
        
        if (method == "wrist") {
            JointAngles angles = solve_ik_full(current_position.x, current_position.y, current_position.z);
            Point3D wrist_point = find_wrist_point(angles.theta1, angles.theta2, angles.theta3, angles.theta4);
            move_to_point(wrist_point.x, wrist_point.y, wrist_point.z, "wrist");
            current_position = wrist_point;
        }
        Serial.print("Zmieniono tryb na: ");
        Serial.print(method);
        Serial.print(", krok: ");
        Serial.println(move_step);
    }
    
    lastTriangleState = triangle;
    lastCircleState = circle;
    lastCrossState = cross;
    
    Point3D new_position = process_PS4_input(current_position);
    
    if (new_position.x != current_position.x || 
        new_position.y != current_position.y || 
        new_position.z != current_position.z) {
        
        move_to_point(new_position.x, new_position.y, new_position.z, 
                     method.c_str(), orientation_mode);
        
        current_position = new_position;
        
        if (millis() - lastPrintTime > 500) {
            Serial.print("Position: (");
            Serial.print(current_position.x);
            Serial.print(", ");
            Serial.print(current_position.y);
            Serial.print(", ");
            Serial.print(current_position.z);
            Serial.print("), Method: ");
            Serial.print(method);
            Serial.print(", Orientation: ");
            Serial.print((orientation_mode == ORIENTATION_DOWN) ? "down" : "flat");
            Serial.print(", Step: ");
            Serial.println(move_step);
            
            lastPrintTime = millis();
        }
    }
    
    delay(10);
}