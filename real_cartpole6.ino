#include <M5StickC.h>

int q_table[36] = {5 ,14 ,19 ,3 ,3 ,5 ,7 ,20 ,20 ,2 ,16 ,18 ,6 ,1 ,19 ,19 ,6 ,4 ,5 ,18 ,13 ,8 ,10 ,4 ,4 ,11 ,10 ,7 ,0 ,4 ,15 ,13 ,2 ,0 ,3 ,14};
                //{0 ,1 ,1 ,1 ,0 ,2 ,0 ,1 ,1 ,1 ,0 ,2 ,2 ,1 ,1 ,2 ,2 ,0 ,2 ,1 ,1 ,0 ,2 ,1 ,0 ,1 ,2 ,2 ,2 ,0 ,2 ,2 ,2 ,2 ,0 ,1};
                //{0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0};
                //{1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1};
                //{1, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 0, 1, 1, 1, 1};
                //{1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1};
                //{0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1, 1, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0};
                //{0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1};
                //{0, 0, 0, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 0, 1};
int idx1, idx2, idx;

float roll, pitch, yaw;
float gyroX, gyroY, gyroZ;

float power = 0;
float dt, preTime;

bool started = false;

float target = -98.9;
float target2 = 0.0;
float now_r, now_g;
bool led = false;

float pole_angle, pole_vel;

void wheel(uint8_t val) {
    Wire.beginTransmission(0x38);
    Wire.write(-0x00);
    Wire.write(-val);
    Wire.endTransmission();
    Wire.beginTransmission(0x38);
    Wire.write(0x01);
    Wire.write(val);
    Wire.endTransmission();
}

void setup() {
  // put your setup code here, to run once:
    M5.begin();
    Wire.begin(0, 26);
    M5.Lcd.setRotation(2);
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextSize(1);
    M5.Axp.ScreenBreath(8);
    M5.IMU.Init();
    wheel(0);
    pinMode(GPIO_NUM_10, OUTPUT);
    digitalWrite(GPIO_NUM_10, HIGH);

}

void loop() {
  // put your main code here, to run repeatedly:
    M5.update();
    if (M5.BtnB.wasReleased()) {
        // ボタンBを押すと再起動
        esp_restart();
    }

    if (M5.BtnA.wasReleased()) {
        // BtnAを押して3秒後にモーター駆動開始
        Serial.println("BtnA.wasReleased() == TRUE");

        M5.Lcd.setCursor(0, 0);
        M5.Lcd.print("3");
        delay(1000);
        M5.Lcd.setCursor(0, 0);
        M5.Lcd.print("2");
        delay(1000);
        M5.Lcd.setCursor(0, 0);
        M5.Lcd.print("1");
        // ボタンを押したら直立させて待ち残り1秒になったら現在の角度を基準に設定する
        float temp = 0.0;
        float temp2 = 0.0;
        for (int i = 0; i < 10; i++) {
            M5.IMU.getAhrsData(&pitch, &roll, &yaw);
            M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);
            temp += roll;
            temp2 += gyroX;
            delay(10);
        }
        target = temp / 10.0;
        target2 = temp2 / 10.0;
        delay(800);
        M5.Lcd.setCursor(0, 0);
        M5.Lcd.print("0");
        preTime = micros();
        started = true;
    }
    M5.IMU.getAhrsData(&pitch, &roll, &yaw);
    M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);

    //Serial.printf("%8.1f,%8.1f,%8.1f\n", roll, pitch, yaw);
    //Serial.printf("%6.2f,%6.2f,%6.2f\n", gyroX, gyroY, gyroZ);
    //drawScreen();
    delay(1);
    if (!started) {
        return;
    }
    now_r = target - roll;
    now_g = target2 - gyroX;
    dt = (micros() - preTime) / 1000000; // 処理時間を求める
    preTime = micros(); // 処理時間を記録

    //if (-1 < now_r && now_r < 1) {
        // +-1以内は制御しない
        //power = 0;
        //wheel(0);
        //return;
    //}
    //if (now_r < -28 || 28 < now_r) {
        // +-40度を超えたら倒れたとみなす
        //power = 0;
        //wheel(power);
        //return;
    //}
    pole_angle = now_r / 180 * PI;
    pole_vel = now_g / 180 * PI;
    Serial.printf("%6.2f,%6.2f\n", pole_angle, pole_vel);

    if (-0.5 <= pole_angle && pole_angle < -0.333) {
      idx1 = 0;
    }
    else if (-0.333 <= pole_angle && pole_angle < -0.166) {
      idx1 = 1;
    }
    else if (-0.166 <= pole_angle && pole_angle < 0.0 ) {
      idx1 = 2;
    }
    else if (0.0 <= pole_angle && pole_angle < 0.167) {
      idx1 = 3;
    }
    else if (0.167 <= pole_angle && pole_angle < 0.334) {
      idx1 = 4;
    }
    else if (0.334 <= pole_angle && pole_angle <= 0.5) {
      idx1 = 5;
    }
    


    if (pole_vel < -1.333) {
      idx2 = 0;
    }
    else if (-1.333 <= pole_vel && pole_vel < -0.666) {
      idx2 = 6;
    }
    else if (-0.666 <= pole_vel && pole_vel < 0.0) {
      idx2 = 12;
    }
    else if (0.0 <= pole_vel && pole_vel <= 0.666) {
      idx2 = 18;
    }
    else if (0.666 < pole_vel && pole_vel < 1.334) {
      idx2 = 24;
    }
    else if (1.334 <= pole_vel ) {
      idx2 = 30;
    }
    

    idx = idx2 + idx1;

    //Serial.printf("%d, %d, %d\n", idx1, idx2, idx);

    if (q_table[idx] == 1) {
      power = 60 * 0.1;
    }
    else if (q_table[idx] == 2) {
      power = 60 * 0.2;
    }
    else if (q_table[idx] == 3) {
      power = 60 * 0.3;
    }
    else if (q_table[idx] == 4) {
      power = 60 * 0.4;
    }
    else if (q_table[idx] == 5) {
      power = 60 * 0.5;
    }
    else if (q_table[idx] == 6) {
      power = 60 * 0.6;
    }
    else if (q_table[idx] == 7) {
      power = 60 * 0.7;
    }
    else if (q_table[idx] == 8) {
      power = 60 * 0.8;
    }
    else if (q_table[idx] == 9) {
      power = 60 * 0.9;
    }
    else if (q_table[idx] == 10) {
      power = 60;
    }
    else if (q_table[idx] == 11) {
      power = -60 * 0.1;
    }
    else if (q_table[idx] == 12) {
      power = -60 * 0.2;
    }
    else if (q_table[idx] == 13) {
      power = -60 * 0.3;
    }
    else if (q_table[idx] == 14) {
      power = -60 * 0.4;
    }
    else if (q_table[idx] == 15) {
      power = -60 * 0.5;
    }
    else if (q_table[idx] == 16) {
      power = -60 * 0.6;
    }
    else if (q_table[idx] == 17) {
      power = -60 * 0.7;
    }
    else if (q_table[idx] == 18) {
      power = -60 * 0.8;
    }
    else if (q_table[idx] == 19) {
      power = -60 * 0.9;
    }
    else if (q_table[idx] == 20) {
      power = -60;
    }
    else if (q_table[idx] == 0) {
      power = 0;
    }

   if (-0.5 > pole_angle) {
      power = -100;
    }
   if (0.5 < pole_angle) {
      power = 100;
    }

    


    // モーターの出力を決定する
    wheel(power);
}
