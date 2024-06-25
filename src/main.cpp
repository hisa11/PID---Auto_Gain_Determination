#include "mbed.h"
#include "PID.hpp"

// ピンの定義
CAN can(PA_11, PA_12);  // CAN通信ピン
BufferedSerial pc(USBTX, USBRX,115200);  // シリアル通信

// 定数
const int TARGET_POSITION = 1000;  // 目標位置
const float DT = 0.01;  // サンプリングタイム（秒）

// PID制御オブジェクト
PID pid(1.0, 0.0, 0.0, DT);  // 初期ゲイン

// エンコーダカウント変数
volatile int encoder_count = 0;

// RoboMaster C610制御関数
void send_can_message(int16_t motor_output) {
    // CANメッセージの作成
    CANMessage msg;
    msg.id = 0x200;  // 制御ID（例：0x200はモータ制御）
    msg.len = 8;
    msg.data[0] = motor_output >> 8;
    msg.data[1] = motor_output & 0xFF;
    // 他のモータが無い場合は0で埋める
    for (int i = 2; i < 8; i++) {
        msg.data[i] = 0;
    }
    // メッセージを送信
    can.write(msg);
}

// エンコーダのデータを処理する関数（仮）
void receive_can_message() {
    CANMessage msg;
    if (can.read(msg)) {
        // メッセージのIDとデータ長を確認
        if (msg.id == 0x201 && msg.len == 8) {
            // エンコーダデータの取得（仮の処理、実際にはデータフォーマットに合わせて調整）
            encoder_count = (msg.data[0] << 8) | msg.data[1];
        }
    }
}

int main() {

    while (true) {
        // エンコーダデータの受信
        receive_can_message();

        // PID制御
        float control = pid.calculate(TARGET_POSITION, encoder_count);

        // モータ制御（制御値を適切な範囲にクリップ）
        int16_t motor_output = control * 16384;  // 16384はモータの最大制御値
        send_can_message(motor_output);

        // 結果をシリアル出力
        printf("Kp: %.2f, Ki: %.2f, Kd: %.2f, Control: %.2f, Position: %d\n",
                  pid.getSampleTime(), control, encoder_count);

        // 調整のための短い遅延
        ThisThread::sleep_for(DT);
    }
}
