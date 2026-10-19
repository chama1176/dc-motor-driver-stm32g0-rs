# モータ PID 制御 実装項目・スコープ案

## 目的

- [x] 1 ms 周期の `App::control()` でモータ状態を取得する。
- [x] 取得値を `ControlTableData` の Present 系項目へ反映する。
- [x] `ControlTableData` の指令値、ゲイン、制限値に基づいて PID 演算を行う。
- [x] PID 出力を安全な PWM 値へ変換し、DC モータを駆動する。

## 初期スコープ案

- [x] 第1段階は位置制御モードに限定する。
  - `GoalPosition` を目標値にする。
  - `PositionPGain`、`PositionIGain`、`PositionDGain` を使用する。
  - PID 出力を `PWMLimit` で制限し、正負を回転方向へ変換する。
- [x] PWM 直接指令モードを追加する。
  - `GoalPWM` を制限したうえで、そのまま PWM 出力へ変換する。
- [x] `TorqueEnable == 0` の場合は PWM を 0 にし、積分値と前回誤差をリセットする。
- [x] 未対応の `OperatingMode` ではモータを停止する。
- [ ] 未対応モードを通信側から判別できるステータスを定義する。
- [x] 第2段階の速度 PI 制御を追加する（設定・実行方法はREADMEを参照）。
  - `GoalVelocity`、`VelocityPGain`、`VelocityIGain` を使用する。
- [x] 電流制御および電流ベース位置制御は、電流計測経路がないため対象外とする。

## フィードバック値とテーブル更新

- [x] TIM3 のエンコーダカウントを取得するインターフェースを `App` に渡す。
- [x] カウンタのオーバーフローを考慮し、起動位置を原点とした連続符号付き位置を算出する。
- [x] 前回位置との差と制御周期から速度を算出する。
- [x] 速度PIでは10 Hz、位置D項では20 Hzの1次ローパスフィルタを使う。フィルタ状態は共用し、モードで周波数を切り替える。
- [x] 各制御周期で次の項目を更新する。
  - `PresentPosition`: 現在位置。
  - `PresentVelocity`: 現在速度。
  - `PresentPWM`: 実際に指令した符号付き PWM。
  - `RealtimeTick`: 経過時間。16 bit のラップを許容する。
  - `Moving`: 速度が `MovingThreshold` を超える場合に 1 とする。
- [ ] `MovingStatus` と `HardwareErrorStatus` は意味とビット割り当てを決めてから更新する。
- [x] `PresentCurrent`、`PresentInputVoltage`、`PresentTemperature` は計測手段を追加するまで更新対象外とする。

## PID 演算

- [x] PID の積分値と速度フィルタ状態を専用構造体に保持する。位置D項はフィルタ済み速度を使う。
- [x] `App::control()` を `&mut self` で実行できるようにする。
- [x] 制御周期 `dt` を `App` のパラメータとし、初期値 `0.001 s` を TIM14 の実設定と一致させる。
- [x] 積分項にアンチワインドアップを設ける。
- [x] 微分項は目標値の急変によるキックを避けるため、測定値の差分を使用する。
- [x] PWM 出力を `0.0..=1.0` にクランプし、方向と絶対値に分けて `DcMotorDriver::set_pwm()` へ渡す。
- [x] ゲインが実行中に書き換えられても、1 制御周期では同一スナップショットを使用する。

## 初期化と安全動作

- [x] `ControlTableData` は 0 埋めで生成し、App初期化でゲイン・制限値を設定する。Torque Off で起動する。
- [x] `MinPositionLimit` と `MaxPositionLimit` を目標位置へ適用する。
- [ ] `BusWatchdog` のタイムアウト検出と、タイムアウト時の停止・復帰条件を定義する。
- [x] エンコーダ値を取得できない場合は PWM を 0 にし、PID 状態をリセットする。
- [x] `DcPwm::enable()` と `disable()` を実装し、停止時は PWM 出力を無効化してコーストさせる。
- [x] PWM デューティ計算を飽和させ、範囲外値や `NaN` がタイマレジスタへ渡らないようにする。
- [x] TIM14 を `App` 初期化後に開始し、停止せず周期動作させる。
- [ ] 1 ms 内に制御処理が完了することを GPIO または計測機能で確認する。

## 構成案

- [x] ハードウェア固有のエンコーダ読み出しは `dc_motor_driver_stm32g0.rs` に置く。
- [x] 単位換算と PID 演算を、実機なしでテストできる `src/control.rs` へ分離する。
  - 安定後に `../motml-rs` へ移すか検討する。
- [x] `App` はテーブルの読み取り、制御器の実行、Present 系項目の更新をまとめる。
- [x] UART 割り込みによるテーブル書き換えと制御割り込みが競合しないよう、既存のクリティカルセクション内で整合性を保つ。
- [x] sibling crate の API は変更せず、既存の `ControlTableData::read/modify` とローカルのアダプターで実装する。

## テスト・受け入れ条件

- [x] PID 単体テスト: 比例、積分、微分、正逆転、飽和、アンチワインドアップ、リセット。
- [x] エンコーダ差分テスト: 正転、逆転、16 bit カウンタの上下オーバーフロー。
- [x] テーブル変換テスト: 位置・速度単位、位置制限、PWM raw 値の境界値。
- [ ] 実機で `TorqueEnable`、`GoalPosition`、各ゲインを通信から変更し、応答を確認する。
- [ ] `PresentPosition`、`PresentVelocity`、`PresentPWM` を通信で読み出し、実測と整合することを確認する。
- [ ] 通信停止、エンコーダ異常、未対応モードでモータが安全に停止することを確認する。

## 今回の対象外候補

- [ ] 加減速プロファイル（現行 API の `ProfileAccleration`、`ProfileVelocity`）と軌道生成。
- [ ] 電流、電圧、温度の計測および保護。
- [ ] 不揮発メモリへの設定保存。
- [ ] PID オートチューニング。
- [ ] Dynamixel 製品との制御特性、単位、全ステータスビットの完全互換。

## 決定が必要な項目

- [x] `OperatingMode` は位置制御を 3、PWM 制御を 16 とし、第1段階の閉ループ制御は位置 PID に限定する。
  - 位置制御の原点は起動時のエンコーダ位置とする。
- [x] エンコーダの 1 回転当たりカウント、4逓倍の扱い、減速比、正転方向、原点。
  - モーター軸11 pulse/rev、ハードウェア4逓倍後44 count/rev、減速比18.75:1、出力軸825 count/rev。原点は起動時の位置。
- [x] `GoalPosition` と `PresentPosition` はエンコーダカウントを直接使う（825 count/output rev）。
- [x] `GoalVelocity` と `PresentVelocity` はエンコーダカウント/秒とし、速度モードでは10 Hzのローパスフィルタを使用する。
- [x] 位置ゲインは XC330 を参考に、P=`raw/128`、I=`raw/65536`、D=`raw/16` とする。
- [x] `PWMLimit` と `GoalPWM` の最大 raw 値を 885 とし、885 をデューティ `1.0` に換算する。
- [ ] 位置範囲外、通信タイムアウト、エンコーダ異常時の停止方式とエラービット。
  - 一旦放置
- [x] 位置P/I/D=8192/11830/32000、速度P/I=160/1640を設定し、起動時の `TorqueEnable` は Off とする。
- [x] 第1段階は位置 PID を PWM へ直接接続する。将来のカスケード制御は速度 PI 実装時に再検討する。
