# デブリ周回デモ（Python直接実行版）

従来のament_python版を、Pythonから直接起動する構成へ置き換えました。
このデモには`package.xml`、`setup.py`、amentのlaunchファイルはありません。
ソースから`python3 run.py`でROS 2ノードを起動します。デモ自身のcolconビルドやpip installは不要です。
ROS 2 Jazzy、現在の`pylon_interfaces`、起動済みのPyLoN bridgeは必要です。
`pylon_vehicle_control`や既存デモのインストールは不要です。

元のLiDAR＋6軸IMU推定、後方を含む探索、デタンブル、半径15 mへの接近、RCS周回、
36度ごとのPNG＋計測JSON保存を引き継いでいます。既定では2周目以降も撮影を続けます。
Ground Truthは認識・誘導・制御のいずれも購読しません。
初期IMU姿勢に固定した座標系、自由落下中の無推力対象という仮定も同じです。

## 準備と起動

機体は元デモの[`PyLoN Debris Orbiter.craft`](craft/)を使います。
[機体のコピー・読み込み手順](craft/README.md)と[軌道配置・分離・開始条件のガイド](https://github.com/PyLoN-sim/docs/blob/main/demos/debris-orbit.md)を確認してください。
機体センサーIDは`front_lidar`、`orbit_camera`です。

NumPy、SciPy、PyYAMLとROSの標準メッセージ・TF・点群ライブラリを用意します。

```bash
sudo apt install python3-numpy python3-scipy python3-yaml \
  ros-jazzy-sensor-msgs-py ros-jazzy-tf2-ros-py ros-jazzy-nav-msgs \
  ros-jazzy-visualization-msgs
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

本体のROSパッケージを準備するには、demosルートで`./pylon_demo_debris_orbit/build.sh`を実行します。
このスクリプトは本体のinterfaces・bridge・vehicle_controlを同期・ビルドし、デモのPythonソースも同期します。
デモ自身はcolconの対象になりません。

APIを更新した後は本体側で`pylon_interfaces`とbridgeを再ビルド・sourceしてください。
古い`ControlAuthorityState`（`generation`がない型）のままなら、起動時に説明を表示して終了します。

ターミナルAで、通常のbridgeを1つ起動します。

```bash
ros2 run pylon_bridge udp_bridge \
  --host 127.0.0.1 --port 49010 --disable-ground-truth
```

ターミナルBもROSと本体ワークスペースをsourceしてから、demosルートで実行します。
初めは推定と表示だけで点群・相対位置・相対速度を確認します。

```bash
python3 pylon_demo_debris_orbit/run.py --no-enabled --no-controller --instance orbit_a --rviz
```

確認用プロセスをCtrl-Cで終了し、機体を安定させてから制御を開始します。

```bash
python3 pylon_demo_debris_orbit/run.py --enabled --instance orbit_a --orbit-radius 15 --rviz
```

`./pylon_demo_debris_orbit/run.sh`も同じ入口です。`--help`で全引数を確認できます。
設定は[`config/pylon_demo_debris_orbit.yaml`](config/pylon_demo_debris_orbit.yaml)。元デモと同じ3つのROS parameterセクションを持ちます。
`--config /path/to/custom.yaml`で別の設定を使えます。`--no-enabled`はYAMLが有効でも制御を無効にします。
`--lidar-sensor-id`、`--camera-sensor-id`、`--prefix`、`--output-directory`も変更できます。
`--ros-args`以降はrclpyへ渡します。

制御デモは、同じ機体に対して同時に動かさないでください。
`--instance`は出力namespaceと撮影保存先の識別名で、機体の選択には使いません。
操作対象は常にKSPのactive_vesselです。

## モジュール構成

| モジュール | 責務 |
|---|---|
| [`recognition.py`](debris_orbit/recognition.py) | 認識ノード。位置・速度・姿勢を推定してtarget/navigationを配信 |
| [`lidar_pipeline.py`](debris_orbit/lidar_pipeline.py) | 点群とIMUの時刻照合、取付TF変換、推定Pose/Twist生成 |
| [`inertial.py`](debris_orbit/inertial.py)、[`perception.py`](debris_orbit/perception.py) | ジャイロ・比力の積分、SciPyクラスタ抽出、相対運動Kalman filter |
| [`attitude.py`](debris_orbit/attitude.py) | LiDARの取付姿勢を考慮した目標姿勢、姿勢／角速度loop、デタンブルトルク |
| [`thrust.py`](debris_orbit/thrust.py) | 位置・速度誤差から目標推力を計算し、body軸のWrenchへ組み立て |
| [`guidance.py`](debris_orbit/guidance.py) | 探索→接近→周回の進行、軌道上の位置・速度目標、撮影 |
| [`controller.py`](debris_orbit/controller.py)、[`authority.py`](debris_orbit/authority.py) | ROS入出力、共有操作権の取得・更新・解放、鮮度・世代確認 |
| [`runner.py`](debris_orbit/runner.py) | ローカルYAML読込み、Topic接続、3ノードの起動・終了 |

通常は3ノードを1つのPythonプロセス内で動かします。
別プロセスで実行したい場合は、各ターミナルで同じ設定・instanceを使います。

```bash
python3 pylon_demo_debris_orbit/run.py --node recognition --instance orbit_a
python3 pylon_demo_debris_orbit/run.py --node guidance --enabled --instance orbit_a
python3 pylon_demo_debris_orbit/run.py --node controller --instance orbit_a
```

`attitude.py`と`thrust.py`の計算部分はROSをimportしないため、Pythonから直接呼び出せます。
周回用の並進目標は`guidance.py`、力[N]への変換は`thrust.py`です。

## 更新後のAPIと停止動作

操作権は`STATE_PLAYER` / `STATE_PYLON`の共有状態を見ます。
他ノードのcontroller/lease IDが観測されても、IDの一致を所有条件にしません。
指令のcontroller/lease ID・sequenceは省略し、bridgeの補完に任せます。
`vessel_id`だけは古い機体への指令を拒否するガードとして添えます。
観測の`vessel_id`・`generation`をlifecycleと照合し、同じ機体への再接続も別セッションとして扱います。

Wrenchは`base_link`（+X前、+Y左、+Z上）の力[N]とトルク[N·m]です。
姿勢上限、指令ramp、連続噴射limit時のゼロ指令cooldownは共通制御器と同じ計算を引き継いでいます。
無効状態では操作権を取得せず、別ノードの操作権も解放しません。
実行中の機体切替・操作権喪失・入力欠測は停止を保持し、通信復帰だけでは再開しません。
再開は3ノード全体を再起動してください。0.5秒を超えるIMU欠落も再起動が必要です。

Ctrl-C / SIGTERMではゼロWrenchを送り、操作権を解放してからROSを終了します。
共有操作権なので、このデモが取得した操作権の解放はPyLoN全体をプレイヤーへ戻します。

## 状態・撮影結果

Topicは元デモと同じ`/ksp_vessel/demos/debris_orbit/<instance>/`です。
`target`、`navigation/pose`、`navigation/twist`、`navigation/twist_body`、`setpoint`、
`status`、`estimator_status`、`controller_status`、RViz表示を配信します。

```bash
ros2 topic echo /ksp_vessel/demos/debris_orbit/orbit_a/status
ros2 topic echo /ksp_vessel/demos/debris_orbit/orbit_a/estimator_status
ros2 topic echo /ksp_vessel/demos/debris_orbit/orbit_a/controller_status
```

保存先は実行時のディレクトリから`pylon_demo_debris_orbit_captures/<instance>/<起動日時>/`です。
0、36、…、324、360、396…度で保存し、PNGのJSONに画像時刻・要求角・推定角を記録します。
遅延した画像を現在の角度として保存せず、許容角を外れた場合は`capture_missed`を報告します。

## 検証

demosルートで、現行のインターフェースをsourceして実行します。
ROSテストは独立したdomain 173〜175を使用します。

```bash
PYTHONPATH="pylon_demo_debris_orbit:$PYTHONPATH" \
  /usr/bin/python3 -m unittest discover -s pylon_demo_debris_orbit/test -v
```

推定・指向・制御式、撮影時刻と2周目、現行APIの共有操作権・世代確認・停止保持を検証します。
KSPへの接続や実飛行はこの自動テストに含みません。

2026-10-06にLinux版KSP 1.12.5・ROS 2 Jazzy・現行PyLoNで実飛行も確認しました。
既存の分離済み軌道上セーブの検証用コピーを使用し、機体の保存時刻にシミュレーション時刻を合わせています。
`front_lidar` / `front_camera`を使い、半径15 m・角速度1 deg/sの既定設定で推定角508度まで進み、
36度間隔で15枚を保存しました。撮影角度の最大誤差は0.82度、推定距離は14.45〜15.71 mでした。
制御中のKSPセーブから独立に読み取った相対位置でも、一周を確認しました。
約9分後に約1秒のIMU等の受信空白が発生し、0.5秒の欠測ガードで停止・制御権返却しました。
長時間の無中断運転と、同梱craftの新規打上げ・分離直後の起動は、この検証では確認していません。

元デモと共通制御器の処理を、この版のソースへ引き継いでいます。今後それらを変更した場合は、
この版の対応モジュールと設定にも変更を反映してください。ライセンスはdemosの[MIT License](../LICENSE)です。
