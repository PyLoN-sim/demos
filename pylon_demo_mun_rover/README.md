# Mun rover / RViz Nav2 demo

## 共通の実行入口

demosリポジトリのルートで実行します。依存パッケージの導入は[共通準備](../README.md)を参照してください。

```bash
./pylon_demo_mun_rover/build.sh
./pylon_demo_mun_rover/run.sh  # launch引数を後ろに追加できます
```

機体の準備から停止までの手順は、[デモガイド](https://github.com/PyLoN-sim/docs/blob/main/demos/mun-nav2.md)を参照してください。

ROS 2 Jazzy。前輪操舵または前後輪操舵の4輪・6輪ローバーを、3D LiDAR・IMU・車輪情報から推定して走らせます。RVizの **Nav2 Goal** は到着位置と向きを指定します。内部Nav2 actionは `/pylon/mun_rover/navigate_to_pose`、安全監視を通る公開actionは `/navigate_to_pose` です。直接内部actionへ指令しないでください。

## 起動

KSPの通常操作で下記の条件を満たす機体を月面に配置し、停止・接地を確認します。ビルドは`./pylon_demo_mun_rover/build.sh`です。

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch pylon_demo_mun_rover demo.launch.py
```

地面の白い領域、URDFの機体、車体外形が現れ、状態が `Ready: set Nav2 Goal` になってから矢印を置きます。経路を計画できる地面と旋回スペースが必要です。前進だけで向きを合わせるため、近いゴールでも大きく回り込む場合があります。最高速度は0.5 m/s、加速度は0.2 m/s²です。地図の灰色は未知で、走行させません。

3D LiDARが複数ある場合は`lidar_sensor_id:=lidar_3d_3bb3e35a`のように指定します。自動選択中に配信元が切り替わると走行を停止し、地図と推定を初期化します。機体ごとの旋回半径と車体外形は車輪・機体データから設定されます。前輪操舵では後軸、前後輪操舵では前後軸の中点が走行とゴール位置の基準です。

`start_bridge:=false` で既存bridgeへ接続できます。`evaluate:=true` で真値の配信と独立した評価ノードを起動します。既定ではbridgeの真値配信自体を無効にします。推定・制御ノードはどちらのモードでも真値を購読しません。

## URDFと月面写真の重ね表示

既定で`Rover URDF`と`Surface photos`を表示します。URDFはbridgeが生成したプリミティブ形状の機体モデルで、推定した位置・姿勢へ追従します。KSPの元メッシュやテクスチャではありません。視点も機体へ追従します。

カメラが1台なら自動選択し、走行に合わせて写真を撮り足します。画像の撮影時刻の推定姿勢、CameraInfo、カメラの相対取付位置を使い、LiDARで観測した地面の短い三角形へRGBを投影します。空、未観測の地面、車体の占有範囲、URDFの形状で遮られる部分、LiDARで確認できた手前の障害物を除外します。写真は10 cm格子で120 m四方に蓄積し、同じ場所は近距離で見下ろした観測を優先します。移動0.5 mまたは向き8度の変化を目安に更新します。

`Surface photos`のチェックを外すと白い通行可能地図が見えます。写真の明暗は撮影画像そのままで、通行可否を表しません。`3D LiDAR`、`Ground`、`Local costs`は必要なときに表示できます。カメラが複数ある場合は`camera_sensor_id:=camera_c584088f`のように指定します。`show_visualization:=false`でURDF・写真処理を停止できます。

ライブ映像も見たい場合は`show_camera:=true`を付けます。RVizのImageパネルが一部のHiDPI環境でクラッシュするため、映像は専用の`rqt_image_view`で開きます。写真地図はRViz内に表示されます。

```bash
ros2 launch pylon_demo_mun_rover demo.launch.py show_camera:=true
```

写真地図は表示専用で、自動走行の通行判定へは入力しません。自己位置推定が有効な間だけ更新し、再初期化・機体切替・テレポートで消去します。ディスクへの自動保存や写真測量による3D復元は行いません。投影位置はセンサーの精度に依存し、遠方の欠け、継ぎ目、動く影や小さな未検出遮蔽物は残り得ます。

## 機体の条件

- 左右対称の標準KSPホイール4輪または6輪。前輪だけ操舵、または前後輪とも操舵可能な配置に対応します。前後輪操舵では後輪を逆向きに切り、6輪の中央ペアは固定でも構いません。モーターとブレーキが使用可能で、十分な電源があること。
- 制御点は前方を向けること。Mk2 lander canでは **Control Point: Forward**。body frameはX前、Y左、Z上です。
- 3D LiDARは前方と地面が見える位置へ固定。幅の広いMun-Munでは低い取り付けだと地面の左右端が疎になり、未知領域による停止が起きます。初期試験は長距離・高密度プロファイル、約1.5 m高い位置を使用します。
- 起動時はMun上で車輪を接地させ、ブレーキを掛けたまま静止すること。IMUの重力方向と静止バイアスを30サンプル以上取得してから動作します。

`rober B`は4輪すべての操舵を有効にしたまま使用できます。保存済みのLiDAR配置・中距離プロファイルで直進と左右旋回を検証しています。Mk2 lander canの制御点をForwardに設定し、LiDARが1台なら起動引数は不要です。Nav2が有効になった後で、機体から求めた旋回半径を適用します。

## センサー処理と制約

車輪・IMUの予測を、3D点群のpoint-to-plane ICPと局所submapで補正します。平面上で観測できない移動方向はICPで補正せず、車輪から推定します。推定不確かさが0.6 mを超えると停止します。長距離SLAMや地図のループ閉じ込みは行いません。

地形更新は独立したワーカーで1件ずつ計算し、点群処理や車輪制御を待たせません。センサー欠測の0.5秒監視とは別に、地図に使った観測の経過時間を監視し、2秒を超えたら停止します。到着時はNav2の位置・向き許容値を読み取り、その内側でブレーキを保持して停止判定を待ちます。

地図は120 m四方、解像度0.25 m。観測された地面同士の辺長2 m以下の三角形だけを補間します。大きな欠測は未知のままです。レーザーの無反射を空き地と解釈しません。傾斜12度、段差0.25 m、凹凸0.15 mを初期の通行禁止基準とします。センサーの点間隔より小さい障害物の検出を保証するものではありません。

自車体で隠れる初期占有範囲だけは、静止した全車輪の接地を根拠に地面を初期化します。以後の道路はこの初期化で増やしません。車体寸法と0.5 mの余裕を含め、停止距離まで未知・障害物がないか毎回確認します。

地図・TFは真値ツリーと独立した `map → pylon_rover_odom → pylon_rover_base_footprint → pylon_rover_base_link` を使用します。センサーから機体への相対取り付け変換だけはbridgeのTFから読みます。

取消、指令やセンサーの0.5秒以上の欠測、接地喪失、制御権喪失、推定異常で停止し、古いゴールを自動再開しません。KSP側にも車輪指令0.25秒のwatchdogと駐車ブレーキがあります。異常後に再初期化するには、停止・取消後に:

```bash
ros2 service call /pylon/mun_rover/reset std_srvs/srv/Trigger '{}'
```

機体切替やテレポートでは独立heartbeatのruntime epochからlifecycle世代が変わり、地図と推定を破棄します。テレポート通知がUDPで1回落ちても次のパケットで検出します。

## 主なインターフェース

| 入出力 | Topic / action | 型 |
|---|---|---|
| 入力 | `/ksp_vessel/lidar_3d/<id>/points` | PointCloud2 |
| 入力 | `/ksp_vessel/imu/data_raw` | Imu |
| 入出力 | `/ksp_vessel/actuators/wheel/state`, `command` | WheelState / WheelCommand |
| 入力 | `/navigate_to_pose` | NavigateToPose action |
| 出力 | `/pylon/mun_rover/odom`, `odom_3d` | Odometry |
| 出力 | `/pylon/mun_rover/map` | OccupancyGrid |
| 出力 | `/pylon/mun_rover/points`, `ground`, `obstacles` | PointCloud2 |
| 出力 | `/pylon/mun_rover/plan`, `trajectory` | Path |
| 出力 | `/pylon/mun_rover/robot_description` | String (URDF) |
| 出力 | `/pylon/mun_rover/photo_map` | PointCloud2 (RGB) |
| 出力 | `/pylon/mun_rover/camera/image_raw`, `camera/camera_info` | Image / CameraInfo |
| 出力 | `/pylon/mun_rover/visualization_status` | String (JSON) |
| 出力 | `/pylon/mun_rover/status`, `geometry`, `evaluation` | String (JSON) |

WheelStateにはSIの半径・機体座標内の位置・車体境界・回転符号・操舵符号と上限を追加しています。WheelCommandの`brake`は0〜1で、モーターenabledとは独立です。interfaces・bridge・Modは同時に更新してください。旧Modではgeometryが欠けるため自律走行を開始しません。

## 検証

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
PYTHONPATH="../PyLoN/Ros2/pylon_bridge:../PyLoN/Ros2/pylon_perception:pylon_demo_mun_rover:$PYTHONPATH" \
  python3 -m unittest discover -s pylon_demo_mun_rover/test -v
./pylon_demo_mun_rover/build.sh
```
