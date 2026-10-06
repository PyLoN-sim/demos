# PyLoN Demos

[PyLoN](https://github.com/PyLoN-sim/PyLoN)のセンサー・制御APIを使うROS 2 / Space ROSデモ集です。KSPはホストで動作します。
[Space ROS demos](https://github.com/space-ros/demos)を参考に、デモごとにREADME、`build.sh`、`run.sh`を配置しています。

| デモ | 内容 | bridge |
|---|---|---|
| [debris_orbit](pylon_demo_debris_orbit/) | LiDAR・IMUによるデブリ周回と撮影（Python直接起動） | 別途起動 |
| [lidar_slam](pylon_demo_lidar_slam/) | 2D SLAMとNav2 | 別途起動 |
| [mun_rover](pylon_demo_mun_rover/) | 月面ローバーのNav2走行 | launchが起動 |
| [position_estimator](pylon_demo_position_estimator/) | 3D LiDARによる6DoF推定 | 別途起動 |
| [reusable](pylon_demo_reusable/) | 衛星分離と逆噴射着陸、Space ROS lifecycle | launchが起動 |

## 共通準備（ROS 2 Jazzy）

Ubuntu 24.04、ROS 2 Jazzy、rosdep、colcon、rsyncを用意し、[導入ガイド](https://github.com/PyLoN-sim/docs/blob/main/guide/getting-started.md)に従ってKSPにMODを導入します。

```bash
mkdir -p ~/src
cd ~/src
git clone https://github.com/PyLoN-sim/PyLoN.git
git clone https://github.com/PyLoN-sim/demos.git
cd demos
source /opt/ros/jazzy/setup.bash
# 全デモの依存を導入。特定のデモだけなら . をそのディレクトリに変更します。
rosdep install --from-paths ../PyLoN/Ros2 . --ignore-src --rosdistro jazzy -y
./pylon_demo_lidar_slam/build.sh
./pylon_demo_lidar_slam/run.sh scan_topic:=/ksp_vessel/lidar_2d/front_lidar/scan
```

本体が別の場所にある場合は`PYLON_DIR=/path/to/PyLoN`、ROS環境やワークスペースは`ROS_SETUP`・`ROS2_WS`で指定できます。`build.sh`は本体のROSパッケージと選択したデモを同期し、amentパッケージをビルドします。KSPのMODは本体の`sync.sh`で導入します。

`pylon_demo_debris_orbit`はデモ自身のビルドが不要です。依存する本体のROSパッケージは`./pylon_demo_debris_orbit/build.sh`で準備できます。本体のbridge・メッセージ型を用意したROS環境で、`python3 pylon_demo_debris_orbit/run.py`から直接起動します。[専用README](pylon_demo_debris_orbit/README.md)に認識・目標姿勢・目標推力のモジュール構成と起動手順を記載しています。

bridgeを別途起動するデモでは、別ターミナルでROS環境をsourceし、`ros2 run pylon_bridge udp_bridge --host 127.0.0.1`を実行します。デブリ周回では`--disable-ground-truth`を追加します。同じKSPへ接続するbridgeは1つだけにしてください。機体準備・制御の開始条件・停止手順は各READMEと[デモガイド](https://github.com/PyLoN-sim/docs/blob/main/demos/index.md)を参照してください。

## Space ROS（reusable）

LinuxのDockerと、本体のcloneが必要です。まず本体イメージをビルドし、その上にデモだけを追加します。

```bash
./pylon_demo_reusable/spaceros.sh build
./pylon_demo_reusable/spaceros.sh test
./pylon_demo_reusable/spaceros.sh demo
# 別ターミナルで準備状態を確認し、明示的に打ち上げを開始
./pylon_demo_reusable/spaceros.sh exec ros2 lifecycle get /reusable_mission
./pylon_demo_reusable/spaceros.sh exec ros2 lifecycle set /reusable_mission activate
```

`pylon_demo_reusable/Dockerfile`がコンテナ定義です。他のament版3デモは上記のホストROS 2向けスクリプトを使用します。

## 貢献

各デモには目的、必要な機体・センサー、依存、起動・停止手順、検証範囲を記載し、`build.sh`と`run.sh`を用意してください。再利用するbridge・メッセージ・共通制御は[PyLoN本体](https://github.com/PyLoN-sim/PyLoN)、利用者向けガイドは[docs](https://github.com/PyLoN-sim/docs)で管理します。

旧`KSP_ROS2/Demo`から関連するGit履歴を引き継いでいます。[MIT License](LICENSE)。
