# pylon_demo_reusable

## 共通の実行入口

demosリポジトリのルートで実行します。依存パッケージの導入は[共通準備](../README.md)を参照してください。

```bash
./pylon_demo_reusable/build.sh
./pylon_demo_reusable/run.sh  # launch引数を後ろに追加できます
```

Space ROS / ROS 2 JazzyのLifecycleNodeで、無人機PyLoN Phoenixの打ち上げ、衛星分離、逆噴射着陸を管理するデモです。

使い方・機体・検証範囲は[デモガイド](https://github.com/PyLoN-sim/docs/blob/main/demos/reusable-launch.md)を参照してください。

`mission.py`はROSに依存しない状態機械と誘導則、`node.py`はlifecycle、lease、通信監視を扱います。Ground Truthを使用するシミュレータ用デモで、実機用航法ソフトウェアではありません。

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
PYTHONPATH="pylon_demo_reusable:$PYTHONPATH" python3 -m pytest pylon_demo_reusable/test
```
