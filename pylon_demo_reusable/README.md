# pylon_demo_reusable

Space ROS / ROS 2 JazzyのLifecycleNodeで、無人機PyLoN Phoenixの打ち上げ、衛星分離、逆噴射着陸を管理するデモです。

使い方・機体・検証範囲は[デモガイド](../../docs/demos/reusable-launch.md)を参照してください。

`mission.py`はROSに依存しない状態機械と誘導則、`node.py`はlifecycle、lease、通信監視を扱います。Ground Truthを使用するシミュレータ用デモで、実機用航法ソフトウェアではありません。

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
PYTHONPATH="Demo/pylon_demo_reusable:$PYTHONPATH" python3 -m pytest Demo/pylon_demo_reusable/test
```
