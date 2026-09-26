# PyLoN Debris Orbiter

[`PyLoN Debris Orbiter.craft`](PyLoN%20Debris%20Orbiter.craft)は、このPCでデブリ周回試験に使用した`test A`を配布用に保存した機体です。KSP 1.12.5、stockパーツとPyLoNを使用します。

元データは2026-09-07のLiDAR＋IMU周回試験用セーブ`KerbalLiDAR-Debug-X0ABv4`の`Ships/VAB/Auto-Saved Ship.craft`です。PyLoNの移行ツールで旧パーツ名・PartModule名と接続参照を現行名へ変換し、機体名と説明を変更しました。パーツ配置・資源量・ステージ設定・Sensor IDは元の保存内容を維持しています。旧セーブの移行作業は不要です。

21パーツで、Mk1ランダー缶、RCSブロック8個、モノプロペラントタンク、リアクションホイール、太陽電池2枚、3D LiDAR（`front_lidar`）、RGBカメラ（`orbit_camera`）を搭載しています。デカプラーの先にあるRockomax X200-32タンクとMainsailを分離し、無推力の撮影対象にします。LiDARはLong・250 m・10 Hz、カメラは5 Hz・垂直画角60度で保存されています。

## 読み込み

KSPを終了し、demosリポジトリのルートで実行します。`SAVE`は既存のsandboxセーブのフォルダー名に置き換えてください。

```bash
KSPDIR="$HOME/.local/share/Steam/steamapps/common/Kerbal Space Program"
SAVE='ROS2 debug'
cp -n 'pylon_demo_debris_orbit/craft/PyLoN Debris Orbiter.craft' \
  "$KSPDIR/saves/$SAVE/Ships/VAB/"
```

VABの機体読み込みから **PyLoN Debris Orbiter** を選び、クルーを1名乗せます。`.craft`は機体設計だけを含み、軌道上の位置・速度や分離状態は含みません。この機体は軌道上の周回試験用で、地上からの打ち上げ用ロケットとしては検証していません。KSP標準のデバッグメニューでKerbinの高度約100 kmの円軌道へ配置するか、別途打ち上げ手段を用意してください。

軌道上でエンジンを停止し、デカプラーの右クリックメニューから対象を分離します。保存済みのステージ設定は変更していないため、分離にSpaceキーを使わずデカプラーを直接操作してください。センサーを積んだ機体を操作対象にし、太陽電池とRCSを有効にして相対運動を小さくします。続きは[デモガイド](https://github.com/PyLoN-sim/docs/blob/main/demos/debris-orbit.md)を参照してください。

ROS 2パッケージをビルドした場合は、`$(ros2 pkg prefix pylon_demo_debris_orbit)/share/pylon_demo_debris_orbit/craft/`にも配置されます。

元機体での周回・撮影記録はありますが、現行PyLoN名へ変換した同梱ファイルのKSP再読み込み・再飛行は未確認です。
