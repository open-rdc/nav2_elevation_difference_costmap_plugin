## nav2_elevation_difference_costmap_plugin

Nav2 costmap用の高低差格子地図pluginです.

参考:
井上裕文（千葉工大），上田隆一（千葉工大），林原靖男（千葉工大），
"高低差格子地図を用いた移動ロボットの自己位置推定"，3I1-4，SI2016 (2016)

## Build

```bash
cd ~/ros2_ws/src
git clone https://github.com/open-rdc/nav2_elevation_difference_costmap_plugin.git
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select nav2_elevation_difference_costmap_plugin --symlink-install
source install/setup.bash
```

## Nav2 Params

`nav2_params.yaml`のcostmap pluginsに`elevation_layer`を追加します.

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["elevation_layer"]
      elevation_layer:
        plugin: "nav2_elevation_difference_costmap_plugin::ElevationLayer"
        enabled: True
```

点群topicは現在 `/surestar_points` を購読します.

## RVizでの検証

Nav2を起動し, RViz2で以下を表示します.

- `/surestar_points`
- `/local_costmap/costmap` など, pluginを入れたcostmap topic
- `TF`

確認手順:

1. Nav2起動直後にcostmapが表示されることを確認する.
2. RViz2の`2D Pose Estimate`または`nav2_pose`で自己位置を修正する.
3. `/surestar_points`が出続けていることを確認する.
4. 自己位置修正後もcostmapが空にならないことを確認する.

`map -> odom`の変化も見る場合:

```bash
ros2 run tf2_ros tf2_echo map odom
```

自己位置修正で`map -> odom`が変わってもcostmapが残っていれば, 今回の座標変換問題は改善されています.

## 単体確認用RViz

実機やGazeboを使わず, 標準`nav2_costmap_2d`ノードでpluginを読み込んでRViz確認する場合:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=/tmp python3 src/nav2_elevation_difference_costmap_plugin/tools/run_elevation_layer_pose_check.py --rviz
```

RViz2上で`/costmap/costmap`, `/surestar_points`, `TF`を確認できます.

この確認用ノードは, 5cm程度の段差, 少し傾いた地面, ランダムに散った点群を`/surestar_points`へ出します.
RViz2で黒く見える地面や段差付近の色は, pluginが点群から計算したcostmapです.
