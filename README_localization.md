# GLIM ローカリゼーション機能

GLIMに追加されたローカリゼーション機能により、事前に構築されたマップを使用した純粋な位置推定が可能です。

## 概要

GLIMローカリゼーション機能では以下が可能です：
- 事前のSLAMセッションで構築されたマップの読み込み
- マップを更新せずにリアルタイム位置推定を実行
- 読み込まれたマップ状態からのSLAM継続（マップ更新も継続）
- 起動パラメータまたはRViz2による初期姿勢設定

## アーキテクチャ

ローカリゼーション機能は既存のGLIMアーキテクチャの拡張として実装されています：

- **GlimROSLocalization**: GlimROSクラスをローカリゼーション専用機能で拡張
- **glim_localization_node**: ローカリゼーションモード用の単純な起動ノード
- メインGLIMブランチとの互換性を維持し、簡単に更新可能

## 前提条件

- ROS 2（Jazzyでテスト済み）
- GLIMとその依存関係がインストール済み
- 標準GLIM SLAMモードで作成された事前構築マップ

## ビルド

```bash
cd ~/colcon_ws
colcon build --packages-select glim_ros
source install/setup.bash
```

## 設定

### 設定ファイル

1. **config_localization.json**: メインのローカリゼーション設定
   ```json
   {
     "localization": {
       "map_path": "/tmp/dump",
       "initial_pose": {
         "x": 0.0, "y": 0.0, "z": 0.0,
         "roll": 0.0, "pitch": 0.0, "yaw": 0.0
       },
       "use_rviz_initial_pose": true,
       "enable_imu": true,
       "enable_sub_mapping": false,
       "enable_global_mapping": true
     },
     "odometry_estimation": {
       "so_name": "libodometry_estimation_gpu.so"
     }
   }
   ```

2. **config_ros.json**: ROSインターフェース設定（標準GLIMと共有）
   - IMUとPointCloud2トピックを設定
   - ローカリゼーション用に変更する必要なし

### トピック

入力:
- IMUデータ: `config_ros.json`で設定（デフォルト: `/imu/data`）
- PointCloud2: `config_ros.json`で設定（デフォルト: `/velodyne_points`）
- 初期姿勢: `/initialpose`（RViz2から）

出力:
- 現在姿勢: `/glim/pose` (PoseStamped)
- オドメトリ: `/glim/odom` (Odometry)
- TF: `map` -> `base_link`

## 使用方法

### 1. マップの作成

まず標準GLIM SLAMを使用してマップを作成します：

```bash
ros2 launch glim_ros glim.launch.py
# ロボット/センサーを動かしてマップを作成
# マップはデフォルトで /tmp/dump に保存されます
```

### 2. ローカリゼーションモードの実行

保存されたマップを使用してローカリゼーションノードを起動：

```bash
ros2 launch glim_ros glim_localization.launch.py \
  map_path:=/tmp/dump \
  initial_pose.x:=0.0 \
  initial_pose.y:=0.0 \
  initial_pose.yaw:=0.0
```

### 3. 初期姿勢の設定（オプション）

初期姿勢は以下の3つの方法で設定可能：

1. **起動パラメータ経由**: 上記の通り
2. **設定ファイル経由**: `config_localization.json`を編集
3. **RViz2経由**: "2D Pose Estimate"ツールを使用

### 起動パラメータ

- `map_path`: 保存されたマップディレクトリのパス（デフォルト: `/tmp/dump`）
- `initial_pose.x`, `initial_pose.y`, `initial_pose.z`: 初期位置
- `initial_pose.roll`, `initial_pose.pitch`, `initial_pose.yaw`: 初期方向

## 起動コマンド例

### 基本的なローカリゼーション
```bash
ros2 launch glim_ros glim_localization.launch.py
```

### カスタムマップと初期姿勢を指定
```bash
ros2 launch glim_ros glim_localization.launch.py \
  map_path:=/path/to/your/map \
  initial_pose.x:=10.0 \
  initial_pose.y:=5.0 \
  initial_pose.yaw:=1.57
```

## テスト方法

### 1. マップ読み込み確認

```bash
# ローカリゼーションノードを起動
ros2 launch glim_ros glim_localization.launch.py map_path:=/tmp/dump

# ログで以下のメッセージを確認
# "Map loaded successfully!" が表示されることを確認
```

### 2. 姿勢推定出力確認

```bash
# 姿勢トピックの確認
ros2 topic echo /glim/pose

# オドメトリトピックの確認
ros2 topic echo /glim/odom

# TF確認
ros2 run tf2_tools view_frames
```

### 3. RVizでの可視化

```bash
ros2 run rviz2 rviz2
```

RVizで以下を追加：
- Map（マップ表示）
- PoseStamped（`/glim/pose`トピック）
- TF（`map`フレーム）

## 実装詳細

### 動作原理

1. **マップ読み込み**: 保存されたファクターグラフとサブマップを起動時に読み込み
2. **ローカリゼーション**: 新しいセンサーデータを読み込まれたマップと照合
3. **SLAM継続**: 読み込まれた状態からSLAMを継続し、マップ更新を許可
4. **ファクターグラフ**: 新しい測定値を適切な制約で既存のファクターグラフに追加

### 主要特徴

- **SLAM機能の維持**: 純粋なローカリゼーションとは異なり、マップの更新も可能
- **シームレスな統合**: 既存のGLIMコンポーネントを破綻なく使用
- **簡単な更新**: メインGLIMブランチの更新とgit pullによる互換性

### 標準GLIMとの違い

- 起動時に事前構築マップを読み込み
- 名前空間トピック（`/glim/pose`、`/glim/odom`）へローカリゼーション結果を配信
- 複数の方法による初期姿勢設定をサポート
- ローカリゼーションに焦点を当てた簡素化された設定

## トラブルシューティング

### マップの読み込みが失敗する場合
- マップパスが存在し、有効なdumpファイルが含まれていることを確認
- 互換性のあるGLIMバージョンでマップが作成されたことを確認
- `graph.bin`と`values.bin`ファイルが存在することを確認

### ローカリゼーションドリフトが発生する場合
- 正しい初期姿勢が設定されていることを確認
- センサーデータの品質（IMUとLiDAR）を確認
- マップが現在の動作エリアをカバーしていることを確認

### パフォーマンスの問題
- GPU加速オドメトリ推定の使用を検討
- 前処理設定でダウンサンプリングパラメータを調整
- 必要に応じてサブマップのボクセル解像度を下げる

### ビルドエラー
- 依存関係を確認：
  ```bash
  rosdep install --from-paths src --ignore-src -r -y
  ```
- tf2_geometry_msgsが正しくインストールされていることを確認

## 今後の改善点

純粋なローカリゼーション向けの潜在的な拡張：
- ロスト時の再ローカリゼーション
- 複数仮説追跡
- 更新なしのマップマッチング（真の読み取り専用モード）
- 自動初期姿勢推定

## ライセンス

GLIMと同様 - MITライセンス

## サポート

問題や質問については、メインGLIMリポジトリを参照してください。