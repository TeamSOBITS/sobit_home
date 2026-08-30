<a name="readme-top"></a>

[JA](README_ja.md) | [EN](README.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# SOBIT HOME


<!-- 概要 -->
## 概要

![SOBIT HOME](sobit_home/docs/img/sobit_home.png)

4輪独立ステアリング移動機構・昇降機構・2椀・パンチルト機構を組み合わせたSOBITS自作モバイルマニピュレータを動かすためのパッケージです．

> [!CAUTION]
> 初心者の場合，実機のロボットを扱う際に，先輩方に付き添ってもらいながらロボットを動かしましょう．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- セットアップ -->
## セットアップ

ここで，本レポジトリのセットアップ方法について説明します．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### 環境条件

まず，以下の環境を整えてから，次のインストール段階に進んでください．

| System  | Version |
| --- | --- |
| Ubuntu | 24.04 (Noble Numbat) |
| ROS    | Jazzy Jalisco |
| Python | 3.12 |
| Docker | latest |

> [!NOTE]
> `Ubuntu`や`ROS`のインストール方法に関しては，[SOBITS Manual](https://github.com/TeamSOBITS/sobits_manual#%E9%96%8B%E7%99%BA%E7%92%B0%E5%A2%83%E3%81%AB%E3%81%A4%E3%81%84%E3%81%A6)に参照してください．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### インストール方法

1. ROSの`src`フォルダに移動します．
    ```sh
    $ cd ~/colcon_ws/src/
    ```

2. 本レポジトリをcloneします．
    ```sh
    $ git clone https://github.com/TeamSOBITS/sobit_home
    ```

3. レポジトリの中へ移動します．
    ```sh
    $ cd sobit_home/
    ```

4. 依存パッケージをインストールします．
    ```sh
    $ bash install.sh
    ```

5. コンパイルする前に，`rm_motors_ros`のため，RUSTをセットアップしてください．
    ```sh
    source $HOME/.bashrc

    cd ~/colcon_ws/src/rm_motors_ros/rm_motors_hw/rm_motors_can
    cargo install cargo-expand
    cargo build --release
    ```

6. パッケージをコンパイルします．
    ```sh
    $ cd ~/colcon_ws/
    $ colcon build --symlink-install
    $ source ~/colcon_ws/install/setup.sh
    ```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- 実行・操作方法 -->
## 実行・操作方法

1. まず，[real_minimal.launch.py](sobit_home_bringup/launch/real_minimal.launch.py)を実行します．
  ```sh
  $ ros2 launch sobit_home_bringup real_minimal.launch.py
  ```

2. 起動機能はlaunch引数で有効/無効を切り替えることを推奨します．
  ```sh
  $ ros2 launch sobit_home_bringup real_minimal.launch.py \
    enable_mobile_base:=true \
    enable_body:=true \
    enable_arm_left:=true \
    enable_arm_right:=false \
    enable_head:=true \
    enable_lidar:=true \
    use_rviz:=true
  ```

3. 実機では，起動前に`.bashrc`を読み込み，SOBIT HOME用のドメインに切り替えてください．
  ```sh
  $ source ~/.bashrc
  $ sobit_home_mode
  ```

実機接続に失敗する場合は，以下を確認してください．

- 非常停止ボタンが押されていないか．
- バッテリー残量が十分か．
- USBハブがPCに接続されているか．
- 必要な環境変数が設定されているか（`DXL_X_LOWER_PORT`，`DXL_X_UPPER_PORT`，`DXL_P_UPPER_PORT`，`UM_PORT`，`HOME_CAM_LEFT_PORT`，`HOME_CAM_RIGHT_PORT`）．
- `enable_mobile_base:=true`時に`can0`が利用可能か．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- 制御メモ -->
## 制御メモ

### ホイールコントローラ

`move_wheel_linear`は指定距離（メートル）だけ直進，`move_wheel_rotate`は指定角度（ラジアン）だけその場旋回します．どちらもオドメトリを使ったクローズドループ制御で，ゴール付近では滑らかに停止します．ゲインはROSパラメータで実行時に変更できます（再コンパイル不要）．

### スワーブドライブ

4輪独立ステアリングコントローラが`cmd_vel`のx・y・θ成分から各輪のステア角とドライブ速度を計算します．各輪は常に最短経路でステアリングし，180°反転＋ドライブ逆転の方が速い場合は自動で切り替えます．

### MoveIt連携

`plan_to_pose`は指定した計画グループ（`arm_left`，`arm_right`，`arm_left_body`，`arm_right_body`）に対して軌道を生成してキャッシュします．`execute_plan`でキャッシュした軌道を再生します．全身グループ（`arm_left_body`，`arm_right_body`）では，アームと同時にベースもオドメトリフィードバックで追従します．

`plan_to_named_pose`も同様ですが，目標姿勢の代わりにSRDFで定義された名前付き姿勢（`initial_pose`，`move_pose`など）を指定します．生成した軌道は同じくキャッシュされ，`execute_plan`で実行します．

サーバが初期化する計画グループは`active_planning_groups`パラメータで決まります．`arm`，`head_arm_body`，`mobile_base_*`などSRDFで定義された他のグループを使う場合は，起動時にこのパラメータを指定してください．

実機・Gazeboシミュレーションの両方で動作します．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### 実行時変更可能なパラメータ

以下のパラメータはノードの動作中に再読み込みされるため，再起動・再ビルドなしで`ros2 param set`により調整できます．

`wheel_action_server` — 移動制御：

| パラメータ | 既定値 | 意味 |
| --- | --- | --- |
| `wheel_linear_kp` / `_ki` / `_kd` | 起動時に指定 | `move_wheel_linear`のPIDゲイン |
| `wheel_rotate_kp` / `_ki` / `_kd` | 起動時に指定 | `move_wheel_rotate`のPIDゲイン |
| `wheel_linear_arrival_tol` | `0.02` | 到達判定の許容誤差 [m] |
| `wheel_rotate_arrival_tol` | `0.02` | 到達判定の許容誤差 [rad] |
| `wheel_max_linear_vel` | `0.2` | 速度上限 [m/s] |
| `wheel_max_lateral_vel` | `0.2` | 横方向の速度上限 [m/s] |

`moveit_server` — 計画時間とワークスペース（既定値は[moveit_server.yaml](sobit_home_moveit_config/config/moveit_server.yaml)）：

| パラメータ | 既定値 | 意味 |
| --- | --- | --- |
| `plan_time_sec` | `10.0` | 1回の計画試行あたりの時間上限 [s] |
| `plan_attempts` | `10` | OMPLの試行回数 |
| `workspace_min_x/y/z` | `-5.0`，`-5.0`，`0.0` | 計画ワークスペースの最小座標 [m] |
| `workspace_max_x/y/z` | `5.0`，`5.0`，`5.0` | 計画ワークスペースの最大座標 [m] |

```sh
# 到達判定の許容誤差と計画時間を動作中に変更する
$ ros2 param set /sobit_home/wheel_action_server wheel_linear_arrival_tol 0.01
$ ros2 param set /sobit_home/moveit_server plan_time_sec 5.0
```

起動時に`moveit_server_config:=<path>`を指定すると，パッケージ既定のYAMLの代わりに別の計画パラメータYAMLを読み込めます．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### プランニングシーンWarehouse

MoveItはプランニングシーン・ロボット状態・拘束条件をデータベースに保存できます．RVizの**Stored Scenes**・**Stored States**パネルはこのデータベースを読み書きします．`move_group`が直接接続するため，ロボットを起動した時点で利用可能です．

| バックエンド | `warehouse_backend` | 保存先 | 備考 |
| --- | --- | --- | --- |
| SQLite | `sqlite`（既定） | 単一ファイル（`warehouse_database_path`） | `install.sh`でインストール．サーバ不要 |
| MongoDB | `mongo` | `mongod`が管理するディレクトリ | ソースからビルド（下記参照） |

```sh
# 既定：~/.ros/sobit_home_warehouse.sqlite
$ ros2 launch sobit_home_bringup gz_minimal.launch.py

# ファイルを明示的に指定する
$ ros2 launch sobit_home_bringup gz_minimal.launch.py \
  warehouse_database_path:=$HOME/.ros/my_scenes.sqlite
```

空のデータベースに既定の内容を投入するには，warehouseのlaunchを1度実行します．

```sh
$ ros2 launch sobit_home_moveit_config warehouse_db.launch.py
```

MongoDBはROS 2 Jazzy向けのバイナリ配布もrosdepルールも存在しないため，任意選択でソースからビルドします．

```sh
$ WAREHOUSE_MONGO=1 ./install.sh
$ colcon build --packages-select warehouse_ros_mongo
# データベースサーバを起動してから，同じバックエンドでロボットを起動する
$ ros2 launch sobit_home_moveit_config warehouse_db.launch.py warehouse_backend:=mongo
$ ros2 launch sobit_home_bringup gz_minimal.launch.py warehouse_backend:=mongo
```

> [!WARNING]
> SQLiteバックエンドでは，既に存在する名前でシーンを保存すると`warehouse_ros_sqlite::InternalError`（`no such column: M_planning_scene_id`）が送出され，呼び出し元のプロセスが異常終了します．これは`warehouse_ros_sqlite` 1.0.5の上流の不具合です．メタデータの列は遅延生成されるため，`planning_scene_id`を一度も受け取っていないテーブルには上書き処理が参照する列が存在しません．**新規の名前**での保存と読み込みは正常に動作します．`move_group`自体には影響ありません．

> [!NOTE]
> `warehouse_db.launch.py`とロボットのlaunchでは`warehouse_backend`を一致させてください．異なる場合はそれぞれ別のデータベースを開くことになります．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### 意味論的記述（SRDF）

SRDFは構成ごとに静的ファイルを用意するのではなく，単一の[sobit_home.srdf.xacro](sobit_home_moveit_config/config/sobit_home.srdf.xacro)から生成されます．launchファイルと同じ`enable_*`モジュールフラグを受け取るため，一部の部位を無効にして起動した場合，存在しないリンクを参照する計画グループ・姿勢・エンドエフェクタ・干渉ペアは生成されません．

`enable_teleop:=true`とすると，`mobile_base`の計画グループと平面仮想関節が除外されます．遠隔操作ではオペレータがベースを直接操作するため，ベースを計画グループに含めてはならないためです．

```sh
$ ros2 launch sobit_home_bringup gz_minimal.launch.py enable_teleop:=true
```

> [!IMPORTANT]
> URDFとSRDFは同じフラグで展開されるため，常に同一のロボットを表します．どちらか一方にしかモジュールフラグを渡さないと，MoveItは実際に生成されていないロボットに対して計画することになります．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### Rviz2上の可視化

実機を動かす前段階として，Rviz2上でSOBIT HOMEを可視化し，ロボットの構成を表示することができます．

```sh
$ ros2 launch sobit_home_description display.launch.py
```

<!-- 正常に動作した場合は，次のようなRviz画面が表示されます．
![SOBIT HOME Display with Rviz](sobit_home/docs/img/sobit_home_rviz.png) -->

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

### シミュレータの実行方法

SOBIT HOMEにはGazebo Harmonicのシミュレーション環境が用意されておりますので，実機がなくても，動作確認が可能です．

```sh
$ ros2 launch sobit_home_bringup gz_minimal.launch.py
```

現時点では，これらの仮想環境が用意されています．

| World Name | 説明 |
| --- | --- |
| `empty`      | 家具などのない環境を出現． |
| `wrs`        | WRS2020に実施されたTidy Up環境を出現． |
| `small_house` | AWSが開発した小型部屋のレイアウトを出現．|
| `rcjo2025_arena` | RCJ Open 2025向けのアリーナ環境を出現．|
| `rcjo2026_arena` | RCJ Open 2026向けのアリーナ環境を出現（デフォルト）．|

環境を変更するために，`world_model`を[gz_minimal.launch.py](sobit_home_bringup/launch/gz_minimal.launch.py)で変更してください．

```sh
$ ros2 launch sobit_home_bringup gz_minimal.launch.py world_model:=empty
```


<!-- 正常に動作した場合は，次のようなGazeboの画面が表示されます．
![SOBIT HOME Gazebo Harmonic](sobit_home/docs/img/sobit_home_gz_sim.png) -->

> [!TIP]
> 実機と同じようなセンサも搭載されていますので，パソコンによって処理が重くなる可能性がありますので，必要なセンサだけを[gz_minimal.launch.py](sobit_home_bringup/launch/gz_minimal.launch.py)で選択してください．

```python
'enable_head_cam_color'       : 'true',
'enable_head_cam_depth'       : 'true',
'enable_hand_left_cam_color'  : 'true',
'enable_hand_right_cam_color' : 'true',
'enable_lidar'                : 'true',
```

また，複数のSOBIT HOMEを同じシミュレーション環境でも出現できます．
`robot_id`と出現座標を変えて起動してください．

```sh
# Robot 1
$ ros2 launch sobit_home_bringup gz_minimal.launch.py \
  robot_name:=sobit_home robot_id:=1 robot_coords_x:=0.0 robot_coords_y:=0.0 robot_coords_Y:=0.0

# Robot 2
$ ros2 launch sobit_home_bringup gz_minimal.launch.py \
  robot_name:=sobit_home robot_id:=2 robot_coords_x:=0.0 robot_coords_y:=2.0 robot_coords_Y:=0.0
```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


## ソフトウェア

### パッケージ概要

| パッケージ | 役割 | 主なエントリポイント |
| --- | --- | --- |
| `sobit_home_bringup` | 実機/シミュレータ起動を統合したbringup | `launch/real_minimal.launch.py`, `launch/gz_minimal.launch.py`, `launch/robot.launch.py` |
| `sobit_home_control` | スワーブ移動制御とMoveIt全身追従ブリッジ | `swerve_controller_node`, `moveit_whole_body_bridge_node` |
| `sobit_home_library` | 関節/移動/MoveItの高レベルAction・Service群 | `launch/action_server.launch.py`, `joint_action_server`, `wheel_action_server`, `moveit_action_server` |
| `sobit_home_description` | URDF/Xacroモデル，RViz設定，基本ワールド | `launch/display.launch.py`, `robots/sobit_home_robot.urdf.xacro` |
| `sobit_home_moveit_config` | MoveItの計画設定と起動 | `launch/move_group.launch.py` |
| `sobit_home_kinematics_plugin` | SOBIT HOME向けMoveIt運動学プラグイン | `sobit_home_kinematics_plugin_description.xml` |

<details>
<summary>SOBIT HOMEと関わるソフトの情報まとめ</summary>


### ジョイントコントローラ

SOBIT HOMEのパンチルト機構と昇降機構とマニピュレータを動かすための情報まとめです．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


#### 動作方法

`sobit_home_library`で現在実装されているインターフェースは以下です．

1. Action
   - `move_joint`
   - `move_to_pose`
   - `move_right_hand_to_pose`
   - `move_left_hand_to_pose`

2. Service
   - `get_hand_to_coord/left` — 左アームの解析的IK．任意のTFフレームで指定した目標姿勢を受け取り，関節角度・成功フラグ・到達ヒントを返します．
   - `get_hand_to_coord/right` — 右アームの解析的IK．左と同じインターフェースです．
   - `get_hand_to_tf/left`
   - `get_hand_to_tf/right`
   - `get_head_to_coord`
   - `get_head_to_tf`
   - `get_finger_angle`

   **到達ヒント（`move_pose`）：** 目標がアームのワークスペース外にある場合，サービスは`success=false`を返しますが，`move_pose`に目標を到達可能にするための最小調整量を格納して返します．

   | フィールド | 意味 |
   | --- | --- |
   | `position.x` | ベースの前後移動量（m）；正=前進，負=後退 |
   | `position.y` | 将来の横移動用に予約済み（常に0.0） |
   | `position.z` | ボディリフトの調整量（m）；正=上昇，負=下降 |
   | `orientation` | 目標方向へのヨー角 |

3. MoveIt連携（`action_server.launch.py`で起動）
   - Service: `plan_to_pose` — 計画グループに対して目標姿勢への軌道を生成
   - Service: `plan_to_named_pose` — SRDFで定義された名前付き姿勢（`initial_pose`，`move_pose`など）への軌道を生成
   - Action: `execute_plan` — いずれかのServiceでキャッシュした軌道を実行

4. 配信トピック
   - `hand_left/grasp_state`，`hand_right/grasp_state`（`std_msgs/Bool`）— 把持判定．ハンド動作の完了ごとに1回配信されます．2本以上の指が指令角度に到達せず停止した場合（物体に阻まれている場合）は`true`，指が目標角度まで到達した場合（何も把持していない場合）は`false`となります．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


#### ジョイント名

SOBIT HOMEのジョイント名とその定数名を以下の通りです．

| ジョイント番号 | ジョイント名 | ジョイント定数名 |
| :---: | --- | --- |
|  0 | head_pan_joint                | - |
|  1 | head_tilt_joint               | - |
|  2 | arm_left_shoulder_tilt_joint  | - |
|  3 | arm_left_upper_roll_joint     | - |
|  4 | arm_left_upper_flex_joint     | - |
|  5 | arm_left_elbow_joint          | - |
|  6 | arm_left_lower_flex_joint     | - |
|  7 | arm_left_wrist_tilt_joint     | - |
|  8 | arm_left_wrist_roll_joint     | - |
|  9 | arm_right_shoulder_tilt_joint | - |
| 10 | arm_right_upper_roll_joint    | - |
| 11 | arm_right_upper_flex_joint    | - |
| 12 | arm_right_elbow_joint         | - |
| 13 | arm_right_lower_flex_joint    | - |
| 14 | arm_right_wrist_tilt_joint    | - |
| 15 | arm_right_wrist_roll_joint    | - |
| 16 | hand_left_finger_l_mcp_joint  | - |
| 17 | hand_left_finger_l_pip_joint  | - |
| 18 | hand_left_finger_l_dip_joint  | - |
| 19 | hand_left_finger_c_mcp_joint  | - |
| 20 | hand_left_finger_c_ip_joint   | - |
| 21 | hand_left_finger_r_pip_joint  | - |
| 22 | hand_left_finger_r_dip_joint  | - |
| 23 | hand_right_finger_l_mcp_joint | - |
| 24 | hand_right_finger_l_pip_joint | - |
| 25 | hand_right_finger_l_dip_joint | - |
| 26 | hand_right_finger_c_mcp_joint | - |
| 27 | hand_right_finger_c_ip_joint  | - |
| 28 | hand_right_finger_r_pip_joint | - |
| 29 | hand_right_finger_r_dip_joint | - |
| 30 | body_lift_joint               | - |
| 31 | wheel_steer_f_l_joint         | - |
| 32 | wheel_steer_f_r_joint         | - |
| 33 | wheel_steer_b_l_joint         | - |
| 34 | wheel_steer_b_r_joint         | - |
| 35 | wheel_drive_f_l_joint         | - |
| 36 | wheel_drive_f_r_joint         | - |
| 37 | wheel_drive_b_l_joint         | - |
| 38 | wheel_drive_b_r_joint         | - |

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


#### ポーズの設定方法

[pose_list.yaml](sobit_home_library/config/pose_list.yaml)というファイルでポーズの追加・編集ができます．以下のようなフォーマットになります．

```yaml
/**:
  ros__parameters:
    poses:
      - initial_pose
      - detecting_pose

    initial_pose:
      body_lift               : 0.5
      head_pan                : 0.0
      head_tilt               : 0.0
      arm_left_shoulder_tilt  : 0.0
      arm_left_upper_roll     : 0.0
      arm_left_upper_flex     : 0.0
      arm_left_elbow          : 0.0
      arm_left_lower_flex     : 0.0
      arm_left_wrist_tilt     : 0.0
      arm_left_wrist_roll     : 0.0
      arm_right_shoulder_tilt : 0.0
      arm_right_upper_roll    : 0.0
      arm_right_upper_flex    : 0.0
      arm_right_elbow         : 0.0
      arm_right_lower_flex    : 0.0
      arm_right_wrist_tilt    : 0.0
      arm_right_wrist_roll    : 0.0
...
```  

定義したいポース名を`poses`に追加し，その後ポース名の下に各ジョイントの角度を設定します．

> [!NOTE]
> ポーズ指定の動作（`move_to_pose`）は，変更したジョイントだけでなく，**すべて**のアーム・ボディ・ヘッドのジョイントを指令します．ポーズ定義で省略したジョイントは `0.0` が既定値となり，そこへ実際に駆動されます．そのため各ポーズには必ず全ジョイントを記述してください．（1つのジョイントだけを指令し，残りを現在位置に保持したい場合は `move_joint` Actionを使用してください．）

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


#### 起動時にポーズリストを上書きする

`action_server.launch.py`はポーズYAMLのパスを起動引数として公開しているため，**`sobit_home_library`を編集することなく**，別のパッケージやマシンから独自のポーズを与えることができます．既定値はライブラリ自身の`config/`を指します．

| 起動引数 | 既定値 |
| --- | --- |
| `pose_config` | `sobit_home_library/config/pose_list.yaml` |
| `right_hand_pose_config` | `sobit_home_library/config/right_hand_pose_list.yaml` |
| `left_hand_pose_config` | `sobit_home_library/config/left_hand_pose_list.yaml` |

```sh
$ ros2 launch sobit_home_library action_server.launch.py \
    pose_config:=/path/to/my_pose_list.yaml
```

`robot.launch.py`も同じ引数を引き渡すため，フルbringupからでも同様に上書きできます．さらにロボットのbringupでは，`enable_action_server:=false`を指定することで（別のマシンで起動できるように）Actionサーバーの起動自体をスキップできます．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


#### 実行中にポーズを更新する（再起動不要）

ポーズはROSパラメータとして保持されているため，ノードを再起動せずに実行中に設定・再読み込みできます．パラメータを変更した後，`reload_poses`サービスを呼び出すと，メモリ上のポーズ一覧が再構築されます．

```sh
# 単一の値を設定するか，YAML全体を一括で読み込む
$ ros2 param set /sobit_home/joint_action_server initial_pose.body_lift 0.42
$ ros2 param load /sobit_home/joint_action_server /path/to/new_pose_list.yaml

# 変更を適用する
$ ros2 service call /sobit_home/reload_poses std_srvs/srv/Trigger {}
```

サービスの応答には現在読み込まれている全身ポーズの名前が一覧表示されるため，編集が反映されたかを確認できます．

> [!IMPORTANT]
> `reload_poses`は**`poses`配列の内容のみ**から有効なポーズ一覧を再構築します．既存ポーズの値の更新は即座に反映されますが，**新しいポーズを追加する**場合は，その名前を`poses`配列にも追加する必要があります（例：`ros2 param set /sobit_home/joint_action_server poses "[initial_pose, ..., my_new_pose]"`）．そうしないとその値は無視され，`move_to_pose`は「pose not found」で中断します．配列から削除された名前は，次回の再読み込み時に有効な一覧から取り除かれます．
>
> 実行中の変更はYAMLファイルには**書き戻されません**．値を調整したら，再起動後も保持されるように`pose_list.yaml`へ反映してください．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### ホイールコントローラ

SOBIT HOMEの移動機構を動かすための情報まとめです．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


#### 動作方法

`sobit_home_library`で現在実装されている移動系Actionは以下です．

1. `move_wheel_linear`
2. `move_wheel_rotate`

ホイールActionサーバは`cmd_vel`を出力し，`odom`をフィードバックとして利用します．

</details>

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


## ハードウェア
SOBIT HOMEはオープンソースハードウェアとして[OnShape](https://cad.onshape.com/documents/e17931db96792e39eba48d39/w/a81eeb68b7f4ed981ce8878a/e/42d5107e3af255ccdf5ca7e7?renderMode=0&uiState=69ee43ae00a7b5401b55d390)にて公開しております．

![SOBIT HOME in OnShape](sobit_home/docs/img/sobit_home_onshape.png)

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<details>
<summary>ハードウェアの詳細についてはこちらを確認してください．</summary>

### パーツのダウンロード方法

1. Onshapeにアクセスしましょう．

> [!NOTE]
> ファイルをダウンロードするために，`OnShape`のアカウントを作成する必要はありません．ただし，本ドキュメント全体をコピーする場合，アカウントの作成を推薦します．

2. `Instances`の中にパーツを右クリックで選択します．
3. 一覧が表示され，`Export`ボタンを押してください．
4. 表示されたウィンドウの中に，`Format`という項目があります．`STEP`を選択してください．
5. 最後に，青色の`Export`ボタンを押してダウンロードが開始されます．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### 電子回路図

![SOBIT HOME Circuit](sobit_home/docs/img/sobit_home_circuit.png)

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- ### ロボットの組み立て

TBD

<p align="right">(<a href="#readme-top">上に戻る</a>)</p> -->


### ロボットの特徴

<!-- | 項目 | 詳細 |
| --- | --- |
| 最大直線速度 | 0.8[m/s] |
| 最大回転速度 | 0.229[rad/s] |
| ベース最大積載量 | 20[kg] |
| マニピュレータ最大積載量 | 1.0[kg] |
| サイズ (LxWxH) | 400 x 450 x 1000[mm] |
| 重量 | 16.0[kg] |
| リモートコントローラー | PS4 |
| LiDAR | 不明 |
| RGB-D | RealSense D415（ヘッド）、RealSense D405（ハンド） |
| スピーカー | Jabra Speak 710 |
| マイク | MKE 400 |
| アクチュエータ（アーム） | XM540-W150 ×4、XM430-W320 ×6 |
| 電源 | マキタ 6.0Ah 18V |
| PC接続 | USB + 無線（カチャカ） | -->

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### 部品リスト（BOM）

<!-- アクチュエータ・センサ・計算機・機構部品の一覧．数量は実機（Dynamixelバススキャン＋センサへの実照会）で確認済み（右ハンドは左の構成をミラー）．単価とリンクは2026-08-30時点 — † = 在庫切れ/長納期，‡ = 価格未確認，n/a = その地域で販売元未確認，「login」= MISUMIはログイン後に価格表示．価格はすべて単価（合計は数量を乗算）． -->

**概算費用（日本欄・税込・単価×数量）：必須部品 ≈ ¥3,502,141（電装 ≈ ¥3,130,635 ＋ 機構 ≈ ¥371,506）／オプション ≈ ¥110,299．**
<!-- LiDARは概算見積値，‡/†の価格も記載値のまま合算． -->

#### アクチュエータ・駆動系（小計 ≈ ¥2,072,415）

| 部位 | 部品 | 型番 | 数量 | 日本 | 米国 | 欧州 |
| --- | --- | --- | --- | --- | --- | --- |
| Head + Arms | Dynamixel Actuator | XM430-W350-R | 6 | [¥31,504†](https://e-shop.robotis.co.jp/product.php?id=44) | [$333.39](https://www.robotis.us/dynamixel-xm430-w350-r/) | [€325.56](https://www.generationrobots.com/en/402477-xm430-w350-r-dynamixel-servomotor.html) |
| Arms + Mobile base | Dynamixel Actuator | XM540-W270-R | 16 | [¥51,865†](https://e-shop.robotis.co.jp/product.php?id=43) | [$494.39†](https://www.robotis.us/dynamixel-xm540-w270-r/) | [€489.96†](https://www.generationrobots.com/en/402931-dynamixel-xm540-w270-r-servo.html) |
| Mobile base | Dynamixel Actuator | XM540-W150-R | 2 | [¥51,865](https://e-shop.robotis.co.jp/product.php?id=42) | [$494.39](https://www.robotis.us/dynamixel-xm540-w150-r/) | [€489.96](https://www.generationrobots.com/en/402832-servomoteur-dynamixel-xm540-w150-r.html) |
| Hands | Dynamixel Actuator (Hand) | XL330-M288-T | 14 | [¥4,070†](https://e-shop.robotis.co.jp/product.php?id=417) | [$27.49](https://www.robotis.us/dynamixel-xl330-m288-t/) | [€41.95†](https://www.mybotshop.de/DYNAMIXEL-XL330-M288-T_1) |
| Arms | Dynamixel PRO Actuator (Shoulder) | PH54-200-S500-R | 2 | [¥354,200†](https://e-shop.robotis.co.jp/product.php?id=285) | [$3,541.89†](https://www.robotis.us/dynamixel-ph54-200-s500-r/) | [€3,782.06†](https://www.generationrobots.com/en/403321-dynamixel-pro-plus-h54p-200-s500-r-servo.html) |
| Torso | Dynamixel USB Interface | U2D2 | 4 | [¥6,963](https://e-shop.robotis.co.jp/product.php?id=190) | [$36.92](https://www.robotis.us/u2d2/) | n/a |
| Torso | Dynamixel Power Hub | U2D2 PHB Set | 2 | [¥4,180](https://e-shop.robotis.co.jp/product.php?id=325) | [$21.85](https://www.robotis.us/u2d2-power-hub-board-set/) | n/a |
| Mobile base | Wheel Drive Motor | RoboMaster M3508 P19 | 4 | [¥11,500†](https://store.dji.com/jp/product/rm-m3508-p19-brushless-dc-gear-motor) | [$115.00†](https://www.seeedstudio.com/RoboMaster-M3508-P19-Brushless-DC-Gear-Motor-p-2904.html) | n/a |
| Mobile base | Wheel Motor ESC | RoboMaster C620 | 4 | [¥8,900†](https://store.dji.com/jp/product/rm-c620-brushless-dc-motor-speed-controller) | [$89.00†](https://www.seeedstudio.com/RoboMaster-C620-Brushless-DC-Motor-Speed-Controller-p-2905.html) | n/a |
| Torso (lift) | Lift Motor (NEMA 34 CAN servo stepper, brake) | UIrobot UIM8696CAB | 1 | [¥43,706](https://www.amazon.com/dp/B0DK4W9FTY) ([quote](https://jpacontrol.com/products/show-50.html)) | [‡](https://www.amazon.com/UIROBOT-Stepper-Integrated-Controller-24-48VDC/dp/B0FJQPX36P) (UIM8696CA, no brake) | n/a |
| Torso (lift) | Lift Motor Gateway (RS232 to CAN) | UIrobot UIM2513 | 1 | [¥12,806](https://www.amazon.com/UIROBOT-Adapters-Converter-Hardware-Isolation/dp/B0CNSMJHB1) ([quote](https://jpacontrol.com/products/show-29.html)) | [$125.99‡](https://www.amazon.com/UIROBOT-Adapters-Converter-Hardware-Isolation/dp/B0CNSMJHB1) | n/a |
| Torso (lift) | Lift Motor USB-RS232 Adapter (isolated) | UIrobot UIC321H | 1 | [¥4,481](https://us.amazon.com/UIROBOT-Universal-Converter-Industrial-Ultra-Flexible/dp/B0FKY29JP3) ([quote](https://jpacontrol.com/products/show-63.html)) | [‡](https://us.amazon.com/UIROBOT-Universal-Converter-Industrial-Ultra-Flexible/dp/B0FKY29JP3) | n/a |
| Mobile base | USB-CAN Adapter (isolated, CAN FD) | DSD TECH SH-C31G (CANable 2.0 Pro) | 1 | [¥3,849](https://www.amazon.co.jp/dp/B0FHHCSZY8) | n/a | n/a |
| Mobile base | CAN Bus Hub (CANバスハブ) | — | 1 | [¥1,787](https://www.amazon.co.jp/dp/B0G814QVFB) | n/a | n/a |

#### センサ・AV（小計 ≈ ¥520,883）

| 部位 | 部品 | 型番 | 数量 | 日本 | 米国 | 欧州 |
| --- | --- | --- | --- | --- | --- | --- |
| Head | RGB-D Camera (Head) | Orbbec Gemini 336L | 1 | [¥68,420](https://www.digikey.jp/ja/product-highlight/o/orbbec/gemini-330-series-stereo-depth-cameras) | [$379.00](https://store.orbbec.com/products/gemini-336l) | [€586.31](https://www.mybotshop.de/Orbbec-Gemini-336L_1) |
| Mobile base | 2D LiDAR | Hokuyo UST-10LX | 2 | [quote (approx ¥165,000)](https://www.hokuyo-aut.co.jp/search/single.php?serial=16) | [$1,200.00](https://acroname.com/store/scanning-laser-rangefinder-ethernet-r359-ust-10lx) | [€1,878.00†](https://www.generationrobots.com/en/401755-hokuyo-ust-10lx-scanning-laser-range-finder.html) |
| Hands | Wrist Camera (AR0234 global shutter, 1920x1200@90fps, 120deg) | ELP-USBGS1200P01-H120 | 2 | [¥6,066](https://www.amazon.co.jp/dp/B08FD2N9WG) | [$70.64‡](https://www.elpcctv.com/elp-2mp-global-shutter-1200p-1080p-90fps-no-distortion-120-degree-usb-camera-p-557.html) | n/a |
| Torso | Speaker (USB/BT speakerphone) | Jabra Speak 710 | 1 | [¥58,800](https://www.amazon.co.jp/dp/B06XX2N987) | [‡](https://www.jabra.com/business/speakerphones/jabra-speak-series/jabra-speak-710) | n/a |
| Head | Microphone | RØDE VideoMic GO II HELIX (VMGOIIH) | 1 | [¥15,455](https://www.amazon.co.jp/dp/B0D6X93C58/) | [$94.99‡](https://www.sweetwater.com/store/detail/VideoMicGo2H--rode-videomic-go-ii-camera-mounted-shotgun-microphone) | [€85.00](https://www.thomann.de/de/rode_videomic_go_ii.htm) |
| Head | Head Display (8in 1280x800 IPS touch, HDMI) | Waveshare 8DP-CAPLCD | 1 | [¥13,097](https://jp.robotshop.com/products/waveshare-8inch-capacitive-touch-display-toughened-glass-1280x800-ips-hdmi) | [$69.99](https://www.waveshare.com/8dp-caplcd.htm) | [€79.90](https://www.botnroll.com/en/5-89/5242-8-0inch-capacitive-touch-display-toughened-glass-1280-800-hdmi-ips-optical-bonding-screen-waveshare-23741.html) |
| Torso (rear display) | Rear Monitor (16in 4K portable, USB-C/HDMI) | Acouto Zen16 Ultra | 1 | [¥22,979](https://www.amazon.co.jp/dp/B0G7H44RGD) | n/a | n/a |

#### 計算機・ネットワーク（小計 ≈ ¥314,848）

| 部位 | 部品 | 型番 | 数量 | 日本 | 米国 | 欧州 |
| --- | --- | --- | --- | --- | --- | --- |
| Torso | Ethernet Switch (5-port GbE) | TP-Link TL-SG605 | 1 | [¥2,033](https://www.amazon.co.jp/dp/B0DP2KSNCK) | n/a | n/a |
| Torso | USB Hub (4-port USB3.0, powered) | UGREEN | 2 | [¥2,999](https://www.amazon.co.jp/dp/B08Y8CJKJC) | n/a | n/a |
| Torso | USB Hub (USB-C 6-in-1, 100W PD) | UGREEN Revodok | 2 | [¥2,999](https://www.amazon.co.jp/dp/B0D1XLNWP2) | n/a | n/a |
| Mobile base (lidar) | LAN Extension Connector (RJ45, 2-pack) | UGREEN | 2 | [¥1,274](https://www.amazon.co.jp/dp/B0DMF8G398) | n/a | n/a |
| Torso | PC | Intel NUC 12 Pro Kit NUC12WSHi5 (RNUC12WSHI50000, i5-1240P) | 1 | [¥188,800](https://www.amazon.co.jp/dp/B0BCWDST4J/) | [$519.00](https://www.newegg.com/asus-barebone-systems-mini-pc-12th-gen-intel-core-i5-1240p/p/2SW-000N-00046) | [€444.00†](https://www.alternate.de/ASUS/NUC-13-Pro-Tall-Kit-NUC13ANHi5-Barebone/html/product/100052198) (NUC 13 substitute) |
| Torso | PC RAM (2x16GB DDR4-3200) | Crucial CT2K16G4SFRA32A | 1 | [¥44,491](https://kakaku.com/item/K0001372325/) | [$229.00](https://www.newegg.com/crucial-32gb-ddr4-3200-cas-latency-cl22-laptop-memory/p/N82E16820156263) | [€264.03‡](https://www.amazon.de/dp/B08C4X9VR5) |
| Torso | PC SSD (NVMe 500GB) | Crucial P5 Plus CT500P5PSSD8 (discontinued — successor: T500) | 1 | [¥64,980‡](https://kakaku.com/item/K0001588760/) (T500) | [$72.84†](https://www.sabrepc.com/CT500P5PSSD8-Crucial-S4602797) | [€204.90†](https://www.reichelt.de/de/de/shop/produkt/crucial_t500_pcie_4_0_nvme_m_2_ssd_500_gb-405533) (T500) |

#### 電源（小計 ≈ ¥196,376）

| 部位 | 部品 | 型番 | 数量 | 日本 | 米国 | 欧州 |
| --- | --- | --- | --- | --- | --- | --- |
| Mobile base | Battery 18V 9.0Ah | BL1890 | 5 | [¥26,318](https://makitashop.jp/?pid=189052500) | [$92.99](https://www.vanonbatteries.com/products/for-makita-9000mah-18v-bl1830-bl1840-bl1845-bl1850-bl1860-bl1890-lxt-li-ion-battery2-pack) (Vanon, 2-pack) | [€124.50](https://geizhals.de/makita-bl1890-lxt-werkzeug-akku-18v-1915h4-0-a3589148.html) |
| Mobile base | Battery Connector (18V DIY power connector, 2-pc set) | Gakkiti (Makita compatible) | 2 | [¥1,599](https://www.amazon.co.jp/dp/B094XXC8LL) | n/a | n/a |
| Mobile base | Battery Adapter (2x18V to 36V) | Makita BCV03 (A-57255 / 196809-7) | 1 | [¥14,190](https://makitashop.jp/?pid=108094683) | [$113.99](https://dryitcenter.com/products/makita-36v-adaptor-cordless-bcv03) | [€89.25](https://geizhals.de/makita-bcv03-2x-18v-adapter-fuer-akkus-196809-7-a2202258.html) |
| Offboard (charger) | Battery Charger (2-port rapid; alt: DC18RC / DC18WC) | Makita DC18RD | 1 | [¥16,200](https://search.kakaku.com/dc18rd/) | [$164.99†](https://www.masterwholesale.com/makita-dc18rd-18v-lxt-lithium-ion-dual-port-rapid-optimum-charger.html) | [€71.88](https://geizhals.de/makita-dc18rd-ladegeraet-196933-6-a1292788.html) |
| Mobile base | DC-DC Converter (30-90V to 24V, 20A/480W) | Mzhou buck converter | 1 | [¥7,499](https://www.amazon.co.jp/dp/B0D1G6P599) | n/a | n/a |
| Mobile base | DC-DC Converter (24V to 12V, 30A/360W) | — | 3 | [¥3,799](https://www.amazon.co.jp/dp/B0976VJ5CS) | n/a | n/a |
| Mobile base | DC-DC Converter (12/24V to 5V, 20A/100W, waterproof) | HOMELYLIFE step-down | 2 | [¥4,303](https://www.amazon.co.jp/dp/B089M5R3NJ) | n/a | n/a |
| Mobile base | Screw Terminal Block (8P, 2-row) | — | 1 | [¥3,696](https://www.amazon.co.jp/dp/B0GT8QMDZZ) | n/a | n/a |

#### ケーブル・その他（小計 ≈ ¥26,113）

| 部位 | 部品 | 型番 | 数量 | 日本 | 米国 | 欧州 |
| --- | --- | --- | --- | --- | --- | --- |
| Torso | HDMI 2.1 Cable (2m, 8K) | UGREEN | 1 | [¥1,601](https://www.amazon.co.jp/dp/B0CFFFSFFN) | n/a | n/a |
| Torso (cabling) | Drag Chain (15x30mm, 1m) | Akozon | 4 | [¥1,492](https://www.amazon.co.jp/dp/B07YG2C1D7) | n/a | n/a |
| Torso (cabling) | USB 3.0 Extension Cable (2m) | UGREEN | 4 | [¥1,038](https://www.amazon.co.jp/dp/B086ZJB2JN) | n/a | n/a |
| Torso (cabling) | USB-C Cable (L-shape, 100W, 4K, 2m) | UGREEN | 2 | [¥2,880](https://www.amazon.co.jp/dp/B08R86PLCS) | n/a | n/a |
| Torso (cabling) | Micro-USB Cable (2m) | UGREEN | 3 | [¥954](https://www.amazon.co.jp/dp/B07VNM61ZL) | n/a | n/a |
| Torso (cabling) | Micro-USB Cable (0.5m) | UGREEN | 1 | [¥674](https://www.amazon.co.jp/dp/B07VQTRYY4) | n/a | n/a |
| Mobile base (lidar, NUC) | LAN Cable (CAT8 mesh, short) | UGREEN | 3 | [¥1,099](https://www.amazon.co.jp/dp/B0CMWFN5R8) | n/a | n/a |
| Mobile base (Remote PC) | LAN Cable (CAT8 mesh, long) | UGREEN | 1 | [¥1,799](https://www.amazon.co.jp/dp/B0CMWFN5R8) | n/a | n/a |

#### 機構部品（小計 ≈ ¥371,506）

| 部位 | 部品 | 型番 | 数量 | 日本 | 米国 | 欧州 |
| --- | --- | --- | --- | --- | --- | --- |
| Mobile base | Drive Wheel (150mm rubber, 12mm shaft) | Inoac LR-150W-BK-12 | 4 | [¥2,013](https://www.genbaichiba.com/shop/g/g00621419/) | n/a | n/a |
| Mobile base | Timing Pulley (S5M, 20T, 10mm belt) | MISUMI HTPS20S5M100-A-P10 | 8 | [¥3,343](https://jp.misumi-ec.com/vona2/detail/110300406030/?HissuCode=HTPS20S5M100-A-P10) | [$52.67](https://us.misumi-ec.com/vona2/detail/110300406030/?HissuCode=HTPS20S5M100-A-P10) | [€44.84](https://de.misumi-ec.com/vona2/detail/110300406030/?HissuCode=HTPS20S5M100-A-P10) |
| Mobile base | Timing Belt (S5M, 300mm, 10mm wide) | MISUMI HTBN300S5M-100 | 4 | [¥736](https://jp.misumi-ec.com/vona2/detail/110302653030/?HissuCode=HTBN300S5M-100) | [$15.79](https://us.misumi-ec.com/vona2/detail/110302653030/?HissuCode=HTBN300S5M-100) | [€10.36](https://de.misumi-ec.com/vona2/detail/110302566230/?HissuCode=HTBN300S5M-100) |
| Mobile base | Spur Gear (module 1.0, 40T, 8mm face, 10mm bore) | MISUMI GEAB1.0-40-8-B-10 | 4 | [¥1,533](https://jp.misumi-ec.com/vona2/detail/110300428430/?HissuCode=GEAB1.0-40-8-B-10) | [$40.44‡](https://us.misumi-ec.com/vona2/detail/110300428430/?HissuCode=GEAB1.0-40-8-B-10) | n/a |
| Mobile base | Spur Gear (module 1.0, 40T, hub, 15mm bore) | MISUMI GEAHB1.0-40-8-A-15 | 4 | [¥1,477](https://jp.misumi-ec.com/vona2/detail/110300428430/?HissuCode=GEAHB1.0-40-8-A-15) | [‡](https://us.misumi-ec.com/vona2/detail/110300428430/?HissuCode=GEAHB1.0-40-8-A-15) | n/a |
| Mobile base | Miter Gear (module 1.5, 20T 1:1, SUS304) | MISUMI KGTS1.5-2020-10 | 8 | [¥8,973](https://jp.misumi-ec.com/vona2/detail/110300429650/?HissuCode=KGTS1.5-2020-10) | [$130.65‡](https://us.misumi-ec.com/vona2/result/?Keyword=KGTS1.5-2020-10) | n/a |
| Mobile base | Precision Shaft (10mm dia; L=55/60/120/130) | MISUMI PSSFG10-55/-60/-120/-130 | 4 each | [¥485-802](https://jp.misumi-ec.com/vona2/detail/110302634310/?HissuCode=PSSFG10-55) | [$7.96-13.69](https://us.misumi-ec.com/vona2/detail/110302634310/?HissuCode=PSSFG10-55) | [login](https://de.misumi-ec.com/vona2/detail/110302634310/?HissuCode=PSSFG10-55) |
| Mobile base | Bearing Spacer (10x12mm, L=2) | MISUMI CLBUB10-12-2 | 32 | [¥496](https://jp.misumi-ec.com/vona2/detail/110302644450/?HissuCode=CLBUB10-12-2) | [$22.52](https://us.misumi-ec.com/vona2/detail/110302644450/?HissuCode=CLBUB10-12-2) | n/a |
| Mobile base | Bearing Spacer (10x12mm, L=30) | MISUMI CLBUB10-12-30 | 4 | [¥707](https://jp.misumi-ec.com/vona2/detail/110302644450/?HissuCode=CLBUB10-12-30) | [$24.15‡](https://us.misumi-ec.com/vona2/detail/110302644450/?HissuCode=CLBUB10-12-30) | [€7.84](https://de.misumi-ec.com/vona2/detail/110302644450/?HissuCode=CLBUB10-12-30) |
| Mobile base | Shaft Support (flanged, slit, 10mm dia) | MISUMI SSTHMR10 | 12 | [¥2,166](https://jp.misumi-ec.com/vona2/detail/110300013150/?HissuCode=SSTHMR10) | [login](https://us.misumi-ec.com/vona2/detail/110300013150/?HissuCode=SSTHMR10) | [€28.23](https://de.misumi-ec.com/vona2/detail/110300013150/?HissuCode=SSTHMR10) |
| Mobile base | Flanged Bearing (stainless, 10x15x4) | SFL6700ZZ | 48 | [¥1,099](https://jp.misumi-ec.com/vona2/detail/110302273720/?HissuCode=SFL6700ZZ) | [$19.99](https://vxb.com/products/sf6700zz-stainless-steel-flanged-shielded-bearing-10x15x4) | [€5.31](https://www.123kugellager.de/kugellager-gehauselager/rillenkugellager/einreihig/f6700-zz) (steel equiv.) |
| Mobile base | Bearing (40x52x7) | 6808ZZ | 4 | [¥1,804](https://jp.misumi-ec.com/vona2/detail/221000058301/?HissuCode=6808ZZ) | [$19.99](https://vxb.com/products/6808zz-bearing-40x52x7-shielded) | [€3.25](https://www.hug-technik.com/shop/kugellager-61808-2z-von-zen-rillenkugellager-40x52x7-mm.html) (ZEN) |
| Mobile base | Thrust Bearing (55x78x16) | 51111 | 4 | [¥2,400](https://jp.misumi-ec.com/vona2/detail/221000058299/?HissuCode=51111) | [$49.99](https://vxb.com/products/51111-thrust-bearing-55x78x16) | [€7.71](https://www.hug-technik.com/shop/axial-rillenkugellager-51111-von-zen-55x78x16-mm.html) (ZEN) |
| Mobile base | Standoff (SUS303, M4, 45mm, M-F slotted) | Hirosugi BRU-445S | 16 | [¥280](https://www.hirosugi-net.co.jp/shop/g/g41214/) (20-pc min) | n/a | n/a |
| Mobile base | Standoff (SUS303, M4, 50mm, M-F slotted) | Hirosugi BRU-450S | 16 | [¥294](https://www.hirosugi-net.co.jp/shop/g/g41215/) (20-pc min) | n/a | n/a |
| Mobile base | Aluminium Frame (20x20, 472mm) | MISUMI HFS5-2020-472 | 4 | [¥283](https://jp.misumi-ec.com/vona2/detail/110302683830/?HissuCode=HFS5-2020-472) | [login](https://us.misumi-ec.com/vona2/detail/110302683830/?HissuCode=HFS5-2020-472) | [login](https://de.misumi-ec.com/vona2/result/?Keyword=HFS5-2020) |
| Mobile base | Aluminium Frame (20x20, 432mm) | MISUMI HFS5-2020-432 | 10 | [¥253](https://jp.misumi-ec.com/vona2/detail/110302683830/?HissuCode=HFS5-2020-432) | [login](https://us.misumi-ec.com/vona2/detail/110302683830/?HissuCode=HFS5-2020-432) | [login](https://de.misumi-ec.com/vona2/result/?Keyword=HFS5-2020) |
| Mobile base | Aluminium Frame (20x20, 236mm) | MISUMI HFS5-2020-236 | 4 | [¥325](https://jp.misumi-ec.com/vona2/detail/110302683830/?HissuCode=HFS5-2020-236) | [login](https://us.misumi-ec.com/vona2/detail/110302683830/?HissuCode=HFS5-2020-236) | [login](https://de.misumi-ec.com/vona2/result/?Keyword=HFS5-2020) |
| Mobile base | Aluminium Frame (20x20, 78mm) | MISUMI HFS5-2020-78 | 8 | [¥325](https://jp.misumi-ec.com/vona2/detail/110302683830/?HissuCode=HFS5-2020-78) | [login](https://us.misumi-ec.com/vona2/detail/110302683830/?HissuCode=HFS5-2020-78) | [login](https://de.misumi-ec.com/vona2/result/?Keyword=HFS5-2020) |
| Mobile base (battery tray) | Telescopic Slide Rail (3-step, SUS304, W27) | MISUMI SSRXC2718 | 2 | [¥4,461](https://jp.misumi-ec.com/vona2/detail/110300072560/?HissuCode=SSRXC2718) | [login](https://us.misumi-ec.com/vona2/detail/110300072560/?HissuCode=SSRXC2718) | [login](https://uk.misumi-ec.com/vona2/detail/110300072560/?HissuCode=SSRXC2718) |
| Mobile base (battery tray) | Aluminium Frame (20x20, 200mm) | MISUMI HFS5-2020-200 | 4 | [¥325](https://jp.misumi-ec.com/vona2/detail/110302683830/?HissuCode=HFS5-2020-200) | [login](https://us.misumi-ec.com/vona2/detail/110302683830/?HissuCode=HFS5-2020-200) | [login](https://de.misumi-ec.com/vona2/result/?Keyword=HFS5-2020) |
| Torso lift | Lift Ball Screw (rolled, 20mm dia, 10mm lead, 860mm) | MISUMI C-BSSTA2010-860 | 1 | [¥24,475](https://jp.misumi-ec.com/vona2/detail/110302588540/?HissuCode=C-BSSTA2010-860) | n/a | n/a |
| Torso lift | Lift Linear Guide Rail (SX, W28, 1000mm) | MISUMI SXR28-1000 | 2 | [¥7,904](https://jp.misumi-ec.com/vona2/detail/110300048850/?HissuCode=SXR28-1000) | [login](https://us.misumi-ec.com/vona2/detail/110300048850/?HissuCode=SXR28-1000) | [login](https://uk.misumi-ec.com/vona2/detail/110300048850/?HissuCode=SXR28-1000) |
| Torso lift | Lift Coupling (disc type, OD32, 12/14mm bores) | MISUMI MCSLCRK32-12-14 | 1 | [¥4,694](https://jp.misumi-ec.com/vona2/detail/110302556340/?HissuCode=MCSLCRK32-12-14) | [login](https://us.misumi-ec.com/vona2/detail/110302556340/?HissuCode=MCSLCRK32-12-14) | [login](https://uk.misumi-ec.com/vona2/detail/110302556340/?HissuCode=MCSLCRK32-12-14) |
| Torso lift | Lift Ball Screw Support Unit | MISUMI C-TFF15 | 1 | [¥2,376](https://jp.misumi-ec.com/vona2/detail/110310392309/?HissuCode=C-TFF15) | n/a | n/a |
| Torso frame | Aluminium Frame (20x20, 600mm) | MISUMI HFS5-2020-600 | 1 | [¥364](https://jp.misumi-ec.com/vona2/detail/110302683830/?HissuCode=HFS5-2020-600) | [$9.42‡](https://us.misumi-ec.com/vona2/detail/110302683830/?HissuCode=HFS5-2020-600) | [login](https://de.misumi-ec.com/vona2/result/?Keyword=HFS5-2020) |
| Torso frame | Aluminium Frame (20x20, 100mm) | MISUMI HFS5-2020-100 | 6 | [¥325](https://jp.misumi-ec.com/vona2/detail/110302683830/?HissuCode=HFS5-2020-100) | [$4.66‡](https://us.misumi-ec.com/vona2/detail/110302683830/?HissuCode=HFS5-2020-100) | [login](https://de.misumi-ec.com/vona2/result/?Keyword=HFS5-2020) |
| Torso frame | Aluminium Frame (20x20, 110mm) | MISUMI HFS5-2020-110 | 1 | [¥325](https://jp.misumi-ec.com/vona2/detail/110302683830/?HissuCode=HFS5-2020-110) | [$4.66‡](https://us.misumi-ec.com/vona2/detail/110302683830/?HissuCode=HFS5-2020-110) | [login](https://de.misumi-ec.com/vona2/result/?Keyword=HFS5-2020) |
| Torso (back) | Emergency Stop Button | IDEC HW1B-V404R | 1 | [¥3,504](https://jp.misumi-ec.com/vona2/detail/222000393180/?HissuCode=HW1B-V404R) | [login](https://us.misumi-ec.com/vona2/detail/222000393180/?HissuCode=HW1B-V404R) | n/a |
| Arms + steer (X540 joints) | Flanged Bearing (12x18x4) | F6701ZZ | 18 | [¥1,239](https://jp.misumi-ec.com/vona2/detail/221000529012/?HissuCode=F6701ZZ) | [$6.19](https://bearingsdirect.com/f6701-zz-flanged-ball-bearing-12x18x4mm-shielded/) | [€2.20‡](https://rcbay.de/Kugellager-mit-Bund-F6701-ZZ-12x18x4-mm-Flanschlager-Bundlager) |
| Arms (shoulder) | Bearing (50x65x7) | 6810ZZ | 6 | [¥2,330](https://jp.misumi-ec.com/vona2/detail/221000058301/?HissuCode=6810ZZ) | [$39.99](https://vxb.com/products/6810-open-bearing-50x65x7) (open) | [€4.52](https://www.hug-technik.com/shop/kugellager-61810-von-zen-rillenkugellager-50x65x7-mm.html) (ZEN, open) |
| Arms (joints) | Bearing (20x27x4) | 6704ZZ | 6 | [¥1,268](https://jp.misumi-ec.com/vona2/detail/221000058301/?HissuCode=6704ZZ) | [$9.99](https://vxb.com/products/6704zz-shielded-bearing-20x27x4) | [€10.59](https://www.123kugellager.de/kugellager-gehauselager/rillenkugellager/einreihig/61704-zz) |
| Arms (upper arm) | Aluminium Frame (20x20, 180mm; verified from CAD) | MISUMI HFS5-2020-180 | 2 | [¥325](https://jp.misumi-ec.com/vona2/detail/110302683830/?HissuCode=HFS5-2020-180) | [login](https://us.misumi-ec.com/vona2/detail/110302683830/?HissuCode=HFS5-2020-180) | [login](https://de.misumi-ec.com/vona2/result/?Keyword=HFS5-2020) |
| Arms (forearm) | Aluminium Frame (20x20, 330mm; verified from CAD) | MISUMI HFS5-2020-330 | 2 | [¥191](https://jp.misumi-ec.com/vona2/detail/110302683830/?HissuCode=HFS5-2020-330) | [login](https://us.misumi-ec.com/vona2/detail/110302683830/?HissuCode=HFS5-2020-330) | [login](https://de.misumi-ec.com/vona2/result/?Keyword=HFS5-2020) |

#### オプション（小計 ≈ ¥110,299）

| 部位 | 部品 | 型番 | 数量 | 日本 | 米国 | 欧州 |
| --- | --- | --- | --- | --- | --- | --- |
| (Optional) Torso | USB-C GbE LAN Adapter | UGREEN | 1 | [¥2,099](https://www.amazon.co.jp/dp/B082K62S48) | n/a | n/a |
| (Optional) Torso | Mobile WiFi Router (+ home kit) | Fujisoft +F FS040W | 1 | [¥33,000](https://www.amazon.co.jp/dp/B09HRH6XBL) | n/a | n/a |
| (Optional) Offboard (teleop) | Game Controller | Sony DualShock 4 (CUH-ZCT2J) | 1 | [¥15,800](https://www.amazon.co.jp/dp/B01LPTFJ8W) | n/a | n/a |
| (Optional) Offboard (teleop) | VR Headset | Meta Quest 3S 128GB | 1 | [¥59,400](https://www.amazon.co.jp/dp/B0F8VJ57Q1) | n/a | n/a |

<!-- サーボ配置（バススキャンより）：頭部パン/チルト = XM430-W350 ×2；腕1本 = PH54-200（肩）×1 + XM540-W270 ×7 + XM430-W350（手首）×2；ハンド1個 = XL330-M288 ×7；ステア = XM540-W150 ×2 + XM540-W270 ×2． -->

<!-- 今後追加予定：USB延長ケーブル，電源ケーブル，電源ハブ，ネジ・ナット類，3Dプリント用フィラメント． -->

> [!IMPORTANT]
> 価格は販売店により変動する場合があります．最新の価格は各リンクをご確認ください．

</details>

<!-- > [!NOTE]
> 日本のサイト・値段(円)に更新していく予定です． -->

<!-- | 部品 | 型番 | 数量 | おおよその単価 | 購入先 |
| --- | --- | --- | --- | --- |
| カチャカ | B1A01 | 1 | ¥245,000 | [リンク](https://store.kachaka.life/products/detail/50) |
| カチャカベース | ksh0003 | 1 | ¥13,500 | [リンク](https://store.kachaka.life/products/detail/57) |
| マキタバッテリー | BL1860B | 1 | ¥28,300 | [リンク](https://www.makitatools.com/products/details/BL1860B) |
| マキタアダプター | B0D6R6XSPX | 1 | ¥4,000 | [リンク](https://www.amazon.co.jp/dp/B0D6R6XSPX) |
| ダイナミクセルアクチュエータ | XM430-W350-R | 6 | ¥49,600 | [リンク](https://www.robotis.us/dynamixel-xm430-w350-r/) |
| ダイナミクセルアクチュエータ | XM540-W150-R | 4 | ¥73,600 | [リンク](https://www.robotis.us/dynamixel-xm540-w150-r/) |
| ダイナミクセルフレーム | FR12-S102K セット | 2 | ¥3,200 | [リンク](https://www.robotis.us/fr12-s102k-set) |
| ダイナミクセルフレーム | FR12-H101K セット | 1 | ¥6,900 | [リンク](https://www.robotis.us/fr12-h101k-set/) |
| ダイナミクセルフレーム | FR12-H104K セット | 1 | ¥6,500 | [リンク](https://www.robotis.us/fr12-h104k-set/) |
| ダイナミクセルフレーム | FR13-H101K セット | 1 | ¥11,400 | [リンク](https://www.robotis.us/fr13-h101k-set/) |
| ダイナミクセル U2D2 | 8809052930103 | 1 | ¥5,500 | [リンク](https://www.robotis.us/u2d2/) |
| ダイナミクセル パワーハブ | 8809052930530 | 1 | ¥5,500 | [リンク](https://www.robotis.us/u2d2-power-hub-board-set/) |
| (オプション) USBハブ | B0D1XVNTHJ | 1 | ¥3,700 | [リンク](https://www.amazon.co.jp/dp/B0D1XVNTHJ) |
| (オプション) スピーカー | Jabra Speak 710 | 1 | ¥36,000 | [リンク](https://www.jabra.com/business/speakerphones/jabra-speak-series/jabra-speak-710/) |
| (オプション) マイク | MKE 400 | 1 | ¥30,300 | [リンク](https://www.sennheiser.com/en-ae/catalog/products/microphones/mke-400/mke-400-508898) |
| RealSense | D415 | 1 | ¥42,000 | [リンク](https://www.amazon.co.jp/dp/B07JVGRQZT) |
| (オプション) RealSense | D405 | 1 | ¥42,800 | [リンク](https://www.amazon.co.jp/dp/B09JBBHVTY) |
| (オプション) 非常停止ボタン | HW1B-X411R-MAU | 1 | ¥13,600 | [リンク](https://jp.misumi-ec.com/vona2/detail/222000393180/?HissuCode=HW1B-X411R-MAU) |
| (オプション) M5Stack Basic V2.7 | K001-V27 | 1 | ¥6,200 | [リンク](https://shop.m5stack.com/products/esp32-basic-core-lot-development-kit-v2-7) |
| (オプション) ESP32 DevKitC-1-N16R8 | B0DWWY5KTZ | 1 | ¥1,500 | [リンク](https://www.amazon.co.jp/dp/B0DWWY5KTZ) |
| (オプション) ディスプレイ | B01CZL6QIQ | 2 | ¥2,200 | [リンク](https://www.amazon.co.jp/dp/B01CZL6QIQ) |
| スラストローラーベアリング | AXK1104 | 2 | ¥1,800 | [リンク](https://jp.misumi-ec.com/vona2/detail/221000058345/?HissuCode=AXK1106) |
| スラストローラーベアリング | AXK1106 | 1 | ¥1,400 | [リンク](https://jp.misumi-ec.com/vona2/detail/221000058345/?HissuCode=AXK1106) |
| アルミフレーム | HFS5-2020-600 | 1 | ¥1,500 | [リンク](https://jp.misumi-ec.com/vona2/detail/110302683830/?HissuCode=HFS5-2020-600) |
| アルミフレーム | HFS5-2020-100 | 6 | ¥750 | [リンク](https://jp.misumi-ec.com/vona2/detail/110302683830/?HissuCode=HFS5-2020-100) |
| アルミフレーム | HFS5-2020-110 | 1 | ¥750 | [リンク](https://jp.misumi-ec.com/vona2/detail/110302683830/?HissuCode=HFS5-2020-110) |
| ブラケット | HBLFSNK6 | 3 | ¥270 | [リンク](https://jp.misumi-ec.com/vona2/detail/110300442520/?HissuCode=HBLFSNK6) |
| 六角穴付ボルト | CSH-ST-M2-4 | 16 | ¥190 | [リンク](https://jp.misumi-ec.com/vona2/detail/221000551286/?HissuCode=CSH-ST-M2-4) |
| 六角穴付ボルト | CSH-ST-M2.5-5 | 54 | ¥60 | [リンク](https://jp.misumi-ec.com/vona2/detail/221000551286/?HissuCode=CSH-ST-M2.5-5) |
| 六角穴付ボルト | CSH-ST-M2.5-6 | 16 | ¥180 | [リンク](https://jp.misumi-ec.com/vona2/detail/221000551286/?HissuCode=CSH-ST-M2.5-6) |
| 六角穴付ボルト | CSH-ST-M2.5-8 | 34 | ¥110 | [リンク](https://jp.misumi-ec.com/vona2/detail/221000551286/?HissuCode=CSH-ST-M2.5-8) |
| 六角穴付ボルト | CSH-ST-M2.5-10 | 10 | ¥180 | [リンク](https://jp.misumi-ec.com/vona2/detail/221000551286/?HissuCode=CSH-ST-M2.5-10) |
| 六角穴付ボルト | CSH-ST-M2.5-12 | 16 | ¥180 | [リンク](https://jp.misumi-ec.com/vona2/detail/221000551286/?HissuCode=CSH-ST-M2.5-12) |
| 六角穴付ボルト | CSH-ST-M3-5 | 4 | ¥400 | [リンク](https://jp.misumi-ec.com/vona2/detail/221000551286/?HissuCode=CSH-ST-M3-5) |
| 六角穴付ボルト | CSH-ST-M4-15 | 16 | ¥180 | [リンク](https://jp.misumi-ec.com/vona2/detail/221000551286/?HissuCode=CSH-ST-M4-15) |
| 六角穴付ボルト | CSH-ST-M5-8 | 50 | ¥40 | [リンク](https://jp.misumi-ec.com/vona2/detail/221000551286/?HissuCode=CSH-ST-M5-8) |
| 六角穴付ボルト | CSH-ST-M5-12 | 12 | ¥40 | [リンク](https://jp.misumi-ec.com/vona2/detail/221000551286/?HissuCode=CSH-ST-M5-12) |
| 六角穴付ボルト | CSH-ST-M5-15 | 8 | ¥350 | [リンク](https://jp.misumi-ec.com/vona2/detail/221000551286/?HissuCode=CSH-ST-M5-15) |
| 六角穴付ボルト | CSH-ST-M5-20 | 4 | ¥580 | [リンク](https://jp.misumi-ec.com/vona2/detail/221000551286/?HissuCode=CSH-ST-M5-20) |
| 六角穴付ボルト | CSH-ST-M5-32 | 2 | ¥590 | [リンク](https://jp.misumi-ec.com/vona2/detail/221000551286/?HissuCode=CSH-ST-M5-32) |
| ナット | LBNR2.5 | 24 | ¥25 | [リンク](https://jp.misumi-ec.com/vona2/detail/110300250540/?HissuCode=LBNR2.5) |
| ナット | LBNR4 | 16 | ¥80 | [リンク](https://jp.misumi-ec.com/vona2/detail/110300250540/?HissuCode=LBNR4) |
| ナット | LBNR5 | 26 | ¥80 | [リンク](https://jp.misumi-ec.com/vona2/detail/110300250540/?HissuCode=LBNR5) |
| 5シリーズ用ナット | HNTT5-5 | 44 | ¥100 | [リンク](https://jp.misumi-ec.com/vona2/detail/110302246150/?HissuCode=HNTT5-5) |
| 電源アダプタプラグジャック | B0BV8XCTC9 | 2 | ¥950 | [リンク](https://www.amazon.co.jp/dp/B0BV8XCTC9) |
| eSUN 黒フィラメント | ePLA+HS175B1KG-2SPOOL-US | 1 | ¥5,200 | [リンク](https://www.amazon.co.jp/dp/B0D7Q1JYZM) |
| (オプション) eSUN 青フィラメント | ePLA+HS175U1KG-US | 1 | ¥2,800 | [リンク](https://www.amazon.co.jp/dp/B0CQT8VKF7) |

おおよその合計金額（オプション含む）: **¥1,175,000**

おおよその合計金額（オプション除く）: **¥1,030,000** -->

<!-- > [!IMPORTANT]
> 販売店によって価格は変動します．最新の価格は各リンク先でご確認ください．

</details> -->

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- マイルストーン -->
## マイルストーン

- [ ] [ロボットの特徴](#ロボットの特徴) — 仕様表の記入（速度，可搬重量，寸法，重量，センサ，アクチュエータ，電源）
- [ ] [部品リスト（BOM）](#部品リストbom) — 型番・数量・価格・購入先を含む部品リストの作成
- [ ] ロボットの組み立て — 組み立て手順

いずれの節も現在は`TBD`であり，Markdownソース中にドラフトの表がコメントアウトされています．公開前に，記載値を現在の機体構成に対して実測・確認する必要があります．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- 参考文献 -->
## 参考文献

* [RM Motors HW](https://github.com/mjforan/rm_motors_ros)
* [Dynamixel Hardware](https://github.com/dynamixel-community/dynamixel_hardware)
* [ROS Jazzy](https://docs.ros.org/en/jazzy/index.html)
* [ROS2 Control](https://control.ros.org/jazzy/index.html)
* [ROS2 Control Gazebo](https://github.com/ros-controls/gz_ros2_control)

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/TeamSOBITS/sobit_home.svg?style=for-the-badge
[contributors-url]: https://github.com/TeamSOBITS/sobit_home/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/TeamSOBITS/sobit_home.svg?style=for-the-badge
[forks-url]: https://github.com/TeamSOBITS/sobit_home/network/members
[stars-shield]: https://img.shields.io/github/stars/TeamSOBITS/sobit_home.svg?style=for-the-badge
[stars-url]: https://github.com/TeamSOBITS/sobit_home/stargazers
[issues-shield]: https://img.shields.io/github/issues/TeamSOBITS/sobit_home.svg?style=for-the-badge
[issues-url]: https://github.com/TeamSOBITS/sobit_home/issues
[license-shield]: https://img.shields.io/github/license/TeamSOBITS/sobit_home.svg?style=for-the-badge
[license-url]: LICENSE
