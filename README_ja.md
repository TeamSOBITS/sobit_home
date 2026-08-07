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

TBD

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
