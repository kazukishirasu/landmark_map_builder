# landmark_map_builder
[emcl_with_landmark](https://github.com/kazukishirasu/emcl_with_landmark)で使用するランドマーク地図生成用パッケージ
## インストール
```bash
git clone https://github.com/kazukishirasu/landmark_map_builder.git
catkin build landmark_map_builder
source ~/.bashrc
```
## 使い方
### ランドマークのデータ収集
#### [landmark_map_builder/param/name.yaml](https://github.com/kazukishirasu/landmark_map_builder/blob/master/param/name.yaml)
```yaml
name:
  # 保存したいランドマークのクラス名
  [Door, Elevator, Vending machine]
```
#### [landmark_map_builder/launch/record_landmark.launch](https://github.com/kazukishirasu/landmark_map_builder/blob/master/launch/record_landmark.launch)
```xml
<launch>
<arg name="landmark_name_file"   default="$(find landmark_map_builder)/param/name.yaml"/>
<!-- ランドマーク情報を保存するファイル -->
<arg name="landmark_record_file" default="$(find landmark_map_builder)/landmark/landmark_ver1.yaml"/>

<node pkg="landmark_map_builder" type="record_landmark_node" name="record_landmark_node" output="screen">
    <param name="landmark_name_file" value="$(arg landmark_name_file)"/>
    <param name="landmark_record_file" value="$(arg landmark_record_file)"/>
</node>
</launch>
```
```bash
# 実行
roslaunch landmark_map_builder record_landmark.launch
# 保存
rosservice call /save_landmark
```
### 収集したデータのクラスタリング
DBSCANを使用
#### [landmark_map_builder/build/run.sh](https://github.com/kazukishirasu/landmark_map_builder/blob/master/build/run.sh)
```bash
# 第二引数：DBSCANに用いるパラメータファイル
# 第三引数：収集したランドマークファイル
# 第四引数：クラスタリング後のランドマークファイル
./build/dbscan param/dbscan.yaml landmark/landmark_ver1.yaml map/map_ver1.yaml
```
実行
```bash
roscd landmark_map_builder
./build/run.sh
```
### ランドマークの表示、編集
rviz上でランドマークの移動、追加、削除が可能
#### [landmark_map_builder/launch/visualize_landmark.launch](https://github.com/kazukishirasu/landmark_map_builder/blob/master/launch/visualize_landmark.launch)
```xml
<launch>
<!-- ランドマークファイル -->
<arg name="landmark_file" default="$(find landmark_map_builder)/map/map_ver1.yaml"/>

<node pkg="landmark_map_builder" type="visualize_landmark_node" name="visualize_landmark_node" output="screen">
    <param name="landmark_file" value="$(arg landmark_file)"/>
</node>
</launch>
```
```bash
# 実行
roslaunch landmark_map_builder visualize_landmark.launch
# 保存
rosservice call /save_landmark
```
