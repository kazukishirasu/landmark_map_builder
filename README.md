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
#### [landmark_map_builder/param/class.yaml](https://github.com/kazukishirasu/landmark_map_builder/blob/master/param/class.yaml)
```yaml
# 保存するランドマークのクラス名
class:
  [Door, Elevator, Vending machine, Fire extinguisher]
```
#### [landmark_map_builder/launch/record_landmark.launch](https://github.com/kazukishirasu/landmark_map_builder/blob/master/launch/record_landmark.launch)
```xml
<launch>
<arg name="landmark_class_file"     default="$(find landmark_map_builder)/param/class.yaml"/>
<!-- ランドマークファイルの保存先 -->
<arg name="landmark_file"           default="$(find landmark_map_builder)/landmark/landmark_ver0.yaml"/>
<arg name="image_width"             default="1280"/>
<arg name="prob_threshold"          default="0.9"/>
<arg name="min_obj_size"            default="20"/>
<arg name="cutoff_min_angle"        default="-2.5"/>
<arg name="cutoff_max_angle"        default="2.5"/>

<node pkg="landmark_map_builder" type="calc_landmark_node" name="calc_landmark_node" output="screen">
    <param name="landmark_class_file"   value="$(arg landmark_class_file)"/>
    <param name="image_width"           value="$(arg image_width)"/>
    <param name="prob_threshold"        value="$(arg prob_threshold)"/>
    <param name="min_obj_size"          value="$(arg min_obj_size)"/>
    <param name="cutoff_min_angle"      value="$(arg cutoff_min_angle)"/>
    <param name="cutoff_max_angle"      value="$(arg cutoff_max_angle)"/>
    <remap from="/scan"                 to="/rfans/surestar_scan"/>
</node>

<node pkg="landmark_map_builder" type="visualize_landmark_node" name="visualize_landmark_node" output="screen">
    <param name="landmark_file"         value="$(arg landmark_file)"/>
</node>
</launch>
```
```bash
# 実行
roslaunch landmark_map_builder record_landmark.launch
# 保存
rosservice call /save_landmark
```
### ランドマークデータのクラスタリング
DBSCANを使用
#### [landmark_map_builder/build/run.sh](https://github.com/kazukishirasu/landmark_map_builder/blob/master/build/run.sh)
```bash
# 第二引数：DBSCANに用いるパラメータファイル
# 第三引数：クラスタリングするランドマークファイル
# 第四引数：クラスタリング後のランドマークファイル
./build/dbscan param/dbscan.yaml landmark/landmark_ver1.yaml map/map_ver1.yaml
```
```bash
# 実行
roscd landmark_map_builder
./build/run.sh
```
### ランドマークの表示、編集
rviz上でランドマークの移動、追加、削除が可能
#### [landmark_map_builder/launch/edit_landmark.launch](https://github.com/kazukishirasu/landmark_map_builder/blob/master/launch/edit_landmark.launch)
```xml
<launch>
<!-- ランドマークファイル -->
<arg name="landmark_file" default="$(find landmark_map_builder)/map/map_ver1.yaml"/>

<node pkg="landmark_map_builder" type="edit_landmark_node" name="edit_landmark_node" output="screen">
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
