# Интеграция pallet-докинга

## 1. `nav2_params.yaml` — добавить в `docking_server`

```yaml
docking_server:
  ros__parameters:
    ...
    dock_backwards: true          # виллы сзади base_link -> заезд задним ходом
    dock_plugins: ['simple_charging_dock', 'pallet_dock']
    pallet_dock:
      plugin: 'opennav_docking::SimpleChargingDock'
      docking_threshold: 0.05
      staging_x_offset: -1.0
      use_external_detection_pose: true
      use_battery_status: false
      use_stall_detection: false
      external_detection_timeout: 1.0
      external_detection_translation_x: 0.0
      external_detection_translation_y: 0.0
      external_detection_rotation_roll: 0.0
      external_detection_rotation_pitch: 0.0
      external_detection_rotation_yaw: 0.0
      filter_coef: 0.1
```

`SimpleChargingDock` в вашей версии Nav2 подписывается на топик обнаружения дока
по параметру, специфичному для установленного дистрибутива (проверьте
`external_detection_pose_topic` / поле в actionlib goal `dock_pose`) — приведите
имя топика к `/pallet_detector/dock_pose`, который публикует
`pallet_detector_node`. Если версия `SimpleChargingDock` в вашем дистрибутиве не
поддерживает произвольный dock без предопределённого instance в `docks:` —
понадобится написать тонкий `PalletDock` C++ плагин (наследник
`opennav_docking_core::ChargingDock`), структурно идентичный
`SimpleChargingDock`, но без логики зарядки. Дайте знать — сделаю следующим шагом.

## 2. `bt_navigator` — зарегистрировать кастомные плагины

```yaml
bt_navigator:
  ros__parameters:
    ...
    plugin_lib_names:
      - avf_15_pallet_bt_plugins
    default_nav_to_pose_bt_xml: ".../trees/pick_pallet.xml"   # или отдельный action-сервер под задачу
```

Либо грузите `pick_pallet.xml` не как основной `NavigateToPose` дефолт, а через
отдельный `BehaviorTreeEngine`/собственный task-узел, который явно триггерит
это дерево по команде "забрать паллету №N" (staging_pose/dropoff_pose/pallet id
на blackboard).

## 3. `controllers.yaml` — уже готово

`fork_controller` (`position_controllers/JointGroupPositionController`) уже
публикует состояние в `/joint_states` и принимает команды в
`/fork_controller/commands` (`std_msgs/Float64MultiArray`, порядок
`[J_main_villas, J_second_villas]`) — именно это использует
`SetForkPositionAction`.

## 4. `display_launch.py` — добавить узел детектора паллеты

```python
pallet_detector_node = Node(
    package='avf_15_pallet_perception',
    executable='pallet_detector_node',
    name='pallet_detector_node',
    output='screen',
    parameters=[{'use_sim_time': True}],
)
```
Добавить в список `LaunchDescription([...])` рядом с `robot_localization_node`.

## 5. Проверить в SDF

`front_camera_joint` смотрит вперёд под углом `0.261799` рад вниз — это ок для
обзора пола/паллеты перед роботом. Убедитесь, что `roi_x_min/roi_x_max` в
параметрах `pallet_detector_node` соответствуют реальной зоне видимости камеры
(near/far clip `0.05`/`3` м уже заданы в SDF — `roi_x_max` не должен превышать
~2.8 м).

## 6. Единицы/ориентация вилл

`fork_controller.joints: [J_main_villas, J_second_villas]` — обе `prismatic`.
Значения в дереве (`main_villas_pos`, `second_villas_pos`) заданы в метрах
согласно `<limit><lower>-0.1</lower><upper>1.0</upper>` из SDF — подставьте
реальные высоты подъёма/опускания под вашу геометрию паллеты.
