# alterego_navigation

Pacchetto ROS 2 per AlterEGO che abilita:

- mappatura teleguidata (utente guida il robot mentre SLAM costruisce la mappa)
- mappatura autonoma (frontier exploration con Nav2)
- navigazione autonoma su mappa salvata
- missioni waypoint autonome

## Prerequisiti

- ROS 2 Jazzy (o distribuzione compatibile con Nav2 usata nel progetto)
- pacchetti installati: `nav2_bringup`, `slam_toolbox`, `nav2_map_server`
- topic laser disponibile su `/<robot_name>/scan`
- TF coerente: `map -> odom -> base_link`
- topic comando velocita' raggiungibile da Nav2 (`cmd_vel` nel namespace del robot)

## Build

Dal root workspace:

```bash
colcon build --packages-select alterego_navigation
source install/setup.bash
```

## 1) Mappatura Teleguidata

Avvia Nav2 + SLAM e guida il robot con il tuo teleop/pilot:

```bash
ros2 launch alterego_navigation mapping_teleop.launch.py namespace:=alterego5 use_sim_time:=false
```

Salva la mappa quando l'ambiente e' stato coperto:

```bash
ros2 run nav2_map_server map_saver_cli -f /tmp/alterego_map
```

Questo comando genera `/tmp/alterego_map.yaml` e `/tmp/alterego_map.pgm`.

## 2) Mappatura Autonoma

Avvia Nav2 + SLAM + esploratore frontiere:

```bash
ros2 launch alterego_navigation mapping_autonomous.launch.py namespace:=alterego5 use_sim_time:=false
```

L'esploratore invia automaticamente goal verso frontiere sconosciute.
Salva poi la mappa con `map_saver_cli` come sopra.

## 3) Navigazione Autonoma Su Mappa Salvata

```bash
ros2 launch alterego_navigation navigation.launch.py namespace:=alterego5 map:=/tmp/alterego_map.yaml use_sim_time:=false
```

A questo punto puoi inviare goal da RViz (`Nav2 Goal`) o da nodi esterni.

## 4) Missioni Waypoint

Esempio con file waypoint YAML:

```bash
ros2 launch alterego_navigation waypoint_mission.launch.py namespace:=alterego5 waypoints_file:=/tmp/my_waypoints.yaml
```

Formato YAML:

```yaml
waypoints:
  - x: 0.5
    y: 0.0
    yaw: 0.0
  - x: 1.5
    y: -0.5
    yaw: 1.57
```

## Integrazione con segway / base mobile AlterEGO

Se il tuo controller base non usa direttamente `cmd_vel`, aggiungi un bridge (topic remap o nodo adattatore)
tra `cmd_vel` di Nav2 e il topic del segway.
Nel repository esiste gia' `CMD_VEL_IN_topic: cmd_vel` nel pilot, quindi l'integrazione tipica e' diretta
se Nav2 gira nello stesso namespace robot.
