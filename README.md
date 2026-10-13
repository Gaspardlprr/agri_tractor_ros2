# agri_tractor_ros2 — Motoculteur VTR (ROS 2 Jazzy + Gazebo Harmonic)

Motoculteur à **essieu rigide** (pas de différentiel, pas d'Ackermann) dirigé par
une **force latérale au guidon** qui le fait pivoter autour de l'**éperon de
terrage**. Objectif : algorithme de **Visual Teach & Repeat** transférable au réel.

## Ce qui a changé (v0.2)

| Étape du rapport | Fait |
|---|---|
| 1. Géométrie / physique | URDF réécrit en REP-103 strict : X+ avant. Caméra x=+0.35, essieu x=+0.20, CdM x=0, éperon x=−0.45, guidon x=−1.00. Un seul lien `axle_link` porte les deux roues → vitesse identique garantie mécaniquement. Frictions anisotropes sur les roues (`fdir1`), éperon accrocheur. |
| 2. Contrôleur | `tractor_controller.py` : plus aucun `subprocess`. `/cmd_vel` → `/wheel_cmd` (rad/s) + `/apply_force` (EntityWrench) ponté par `ros_gz_bridge`. Boucle PI sur la vitesse de lacet mesurée. |
| 3. Calibrage | `turning_test` : essai automatique qui mesure rayon de braquage, suivi de lacet, **position réelle du pivot** et force guidon. |
| 4. VTR | `vtr_node.py` : modes `view` (extraction ORB temps réel), `teach` (enregistrement des keyframes) et `repeat` (rejoue le chemin). Monde enrichi (rangées, piquets, arbres, bâtiments, mottes) pour que la caméra ait des points d'intérêt. |

⚠️ Remplace **tout** ton ancien paquet par celui-ci (supprime l'ancien nœud à `subprocess`).

## Installation

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-xacro ros-jazzy-robot-state-publisher \
                 ros-jazzy-teleop-twist-keyboard ros-jazzy-rqt-image-view python3-opencv
cd ~/ros2_ws/src && rm -rf agri_tractor_ros2 && unzip ~/Téléchargements/agri_tractor_ros2.zip
cd ~/ros2_ws && colcon build --symlink-install --packages-select agri_tractor_ros2
source install/setup.bash
colcon test --packages-select agri_tractor_ros2 && colcon test-result --verbose   # 19 tests
```

## Utilisation

**Terminal 1 — simulation**
```bash
ros2 launch agri_tractor_ros2 sim.launch.py          # gui:=false pour aller plus vite
```

**Terminal 2 — conduite clavier** (`i` avancer, `j`/`l` tourner, `k` stop)
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Vérifs rapides**
```bash
ros2 topic echo /handle_force        # force guidon en N
ros2 topic hz /camera/image_raw      # ~30 Hz
```

## Étape 3 — calibrage

```bash
ros2 run agri_tractor_ros2 turning_test --ros-args -p v:=0.5 -p w:=0.4
ros2 run agri_tractor_ros2 turning_test --ros-args -p v:=0.0 -p w:=0.5   # pivot sur place
```
Le rapport affiche le rayon, le % de suivi de lacet et **la position du pivot** :
il doit être proche de x = −0.45 (éperon). Trajectoire en CSV dans `/tmp/`.

| Symptôme | Réglage |
|---|---|
| Tourne trop peu, force saturée à 400 N | baisser `wheel_mu_lat` (URDF) ou monter `max_handle_force` |
| Pivot loin derrière l'éperon (l'éperon dérape) | monter `eperon_mu` ou reculer le CdM (avancer `axle_x`) |
| Oscille en ligne droite | baisser `kp` / `ki` (config/controller.yaml) |
| Tourne trop vite en anticipation | baisser `k_ff` |

Point de départ vérifié par calcul (test `test_default_parameters_pivot_on_eperon`) :
avec 90 kg de châssis + 20 kg d'essieu, μ latéral roues 0.3, μ éperon 3.0, il faut
≈ 290 N au guidon pour faire glisser les roues, et l'éperon tient (demande ≈ 530 N
< adhérence ≈ 815 N). 1500 N comme dans le rapport, c'est 5× ce qu'un opérateur pousse : le tracteur
partirait en toupie, d'où la limite à 400 N.

## Étape 4 — Visual Teach & Repeat

```bash
# Voir les points ORB
ros2 launch agri_tractor_ros2 vtr.launch.py mode:=view
ros2 run rqt_image_view rqt_image_view /vtr/debug_image

# TEACH : lance-le, puis conduis au clavier le long d'un rang ; Ctrl-C pour finir
ros2 launch agri_tractor_ros2 vtr.launch.py mode:=teach path:=~/vtr_paths/rang1

# REPEAT : ramène le tracteur au départ (ou relance la simu), FERME le teleop, puis
ros2 launch agri_tractor_ros2 vtr.launch.py mode:=repeat path:=~/vtr_paths/rang1
```
Principe du repeat : pour chaque image, on cherche la keyframe la plus ressemblante
autour de la distance parcourue, on apparie les points ORB, et le décalage
horizontal médian donne l'erreur de cap → `ω = ω_appris − k·atan(dx/fx)`.
Topics : `/vtr/state`, `/vtr/heading_error`, `/vtr/feature_count`.
Si moins de `min_inliers` appariements pendant 2 s, le tracteur s'arrête.

## Architecture

```
teleop / vtr_node ──/cmd_vel──► tractor_controller ──/wheel_cmd──► JointController (essieu)
                                      │  ▲                       
                         /apply_force │  │ /odom                  
                                      ▼  │                        
                     ros_gz_bridge ──► ApplyLinkWrench (wrench/persistent + clear)
caméra Gazebo ──ros_gz_image──► /camera/image_raw ──► vtr_node
```

## Points à surveiller au premier lancement

Écrit et testé hors simulateur (tests unitaires + nœud contrôleur sur un faux rclpy) ;
la partie Gazebo n'a pas pu être lancée dans mon environnement. Si quelque chose
cloche, regarde d'abord :

1. **Force du guidon sans effet** : vérifie `gz topic -e -t /world/agri_world/wrench/persistent`.
   Si la force s'accumule au lieu d'être remplacée, garde `clear_before_set: true` (défaut).
2. **Roues qui glissent pareil dans tous les sens** : ta version de DART ignore peut-être
   `fdir1`. Repli : mettre `mu1 = mu2 = 0.35` sur `axle_link` et recalibrer.
3. **Le robot tressaute à l'apparition** : monte `-z` du spawn (sim.launch.py) à 0.30.
4. **Pas d'image** : le plugin Sensors exige `ogre2` et un GPU/driver OpenGL fonctionnel.

## Régénérer le monde
```bash
python3 worlds/generate_world.py 42     # autre graine = autre disposition
```
