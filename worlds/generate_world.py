#!/usr/bin/env python3
"""Génère agri_world.sdf (champ agricole riche en repères visuels pour le VTR).

Usage : python3 generate_world.py [graine]   -> écrit agri_world.sdf à côté du script.
Repère monde : X+ = direction des rangées ; le tracteur apparaît en (0, 0), cap X+.
Les objets sont regroupés (un modèle statique = plusieurs visuels) pour un chargement rapide.
"""
import os
import random
import sys

SEED = int(sys.argv[1]) if len(sys.argv) > 1 else 67
rng = random.Random(SEED)


def mat(rgb):
    c = ' '.join(f'{v:.3f}' for v in rgb)
    return f'<material><ambient>{c} 1</ambient><diffuse>{c} 1</diffuse></material>'


def visual(name, pose, geom, rgb):
    return (f'<visual name="{name}"><pose>{pose}</pose>'
            f'<geometry>{geom}</geometry>{mat(rgb)}</visual>')


def collision(name, pose, geom):
    return f'<collision name="{name}"><pose>{pose}</pose><geometry>{geom}</geometry></collision>'


def model(name, parts):
    body = '\n        '.join(parts)
    return (f'    <model name="{name}"><static>true</static>\n'
            f'      <link name="link">\n        {body}\n      </link>\n    </model>')


models = []

# --- Rangées de cultures (couloirs libres à y = 0 et y = +-1.6) ---
for r, row_y in enumerate((-0.8, 0.8, -2.4, 2.4, -4.0, 4.0)):
    parts, x, k = [], 2.0, 0
    while x < 38:
        g = rng.uniform(0.35, 0.75)
        rad = rng.uniform(0.10, 0.18)
        pose = f'{x:.3f} {row_y + rng.uniform(-0.05, 0.05):.3f} {0.12 + rng.uniform(0, 0.08):.3f} 0 0 0'
        parts.append(visual(f'p{k}', pose, f'<sphere><radius>{rad:.3f}</radius></sphere>',
                            (0.15 * g, g, 0.1 * g)))
        x += rng.uniform(0.35, 0.6)
        k += 1
    models.append(model(f'crop_row_{r}', parts))

# --- Piquets colorés le long du champ ---
parts = []
for i in range(16):
    x = 2.5 + i * 2.3
    for side in (-1, 1):
        h = rng.uniform(0.8, 1.6)
        rgb = rng.choice([(0.9, 0.1, 0.1), (0.95, 0.85, 0.1), (1, 1, 1), (0.1, 0.3, 0.9)])
        geom = f'<cylinder><radius>0.04</radius><length>{h:.2f}</length></cylinder>'
        pose = f'{x:.2f} {side * 5.3:.2f} {h / 2:.3f} 0 0 0'
        parts.append(collision(f'sc{i}_{side}', pose, geom))
        parts.append(visual(f's{i}_{side}', pose, geom, rgb))
models.append(model('stakes', parts))

# --- Arbres en bordure ---
parts = []
for i in range(14):
    x = rng.uniform(-5, 45)
    y = rng.choice([-1, 1]) * rng.uniform(8, 14)
    trunk = '<cylinder><radius>0.15</radius><length>2.0</length></cylinder>'
    parts.append(collision(f'tc{i}', f'{x:.2f} {y:.2f} 1.0 0 0 0', trunk))
    parts.append(visual(f't{i}', f'{x:.2f} {y:.2f} 1.0 0 0 0', trunk, (0.35, 0.22, 0.1)))
    parts.append(visual(f'c{i}', f'{x:.2f} {y:.2f} 2.6 0 0 0',
                        f'<sphere><radius>{rng.uniform(0.8, 1.3):.2f}</radius></sphere>',
                        (0.1, rng.uniform(0.3, 0.5), 0.1)))
models.append(model('trees', parts))

# --- Bâtiments / repères lointains ---
parts = []
for name, pose, size, rgb in [
        ('barn', '48 -6 2 0 0 0', '6 8 4', (0.6, 0.15, 0.1)),
        ('shed', '-8 7 1.2 0 0 0.4', '3 4 2.4', (0.7, 0.7, 0.65)),
        ('silo', '44 9 3 0 0 0', '2.5 2.5 6', (0.75, 0.75, 0.8)),
        ('fence_n', '20 16 0.5 0 0 0', '50 0.1 1.0', (0.55, 0.4, 0.25)),
        ('fence_s', '20 -16 0.5 0 0 0', '50 0.1 1.0', (0.55, 0.4, 0.25))]:
    geom = f'<box><size>{size}</size></box>'
    parts.append(collision(name + '_c', pose, geom))
    parts.append(visual(name, pose, geom, rgb))
models.append(model('buildings', parts))

# --- Mottes de terre sombres (texture au sol vue par la caméra inclinée) ---
parts = []
for i in range(200):
    x = rng.uniform(-2, 40)
    y = rng.uniform(-1.5, 1.5) * rng.choice([1, 1, 2.5])
    s = rng.uniform(0.06, 0.2)
    d = rng.uniform(0.18, 0.32)
    parts.append(visual(f'm{i}', f'{x:.2f} {y:.2f} 0.003 0 0 {rng.uniform(0, 3.14):.2f}',
                        f'<box><size>{s:.2f} {s * rng.uniform(0.5, 1.5):.2f} 0.006</size></box>',
                        (d, d * 0.7, d * 0.45)))
models.append(model('soil_clods', parts))

MODELS = '\n\n'.join(models)

WORLD = f'''<?xml version="1.0" ?>
<!--
  agri_world.sdf : champ agricole pour le motoculteur VTR (Gazebo Harmonic).
  FICHIER GÉNÉRÉ par generate_world.py (graine {SEED}) : modifier le script, pas ce fichier.
  Repère monde : X+ = direction des rangées. Le tracteur apparaît en (0,0), cap X+.
-->
<sdf version="1.9">
  <world name="agri_world">

    <physics name="1ms" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <gravity>0 0 -9.81</gravity>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <!-- Forces externes : /world/agri_world/wrench, /wrench/persistent, /wrench/clear -->
    <plugin filename="gz-sim-apply-link-wrench-system" name="gz::sim::systems::ApplyLinkWrench"/>

    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.62 0.78 0.95 1</background>
      <shadows>true</shadows>
      <grid>false</grid>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 30 0 0 0</pose>
      <diffuse>0.9 0.9 0.85 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.2 -0.9</direction>
    </light>

    <!-- Sol : friction élevée. DART retient le MIN des deux surfaces en contact,
         donc ce sont les coefficients des roues / de l'éperon (URDF) qui comptent. -->
    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <surface>
            <friction><ode><mu>5.0</mu><mu2>5.0</mu2></ode></friction>
            <contact><ode><kp>1e6</kp><kd>100</kd></ode></contact>
          </surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          {mat((0.42, 0.30, 0.18))}
        </visual>
      </link>
    </model>

{MODELS}

  </world>
</sdf>
'''

if __name__ == '__main__':
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'agri_world.sdf')
    with open(out, 'w', encoding='utf-8') as f:
        f.write(WORLD)
    print(f'{out} écrit ({len(models)} modèles, {len(WORLD) // 1024} Ko)')
