"""Cœur algorithmique du Visual Teach & Repeat, sans dépendance ROS.

Principe (VTR « à la Furgale & Barfoot », version monoculaire simplifiée) :
  TEACH  : pendant la conduite manuelle on enregistre une suite d'images-clés
           (keyframes), une tous les ~0.3 m, avec la pose odométrique et la
           vitesse de lacet du moment.
  REPEAT : on avance ; pour l'image courante on cherche la keyframe la plus
           ressemblante (fenêtre autour de la distance parcourue), on apparie
           les points ORB, et le DÉCALAGE HORIZONTAL médian des appariements
           donne l'erreur de cap : si la scène apparaît décalée vers la droite,
           le robot est tourné vers la gauche par rapport au chemin appris.
           ω = ω_appris - k * atan(dx / fx)
"""
import json
import math
import os
from dataclasses import asdict, dataclass

import cv2
import numpy as np


# ------------------------------------------------------------------ images

def image_to_gray(data, width, height, step, encoding):
    """Convertit le buffer d'un sensor_msgs/Image en niveaux de gris (sans cv_bridge)."""
    buf = np.frombuffer(bytes(data) if not isinstance(data, (bytes, bytearray)) else data,
                        dtype=np.uint8)
    enc = encoding.lower()
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1, '8uc1': 1,
                '8uc3': 3}.get(enc)
    if channels is None:
        raise ValueError(f'encodage non géré : {encoding}')
    img = buf.reshape(height, step)[:, :width * channels].reshape(height, width, channels)
    if channels == 1:
        return img[:, :, 0].copy()
    code = {'rgb8': cv2.COLOR_RGB2GRAY, 'bgr8': cv2.COLOR_BGR2GRAY, '8uc3': cv2.COLOR_BGR2GRAY,
            'rgba8': cv2.COLOR_RGBA2GRAY, 'bgra8': cv2.COLOR_BGRA2GRAY}[enc]
    return cv2.cvtColor(img, code)


def focal_from_hfov(width, hfov):
    return (width / 2.0) / math.tan(hfov / 2.0)


# ------------------------------------------------------------------ features

class FeatureExtractor:
    """ORB réparti sur une grille pour éviter que tous les points tombent sur
    un seul objet très texturé (un piquet, un arbre)."""

    def __init__(self, n_features=800, grid=(4, 3), fast_threshold=12):
        self.n = n_features
        self.grid = grid
        self.orb = cv2.ORB_create(nfeatures=n_features * 2, scaleFactor=1.2, nlevels=8,
                                  edgeThreshold=15, patchSize=31, fastThreshold=fast_threshold)
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

    def extract(self, gray):
        g = self.clahe.apply(gray)
        kps = self.orb.detect(g, None)
        kps = self._bucket(kps, g.shape)
        kps, desc = self.orb.compute(g, kps)
        if desc is None:
            return [], np.zeros((0, 32), np.uint8)
        return kps, desc

    def _bucket(self, kps, shape):
        h, w = shape[:2]
        gx, gy = self.grid
        per_cell = max(1, self.n // (gx * gy))
        cells = {}
        for k in kps:
            c = (min(gx - 1, int(k.pt[0] * gx / w)), min(gy - 1, int(k.pt[1] * gy / h)))
            cells.setdefault(c, []).append(k)
        out = []
        for lst in cells.values():
            lst.sort(key=lambda k: -k.response)
            out.extend(lst[:per_cell])
        return out


def match(desc_live, desc_ref, ratio=0.8):
    """Appariement Hamming + test du ratio de Lowe + vérification croisée."""
    if len(desc_live) < 2 or len(desc_ref) < 2:
        return []
    bf = cv2.BFMatcher(cv2.NORM_HAMMING)
    fwd = bf.knnMatch(desc_live, desc_ref, k=2)
    good = {}
    for pair in fwd:
        if len(pair) == 2 and pair[0].distance < ratio * pair[1].distance:
            good[pair[0].queryIdx] = pair[0]
    bwd = bf.match(desc_ref, desc_live)
    back = {m.queryIdx: m.trainIdx for m in bwd}
    return [m for q, m in good.items() if back.get(m.trainIdx) == q]


def horizontal_offset(kps_live, kps_ref, matches, max_dy=40.0):
    """Décalage horizontal robuste (px) : médiane après rejet par MAD.

    Retourne (dx, nb_inliers). dx > 0 => la scène est plus à droite dans l'image
    courante que dans la keyframe => le robot pointe trop à GAUCHE.
    """
    if not matches:
        return None, 0
    d = np.array([[kps_live[m.queryIdx].pt[0] - kps_ref[m.trainIdx].pt[0],
                   kps_live[m.queryIdx].pt[1] - kps_ref[m.trainIdx].pt[1]] for m in matches])
    d = d[np.abs(d[:, 1]) < max_dy]   # le VTR suppose peu de mouvement vertical
    if len(d) == 0:
        return None, 0
    med = np.median(d[:, 0])
    mad = np.median(np.abs(d[:, 0] - med)) + 1e-6
    inl = d[np.abs(d[:, 0] - med) < 3.0 * 1.4826 * mad + 2.0]
    if len(inl) == 0:
        return None, 0
    return float(np.median(inl[:, 0])), int(len(inl))


def heading_error(dx, fx):
    """Erreur de cap (rad) correspondant à un décalage horizontal dx (px).
    Positif = le robot est tourné à gauche du chemin appris."""
    return math.atan2(dx, fx)


def steering_command(dx, fx, w_taught, k_heading, w_max):
    w = w_taught - k_heading * heading_error(dx, fx)
    return max(-w_max, min(w_max, w))


# ------------------------------------------------------------------ chemin

@dataclass
class Keyframe:
    index: int
    image: str        # nom de fichier PNG
    s: float          # abscisse curviligne (m)
    x: float
    y: float
    yaw: float
    w: float          # vitesse de lacet pendant l'apprentissage (rad/s)
    v: float          # vitesse d'avance pendant l'apprentissage (m/s)


class PathRecorder:
    """Enregistre les keyframes sur disque ; l'index JSON est réécrit à chaque
    ajout pour qu'un Ctrl-C ne perde rien."""

    def __init__(self, directory, min_dist=0.3, min_yaw=0.17):
        self.dir = os.path.expanduser(directory)
        os.makedirs(self.dir, exist_ok=True)
        self.min_dist, self.min_yaw = min_dist, min_yaw
        self.keyframes = []
        self.s = 0.0
        self._last_pose = None

    def update_odometry(self, x, y):
        if self._last_pose is not None:
            self.s += math.hypot(x - self._last_pose[0], y - self._last_pose[1])
        self._last_pose = (x, y)

    def should_record(self, yaw):
        if not self.keyframes:
            return True
        last = self.keyframes[-1]
        dyaw = abs(math.atan2(math.sin(yaw - last.yaw), math.cos(yaw - last.yaw)))
        return self.s - last.s >= self.min_dist - 1e-6 or dyaw >= self.min_yaw

    def add(self, gray, x, y, yaw, w, v):
        i = len(self.keyframes)
        name = f'kf_{i:05d}.png'
        cv2.imwrite(os.path.join(self.dir, name), gray)
        kf = Keyframe(i, name, self.s, x, y, yaw, w, v)
        self.keyframes.append(kf)
        self._write_index()
        return kf

    def _write_index(self):
        tmp = os.path.join(self.dir, 'path.json.tmp')
        with open(tmp, 'w', encoding='utf-8') as f:
            json.dump({'version': 1, 'keyframes': [asdict(k) for k in self.keyframes]}, f, indent=1)
        os.replace(tmp, os.path.join(self.dir, 'path.json'))


def load_path(directory):
    d = os.path.expanduser(directory)
    with open(os.path.join(d, 'path.json'), encoding='utf-8') as f:
        data = json.load(f)
    kfs = [Keyframe(**k) for k in data['keyframes']]
    if not kfs:
        raise ValueError(f'chemin vide : {d}')
    return d, kfs


class Localizer:
    """Choisit la keyframe de référence pendant le REPEAT.

    Candidates : keyframes dont l'abscisse s est proche de la distance parcourue
    (odométrie), dans une fenêtre [idx-1, idx+window]. On garde celle qui a le
    plus d'appariements inliers. L'index ne recule jamais de plus d'un cran.
    """

    def __init__(self, directory, keyframes, extractor, window=3):
        self.dir, self.kfs, self.ext, self.window = directory, keyframes, extractor, window
        self.idx = 0
        self._cache = {}

    def features(self, i):
        if i not in self._cache:
            g = cv2.imread(os.path.join(self.dir, self.kfs[i].image), cv2.IMREAD_GRAYSCALE)
            if g is None:
                raise FileNotFoundError(self.kfs[i].image)
            self._cache[i] = self.ext.extract(g)
            if len(self._cache) > 4 * self.window + 8:   # petite mémoire glissante
                self._cache.pop(min(self._cache))
        return self._cache[i]

    def index_for_distance(self, s):
        i = self.idx
        while i + 1 < len(self.kfs) and self.kfs[i + 1].s <= s:
            i += 1
        return i

    def localize(self, kps, desc, s_travelled):
        """Retourne (keyframe, dx, nb_inliers) ou (None, None, 0)."""
        center = max(self.idx, self.index_for_distance(s_travelled))
        lo = max(0, self.idx - 1)
        hi = min(len(self.kfs) - 1, center + self.window)
        best_i, best_dx, best_n, best_score = None, None, 0, 0.0
        for i in range(lo, hi + 1):
            kr, dr = self.features(i)
            dx, n = horizontal_offset(kps, kr, match(desc, dr))
            if dx is None:
                continue
            # léger bonus à la keyframe attendue par l'odométrie
            score = n * (1.15 if i == center else 1.0)
            if score > best_score:
                best_i, best_dx, best_n, best_score = i, dx, n, score
        if best_i is None:
            return None, None, 0
        self.idx = max(self.idx, best_i)
        return self.kfs[best_i], best_dx, best_n

    def finished(self, s_travelled):
        return self.idx >= len(self.kfs) - 1 and s_travelled >= self.kfs[-1].s - 0.05


def draw_debug(gray, kps, text_lines, matches_ref=None):
    img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    img = cv2.drawKeypoints(img, kps, None, color=(0, 255, 0))
    if matches_ref is not None:
        kps_ref, matches = matches_ref
        for m in matches:
            p = tuple(int(v) for v in kps[m.queryIdx].pt)
            q = tuple(int(v) for v in kps_ref[m.trainIdx].pt)
            cv2.line(img, q, p, (0, 140, 255), 1)
    for i, t in enumerate(text_lines):
        cv2.putText(img, t, (8, 22 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
        cv2.putText(img, t, (8, 22 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    return img
