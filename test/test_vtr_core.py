import math

import cv2
import numpy as np
import pytest

from agri_tractor_ros2.vtr_core import (FeatureExtractor, Localizer, PathRecorder,
                                        focal_from_hfov, heading_error, horizontal_offset,
                                        image_to_gray, load_path, match, steering_command)

W, H = 640, 480


def scene(seed=1, size=(H, W + 200)):
    """Grande image texturée (formes aléatoires) dans laquelle on découpe des vues."""
    rng = np.random.default_rng(seed)
    img = np.full(size, 110, np.uint8)
    for _ in range(400):
        c = (int(rng.integers(0, size[1])), int(rng.integers(0, size[0])))
        col = int(rng.integers(0, 255))
        if rng.random() < 0.5:
            cv2.circle(img, c, int(rng.integers(4, 25)), col, -1)
        else:
            cv2.rectangle(img, c, (c[0] + int(rng.integers(5, 40)), c[1] + int(rng.integers(5, 40))),
                          col, -1)
    return cv2.GaussianBlur(img, (3, 3), 0)


def view(big, x0):
    return big[:, x0:x0 + W].copy()


def test_image_conversion_rgb8_with_padding():
    rgb = np.zeros((4, 3, 3), np.uint8)
    rgb[..., 0] = 255                       # rouge pur
    step = 3 * 3 + 2                        # 2 octets de padding par ligne
    buf = np.zeros((4, step), np.uint8)
    buf[:, :9] = rgb.reshape(4, 9)
    g = image_to_gray(buf.tobytes(), 3, 4, step, 'rgb8')
    assert g.shape == (4, 3)
    assert int(g[0, 0]) == 76               # 0.299 * 255
    with pytest.raises(ValueError):
        image_to_gray(buf.tobytes(), 3, 4, step, '16UC1')


def test_features_are_spread():
    ext = FeatureExtractor(600)
    kps, desc = ext.extract(view(scene(), 100))
    assert len(kps) > 300
    assert desc.shape[1] == 32
    xs = [k.pt[0] for k in kps]
    assert min(xs) < W * 0.25 and max(xs) > W * 0.75   # couvre toute la largeur


@pytest.mark.parametrize('shift', [-30, -8, 0, 12, 40])
def test_horizontal_offset_recovers_shift(shift):
    big = scene(3)
    ext = FeatureExtractor(800)
    ref = view(big, 100)
    live = view(big, 100 - shift)           # scène décalée de +shift px dans l'image
    kr, dr = ext.extract(ref)
    kl, dl = ext.extract(live)
    m = match(dl, dr)
    dx, n = horizontal_offset(kl, kr, m)
    assert n > 50
    assert dx == pytest.approx(shift, abs=1.0)


def test_unrelated_images_few_matches():
    ext = FeatureExtractor(800)
    kr, dr = ext.extract(view(scene(5), 50))
    kl, dl = ext.extract(view(scene(9), 50))
    _, n = horizontal_offset(kl, kr, match(dl, dr))
    assert n < 25                            # < min_inliers par défaut


def test_steering_sign():
    fx = focal_from_hfov(W, 1.2)
    # scène vue décalée à droite -> robot tourné à gauche -> il faut tourner à droite
    assert heading_error(20, fx) > 0
    assert steering_command(20, fx, 0.0, 1.5, 0.6) < 0
    assert steering_command(-20, fx, 0.0, 1.5, 0.6) > 0
    assert steering_command(5000, fx, 0.0, 1.5, 0.6) == -0.6     # saturation
    assert math.degrees(heading_error(fx * math.tan(0.1), fx)) == pytest.approx(math.degrees(0.1))


def test_teach_then_repeat_localization(tmp_path):
    """Simule un TEACH (la caméra glisse le long d'une scène) puis un REPEAT
    avec un décalage de cap : le localiseur doit suivre l'ordre des keyframes
    et retrouver le décalage."""
    big = scene(11, size=(H, W + 1200))
    ext = FeatureExtractor(800)
    rec = PathRecorder(tmp_path / 'p', min_dist=0.3)
    for i in range(12):                      # 12 keyframes, 0.3 m d'écart, 80 px entre vues
        rec.update_odometry(i * 0.3, 0.0)
        assert rec.should_record(0.0)
        rec.add(view(big, 80 * i), i * 0.3, 0.0, 0.0, 0.0, 0.4)
    rec.update_odometry(3.31, 0.0)
    assert not rec.should_record(0.0)   # 0.01 m depuis la dernière keyframe

    d, kfs = load_path(tmp_path / 'p')
    assert len(kfs) == 12 and kfs[-1].s == pytest.approx(3.3)
    loc = Localizer(d, kfs, ext, window=3)
    visited = []
    for i in range(12):
        live = view(big, 80 * i + 15)       # robot légèrement tourné : scène à -15 px
        kl, dl = ext.extract(live)
        kf, dx, n = loc.localize(kl, dl, s_travelled=i * 0.3)
        assert kf is not None and n >= 25
        visited.append(kf.index)
        assert dx == pytest.approx(-15, abs=2.0)
    assert visited == sorted(visited)        # jamais de retour en arrière
    assert visited[-1] == 11
    assert loc.finished(3.3)
