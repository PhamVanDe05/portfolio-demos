"""Hand-gesture classification from synthetic 21-point hand landmarks with an sklearn MLP."""
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from sklearn.metrics import ConfusionMatrixDisplay, accuracy_score
from sklearn.model_selection import train_test_split
from sklearn.neural_network import MLPClassifier
from sklearn.preprocessing import StandardScaler
from sklearn.pipeline import make_pipeline

GESTURES = ["fist", "open_palm", "point"]
# Fingers: thumb, index, middle, ring, pinky -> extension (0 curled .. 1 straight) per gesture
EXTENSION = {
    "fist":      [0.2, 0.1, 0.1, 0.1, 0.1],
    "open_palm": [1.0, 1.0, 1.0, 1.0, 1.0],
    "point":     [0.2, 1.0, 0.1, 0.1, 0.1],
}
BASE_ANGLE = np.deg2rad([-60, -20, 0, 20, 40])  # finger direction fan-out from the wrist


def make_hand(gesture, rng):
    """One sample: 21 (x, y) landmarks = wrist + 4 joints per finger, randomly rotated/scaled/noised."""
    pts = [np.zeros(2)]
    for f in range(5):
        ext = np.clip(EXTENSION[gesture][f] + rng.normal(0, 0.3), 0, 1.1)
        ang = BASE_ANGLE[f] + rng.normal(0, 0.05)
        for j in range(1, 5):
            # straight finger grows along direction; curled finger folds back toward the palm
            r = 0.25 * j * (0.3 + 0.7 * ext)
            fold = (1 - ext) * 0.35 * j
            pts.append(np.array([r * np.sin(ang), r * np.cos(ang) - fold * 0.5 + 0.3]))
    pts = np.array(pts)
    th = rng.normal(0, 0.25)  # in-plane rotation
    rot = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    pts = (pts @ rot.T) * rng.uniform(0.8, 1.2) + rng.normal(0, 0.01, pts.shape)
    return (pts - pts[0]).ravel()  # wrist-relative -> translation invariant


def make_dataset(n_per_class=300, seed=0):
    rng = np.random.default_rng(seed)
    X = np.array([make_hand(g, rng) for g in GESTURES for _ in range(n_per_class)])
    y = np.repeat(np.arange(len(GESTURES)), n_per_class)
    return X, y


def train_eval(seed=0):
    X, y = make_dataset(seed=seed)
    Xtr, Xte, ytr, yte = train_test_split(X, y, test_size=0.25, stratify=y, random_state=seed)
    model = make_pipeline(StandardScaler(),
                          MLPClassifier((32, 16), max_iter=300, alpha=1e-2, random_state=seed))
    model.fit(Xtr, ytr)
    return model, Xte, yte, accuracy_score(yte, model.predict(Xte))


if __name__ == "__main__":
    model, Xte, yte, acc = train_eval()
    print(f"test accuracy: {acc:.3f}")
    ConfusionMatrixDisplay.from_predictions(yte, model.predict(Xte), display_labels=GESTURES,
                                            cmap="Blues")
    plt.title(f"Gesture MLP confusion matrix (acc={acc:.3f})")
    plt.tight_layout(); plt.savefig("confusion_matrix.png", dpi=120)
