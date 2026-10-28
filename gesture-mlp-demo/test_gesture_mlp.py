from gesture_mlp import make_dataset, train_eval


def test_dataset_shape():
    X, y = make_dataset(n_per_class=10)
    assert X.shape == (30, 42) and set(y) == {0, 1, 2}   # 21 points x (x, y)


def test_accuracy_above_90_percent():
    assert train_eval()[3] > 0.9
