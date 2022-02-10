import numpy as np


def tf_from_dh(
    alpha: float, a: float, d: float, theta: float
) -> np.ndarray:  # type: ignore
    """
    Converts DH parameters to a 3D homogenous transformation matrix.
    """

    return np.array(  # type: ignore
        [
            [np.cos(theta), -np.sin(theta), 0, a],
            [
                np.sin(theta) * np.cos(alpha),
                np.cos(theta) * np.cos(alpha),
                -np.sin(alpha),
                -np.sin(alpha) * d,
            ],
            [
                np.sin(theta) * np.sin(alpha),
                np.cos(theta) * np.sin(alpha),
                np.cos(alpha),
                np.cos(alpha) * d,
            ],
            [0, 0, 0, 1],
        ]
    )
