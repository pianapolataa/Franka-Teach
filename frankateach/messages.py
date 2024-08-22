from dataclasses import dataclass
import numpy as np


@dataclass
class FrankaState:
    pos: np.ndarray
    quat: np.ndarray
    gripper: np.ndarray
    timestamp: float


@dataclass
class FrankaAction:
    pos: np.ndarray
    quat: np.ndarray
    gripper: np.ndarray
    reset: bool
    timestamp: float
