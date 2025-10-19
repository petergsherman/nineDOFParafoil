#nineDOF_Visualization.py
import matplotlib.pyplot as plt
import numpy as np

def plot_3D_trajectory(data, title = "Parafoil 3D Trajectory"):
    xs = [s[0] for s in data]
    ys = [s[1] for s in data]
    zs = [s[2] for s in data]