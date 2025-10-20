# nineDOF_Visualization.py

import numpy as np
import matplotlib.pyplot as plt
from nineDOF_Transform import makeT_IP

def plot_trajectory_3D(
    data,
    title="Parafoil 3D Trajectory",
    zrp=1.5,
    body_z_is_down=True,
    show_cp=True,
):
    """
    Plot CM and (optionally) canopy CP trajectories for the 9-DOF simulation.

    Parameters
    ----------
    data : list[np.ndarray] | np.ndarray | list[dict]
        Simulation history. Supported formats:
          - list of state vectors (1D arrays) length >= 6  (x,y,z, phi,theta,psi, ...)
          - 2D array shape (N, >=6)
          - list of dicts with keys: 'x','y','z' and either 'p_euler' or 'p_quat' (not required)
    title : str
        Plot title.
    zrp : float
        Offset magnitude (meters) from parafoil CM to canopy reference point along body z-axis.
        If body_z_is_down=True, the offset is along -zb (down) like many aero conventions.
    body_z_is_down : bool
        If True, CP offset is [0, 0, -zrp] in the body frame; else [0, 0, +zrp].
    show_cp : bool
        If True, also plot the CP path using the body->inertial rotation from Euler angles.
    """

    # ---------- helpers to extract position & Euler ----------
    def _as_state_array_list(d):
        # Normalize to list of 1D numpy arrays
        if isinstance(d, np.ndarray):
            if d.ndim == 2:
                return [np.asarray(row).ravel() for row in d]
            elif d.ndim == 1:
                return [np.asarray(d).ravel()]
        # list of dicts or arrays
        out = []
        for s in d:
            if isinstance(s, dict):
                # Dict form: prefer explicit fields; fall back to 'state'
                if "state" in s:
                    out.append(np.asarray(s["state"]).ravel())
                else:
                    # try to build from keys (x,y,z, and Euler)
                    x = float(s.get("x", 0.0))
                    y = float(s.get("y", 0.0))
                    z = float(s.get("z", 0.0))
                    # Euler (phi, theta, psi) for parafoil if present, else zeros
                    if "p_euler" in s:
                        eul = np.asarray(s["p_euler"], dtype=float).ravel()
                    else:
                        eul = np.zeros(3)
                    # pad minimal vector to length >= 6
                    arr = np.concatenate([np.array([x, y, z], float), eul])
                    out.append(arr)
            else:
                out.append(np.asarray(s).ravel())
        return out

    states = _as_state_array_list(data)

    # ---------- extract CM trajectory ----------
    xs = [float(s[0]) for s in states]
    ys = [float(s[1]) for s in states]
    zs = [float(s[2]) for s in states]

    # ---------- compute CP trajectory (optional) ----------
    cp_xs, cp_ys, cp_zs = [], [], []
    if show_cp:
        # Offset direction in body frame
        offset_body = np.array([0.0, 0.0, -zrp if body_z_is_down else +zrp], dtype=float)
        for s in states:
            # Euler for parafoil: indices [3:6] = (phiP, thetaP, psiP)
            if s.size >= 6:
                euler_p = s[3:6]
            else:
                euler_p = np.zeros(3, dtype=float)

            # Body->Inertial rotation
            T_IP = np.asarray(makeT_IP(euler_p), dtype=float)  # maps body -> inertial

            # Offset expressed in inertial coords
            offset_I = T_IP @ offset_body

            cp_xs.append(float(s[0] + offset_I[0]))
            cp_ys.append(float(s[1] + offset_I[1]))
            cp_zs.append(float(s[2] + offset_I[2]))

    # ---------- plotting ----------
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    # CM trajectory
    ax.plot(xs, ys, zs, linewidth=2, label="Payload CM Trajectory")
    ax.scatter(xs[0], ys[0], zs[0], label="Start (CM)")
    ax.scatter(xs[-1], ys[-1], zs[-1], label="End (CM)")

    # CP trajectory
    if show_cp:
        ax.plot(cp_xs, cp_ys, cp_zs, linewidth=2, linestyle="--", label="Canopy CP Trajectory")
        ax.scatter(cp_xs[0], cp_ys[0], cp_zs[0], label="Start (CP)")
        ax.scatter(cp_xs[-1], cp_ys[-1], cp_zs[-1], label="End (CP)")

    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.set_title(title)
    ax.legend()
    ax.grid(True)

    # Equal aspect for 3D
    if show_cp:
        all_x = xs + cp_xs
        all_y = ys + cp_ys
        all_z = zs + cp_zs
    else:
        all_x, all_y, all_z = xs, ys, zs

    x_min, x_max = min(all_x), max(all_x)
    y_min, y_max = min(all_y), max(all_y)
    z_min, z_max = min(all_z), max(all_z)
    max_range = 0.5 * max(x_max - x_min, y_max - y_min, z_max - z_min)
    mid_x = 0.5 * (x_max + x_min)
    mid_y = 0.5 * (y_max + y_min)
    mid_z = 0.5 * (z_max + z_min)
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    plt.show()
