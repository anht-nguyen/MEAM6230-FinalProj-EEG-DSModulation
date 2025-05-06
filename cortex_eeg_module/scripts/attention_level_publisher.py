#!/usr/bin/env python3
"""
attention_level_publisher.py

Subscribe to relative band powers via ZeroMQ, compute attention
level per brain region and globally using beta/theta ratio, normalize
indices to [0,1] bounds using an arctan-based mapping, and publish
attention metrics over ZeroMQ with real-time plotting (y-axis fixed to [0,1]).
"""

import zmq
import json
import time
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from config import FLEX_CHANNEL_LABELS, BRAIN_REGIONS, BANDS


def normalize_ratio(ratio):
    """
    Map a non-negative ratio to [0,1) using an arctan-based function:
    normalized = (2 / pi) * arctan(ratio)
    This ensures 0 -> 0, inf -> 1, smoothly bounded.
    """
    return (2.0 / np.pi) * np.arctan(ratio)


def compute_attention(pow_list, epsilon=1e-6):
    """
    Compute and normalize attention index per brain region and globally.
    Raw attention = beta_mean / theta_mean; normalized to [0,1).

    Returns dict of normalized attention or None if invalid.
    """
    num_bands = len(BANDS)
    num_channels = len(FLEX_CHANNEL_LABELS)

    arr = np.array(pow_list)
    if arr.size != num_channels * num_bands:
        print(f"Error: expected {num_channels*num_bands} values, got {arr.size}")
        return {}
    pow_array = arr.reshape(num_channels, num_bands)

    band_names = list(BANDS.keys())
    theta_idx = band_names.index('theta')
    beta_idx  = band_names.index('beta')

    label_to_idx = {lbl: i for i,lbl in enumerate(FLEX_CHANNEL_LABELS)}
    attention = {}

    for region, labels in BRAIN_REGIONS.items():
        idxs = [label_to_idx[l] for l in labels if l in label_to_idx]
        if not idxs:
            attention[region] = None
            continue
        theta_mean = np.mean(pow_array[idxs, theta_idx])
        beta_mean  = np.mean(pow_array[idxs, beta_idx])
        if theta_mean > epsilon:
            raw = beta_mean / theta_mean
            attention[region] = normalize_ratio(raw)
        else:
            attention[region] = None

    # Global
    global_theta = np.mean(pow_array[:, theta_idx])
    global_beta  = np.mean(pow_array[:, beta_idx])
    if global_theta > epsilon:
        raw_g = global_beta / global_theta
        attention['global'] = normalize_ratio(raw_g)
    else:
        attention['global'] = None

    return attention


def main():
    # ZeroMQ setup
    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.connect("tcp://localhost:5556")
    sub.setsockopt_string(zmq.SUBSCRIBE, "")
    pub = ctx.socket(zmq.PUB)
    pub.bind("tcp://*:5557")

    # Plot setup with fixed y-limits
    regions = list(BRAIN_REGIONS.keys()) + ['global']
    fig, axs = plt.subplots(len(regions), 1, figsize=(8, 2 * len(regions)), sharex=True)
    if len(regions) == 1:
        axs = [axs]
    lines = {}
    times = []
    hist = {r: [] for r in regions}

    for ax, region in zip(axs, regions):
        ln, = ax.plot([], [], lw=2)
        lines[region] = ln
        ax.set_ylabel(region)
        ax.set_ylim(0, 1)

    axs[-1].set_xlabel('Time (s)')
    plt.suptitle('Normalized Attention [0,1]')
    plt.ion()
    plt.show(block=False)

    print("Started: listening on 5556, publishing on 5557")

    try:
        while True:
            msg = sub.recv_json()
            t = msg.get('time', time.time())
            pow_list = msg.get('pow', [])
            if not pow_list:
                continue

            att = compute_attention(pow_list)
            out = {'time': t, 'attention': att}
            pub.send_json(out)
            print("Sent attention:", out)

            # Update plots
            times.append(t)
            for region in regions:
                val = att.get(region)
                hist[region].append(val if val is not None else np.nan)
                lines[region].set_data(times, hist[region])

            for ax in axs:
                ax.relim()
                ax.autoscale_view(scaley=False)

            fig.canvas.draw_idle()
            plt.pause(0.001)

    except KeyboardInterrupt:
        print("Exiting attention_level_publisher.")
    finally:
        sub.close()
        pub.close()
        ctx.term()


if __name__ == '__main__':
    main()
