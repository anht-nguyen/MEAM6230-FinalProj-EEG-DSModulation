#!/usr/bin/env python3
import zmq
import time
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
from band_power_helpers import compute_relative_band_powers_all_channels
from config import FLEX_CHANNEL_LABELS  # list of 32 channel names

# Configuration
FS = 128               # Sampling rate (Hz)
TIME_WINDOW = 1.0      # Window length for band-power computation (s)
NUM_CHANNELS = len(FLEX_CHANNEL_LABELS)
BAND_NAMES = ['theta', 'alpha', 'beta']
NUM_BANDS = len(BAND_NAMES)
HISTORY_LEN = 100      # windows to keep

# Synthetic attention conditions: low, medium, high (theta_amp, beta_amp)
CONDITIONS = [
    {'name': 'low_attention',    'theta_amp': 1.0, 'beta_amp': 0.1},
    # {'name': 'medium_attention', 'theta_amp': 0.5, 'beta_amp': 0.5},
    {'name': 'high_attention',   'theta_amp': 0.1, 'beta_amp': 1.0},
]
cond_idx = 0


def generate_synthetic_eeg_window(duration, fs, n_channels, theta_amp, beta_amp):
    t = np.arange(0, duration, 1/fs)
    data = np.zeros((n_channels, len(t)))
    for ch in range(n_channels):
        theta_f = np.random.uniform(5, 7)
        alpha_f = np.random.uniform(9, 11)
        beta_f  = np.random.uniform(18, 22)
        ph = np.random.rand(3) * 2*np.pi
        sig = (theta_amp*np.sin(2*np.pi*theta_f*t+ph[0]) +
               0.3*np.sin(2*np.pi*alpha_f*t+ph[1]) +
               beta_amp*np.sin(2*np.pi*beta_f*t+ph[2]))
        data[ch] = sig + 0.1*np.random.randn(len(t))
    return t, data


def main():
    global cond_idx
    # ZMQ publisher
    ctx = zmq.Context()
    pub = ctx.socket(zmq.PUB)
    pub.bind('tcp://*:5556')
    print(f"[Publisher] Synthetic EEG & band-powers every {TIME_WINDOW}s")
    time.sleep(1)

    # Setup plots
    plt.ion()
    fig_eeg, ax_eeg = plt.subplots(figsize=(8,3))
    ax_eeg.set_xlabel('Time (s)')
    ax_eeg.set_ylabel('Amplitude')
    
    times = deque(maxlen=HISTORY_LEN)
    hist_pows = [deque(maxlen=HISTORY_LEN) for _ in range(NUM_CHANNELS*NUM_BANDS)]
    fig_bp, axs = plt.subplots(NUM_CHANNELS, NUM_BANDS, figsize=(NUM_BANDS*2, NUM_CHANNELS*1), sharex=True)
    lines = []
    for i, ax in enumerate(axs.flat):
        line, = ax.plot([], [], lw=1)
        ax.set_title(BAND_NAMES[i%NUM_BANDS], fontsize=8)
        ax.set_ylabel(FLEX_CHANNEL_LABELS[i//NUM_BANDS], fontsize=6)
        lines.append(line)
    fig_bp.suptitle('Real-time Band Powers')
    fig_bp.subplots_adjust(hspace=0.5, wspace=0.3, top=0.95)

    try:
        while True:
            cond = CONDITIONS[cond_idx]
            cond_idx = (cond_idx+1) % len(CONDITIONS)

            # Generate and compute
            t_win, eeg_win = generate_synthetic_eeg_window(TIME_WINDOW, FS, NUM_CHANNELS,
                                                           cond['theta_amp'], cond['beta_amp'])
            result = compute_relative_band_powers_all_channels(eeg_win)
            # result could be dict with 'pow' key or array
            if isinstance(result, dict) and 'pow' in result:
                flat_pow = result['pow']
            else:
                flat_pow = np.array(result).flatten().tolist()

            # Compute mean theta and beta
            theta_vals = flat_pow[0::NUM_BANDS]
            beta_vals  = flat_pow[2::NUM_BANDS]
            theta_mean = np.mean(theta_vals)
            beta_mean  = np.mean(beta_vals)
            attention_idx = theta_mean/(beta_mean+1e-6)

            # Shade logic
            if attention_idx>1.0: shade=(1,0.8,0.8,0.3)
            elif attention_idx<0.5: shade=(0.8,1,0.8,0.3)
            else: shade=(0.8,0.8,1,0.3)

            # Publish
            msg={'time':time.time(),'condition':cond['name'],
                 'attention_index':attention_idx,'pow':flat_pow}
            pub.send_json(msg)
            print(f"Published {cond['name']} idx={attention_idx:.2f}")

            # Plot EEG
            ax_eeg.clear()
            ax_eeg.set_facecolor(shade)
            for ch in range(NUM_CHANNELS): ax_eeg.plot(t_win,eeg_win[ch],color='gray',alpha=0.3)
            # ax_eeg.set_title(f"{cond['name']}")
            fig_eeg.canvas.draw(); fig_eeg.canvas.flush_events()

            # Update history
            times.append(time.time())
            for i,val in enumerate(flat_pow): hist_pows[i].append(val)

            # Plot band powers
            x=np.array(times)
            for i,line in enumerate(lines):
                y=np.array(hist_pows[i])
                axs.flat[i].set_facecolor(shade)
                line.set_data(x,y)
                axs.flat[i].relim(); axs.flat[i].autoscale_view()

            fig_bp.canvas.draw(); fig_bp.canvas.flush_events()
            time.sleep(TIME_WINDOW)
    except KeyboardInterrupt:
        print('Stopped.')

if __name__=='__main__': main()
