import numpy as np
from scipy.signal import welch, butter, lfilter
import time
from config import FS, NUM_CHANNELS, FLEX_CHANNEL_LABELS


# # For simulation, define configuration variables (if not imported from config)
FS_SIM = 250  # Sampling frequency in Hz
NUM_CHANNELS_SIM = 4  # Number of channels for simulation
FLEX_CHANNEL_LABELS_SIM = ['Fz', 'Cz', 'Pz', 'Oz']

fs = FS  # Sampling frequency in Hz
WINDOW_SEC = 2  # Window length in seconds - Change this to adjust the length of the segments for Welch's method

# Frequency bands definition (Hz)
bands = {
    'theta': (4, 8),
    'alpha': (8, 12),
    'beta':  (12, 30)
}

def band_powers_labels(flex_channel_labels = FLEX_CHANNEL_LABELS):
    """
    Generate labels for band powers based on channel labels and frequency bands.

    Returns:
    list: List of labels in the format 'Channel/Band'.
    """
    labels = []
    for chan_label in flex_channel_labels:
        for band_name in bands.keys():
            labels.append(f"{chan_label}/{band_name}")
    return labels

def band_power(psd, freqs, band):
    """
    Calculate the power in a specific frequency band by integrating the power spectral density.

    Parameters:
    psd (np.ndarray): Power spectral density values.
    freqs (np.ndarray): Corresponding frequency values.
    band (tuple): Frequency range as (low, high).

    Returns:
    float: Integrated power within the specified frequency band.
    """
    low, high = band
    idx = np.logical_and(freqs >= low, freqs <= high)
    power = np.trapz(psd[idx], freqs[idx])
    return power

def compute_band_power_single_channel(eeg_data_single_channel, band_range, fs=FS, window_sec=WINDOW_SEC):
    """
    Compute the power in a given frequency band for a single EEG channel.

    Parameters:
    eeg_data_single_channel (np.ndarray): 1D array of EEG data for one channel.
    band_range (tuple): Frequency range (low, high) for the band of interest.

    Returns:
    float: Computed band power for the channel.
    """
    nperseg = window_sec * fs  # Number of samples per segment for Welch's method
    freqs, psd = welch(eeg_data_single_channel, fs=fs, nperseg=nperseg)
    power = band_power(psd, freqs, band_range)
    return power

def compute_band_powers_all_channels(eeg_data, num_channels=NUM_CHANNELS):
    """
    Compute the band powers for all channels and frequency bands.

    Parameters:
    eeg_data (np.ndarray): 2D array of EEG data with shape (NUM_CHANNELS, n_samples).

    Returns:
    dict: Dictionary with keys:
        'pow' - List of dictionaries containing band powers for each channel.
        'time' - Timestamp when the computation was performed.
    """
    band_powers = dict.fromkeys(['pow', 'time'])
    band_powers_all = []
    # Iterate over each channel (assumes eeg_data is ordered by channel)
    for chan in range(num_channels):
        channel_band_powers = {}
        for band_name, band_range in bands.items():
            # Extract EEG data for the current channel (using proper indexing)
            eeg_data_single_channel = eeg_data[chan, :]
            # Compute band power for the specific band
            power = compute_band_power_single_channel(eeg_data_single_channel, band_range)
            channel_band_powers[band_name] = power
            
            band_powers_all.append(power)
        print(f"Channel {chan+1} Band Powers: {channel_band_powers}")
    band_powers['pow'] = band_powers_all
    band_powers['time'] = time.time()  # Current timestamp
    return band_powers






#------------------------------------------------------
# Simulation function to generate synthetic EEG data
#------------------------------------------------------
import random
def simulate_eeg_data(duration_sec, fs, num_channels):
    """
    Generate simulated EEG data for testing purposes.

    The simulated data is a combination of a sine wave (to represent a dominant rhythm)
    and random noise. Different channels are simulated with different dominant frequencies.

    Parameters:
    duration_sec (int): Duration of the simulated data in seconds.

    Returns:
    np.ndarray: Simulated EEG data with shape (NUM_CHANNELS, duration_sec * FS).
    """
    n_samples = duration_sec * fs  # Number of samples

    simulated_data = np.zeros((num_channels, n_samples))
    t = np.linspace(0, duration_sec, n_samples, endpoint=False)
    # Example dominant frequencies for each channel (representative of delta, theta, alpha, beta)
    frequencies = [random.randint(1, 30) for _ in range(num_channels)]  # Random frequencies between 1 and 30 Hz
    # Generate synthetic EEG data for each channel
    for i in range(num_channels):
        # Create a sine wave with added noise
        simulated_data[i, :] = np.sin(2 * np.pi * frequencies[i] * t) + 0.5 * np.random.randn(n_samples)
    return simulated_data

# Simulation test
if __name__ == "__main__":
    # Simulate 2 seconds of EEG data
    simulated_eeg = simulate_eeg_data(duration_sec=2, fs=FS, num_channels=NUM_CHANNELS)
    print("Simulated EEG Data Shape:", simulated_eeg.shape)

    # Compute band powers for the simulated data
    result = compute_band_powers_all_channels(simulated_eeg, NUM_CHANNELS)
    print("Computed Band Powers:")
    print(result)

    # Display generated band power labels
    labels = band_powers_labels(FLEX_CHANNEL_LABELS)
    print("Band Power Labels:")
    print(labels)
