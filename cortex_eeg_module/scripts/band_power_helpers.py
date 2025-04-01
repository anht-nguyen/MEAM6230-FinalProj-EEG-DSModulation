import numpy as np
from scipy.signal import welch, butter, lfilter
import time
from config import FS, NUM_CHANNELS, FLEX_CHANNEL_LABELS, TIME_WINDOWS, BANDS, BAND_RATIO_PAIRS


# # For simulation, define configuration variables (if not imported from config)
# FS_SIM = 250  # Sampling frequency in Hz
# NUM_CHANNELS_SIM = 4  # Number of channels for simulation
# FLEX_CHANNEL_LABELS_SIM = ['Fz', 'Cz', 'Pz', 'Oz']

fs = FS  # Sampling frequency in Hz
time_windows = TIME_WINDOWS  # Window length in seconds - Change this to adjust the length of the segments for Welch's method

# Frequency bands definition (Hz)
bands = BANDS
num_channels = NUM_CHANNELS
band_ratio_pairs = BAND_RATIO_PAIRS

def band_powers_labels(flex_channel_labels = FLEX_CHANNEL_LABELS):
    """
    Generate labels for band powers based on channel labels and frequency bands.

    Returns:
    list: List of labels in the format 'Channel/Band'.

    Example:     Band Power Labels:
    ['Cz/theta', 'Cz/alpha', 'Cz/beta', 'Fz/theta', 'Fz/alpha', 'Fz/beta', 'Fp1/theta', 'Fp1/alpha', 'Fp1/beta', 'F7/theta', 'F7/alpha', 'F7/beta', 'F3/theta', 'F3/alpha', 'F3/beta', 'FC1/theta', 'FC1/alpha', 'FC1/beta', 'C3/theta', 'C3/alpha', 'C3/beta', 'FC5/theta', 'FC5/alpha', 'FC5/beta', 'FT9/theta', 'FT9/alpha', 'FT9/beta', 'T7/theta', 'T7/alpha', 'T7/beta', 'CP5/theta', 'CP5/alpha', 'CP5/beta', 'CP1/theta', 'CP1/alpha', 'CP1/beta', 'P3/theta', 'P3/alpha', 'P3/beta', 'P7/theta', 'P7/alpha', 'P7/beta', 'PO9/theta', 'PO9/alpha', 'PO9/beta', 'O1/theta', 'O1/alpha', 'O1/beta', 'Pz/theta', 'Pz/alpha', 'Pz/beta', 'Oz/theta', 'Oz/alpha', 'Oz/beta', 'O2/theta', 'O2/alpha', 'O2/beta', 'PO10/theta', 'PO10/alpha', 'PO10/beta', 'P8/theta', 'P8/alpha', 'P8/beta', 'P4/theta', 'P4/alpha', 'P4/beta', 'CP2/theta', 'CP2/alpha', 'CP2/beta', 'CP6/theta', 'CP6/alpha', 'CP6/beta', 'T8/theta', 'T8/alpha', 'T8/beta', 'FT10/theta', 'FT10/alpha', 'FT10/beta', 'FC6/theta', 'FC6/alpha', 'FC6/beta', 'C4/theta', 'C4/alpha', 'C4/beta', 'FC2/theta', 'FC2/alpha', 'FC2/beta', 'F4/theta', 'F4/alpha', 'F4/beta', 'F8/theta', 'F8/alpha', 'F8/beta', 'Fp2/theta', 'Fp2/alpha', 'Fp2/beta']
    """
    labels = []
    for chan_label in flex_channel_labels:
        for band_name in bands.keys():
            labels.append(f"{chan_label}/{band_name}")
    return labels

#------------------------------------------------------
# Relative Band Power Calculation
#------------------------------------------------------
def relative_band_power(psd, freqs, band):
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
    # print(f"Band: {band}, Low: {low}, High: {high}")
    # print(f"PSD: {psd}")
    # print(f"Freqs: {freqs}")

    idx = np.logical_and(freqs >= low, freqs <= high)
    relative_power = np.sum(psd[idx]) / np.sum(psd)  # Normalize by total power 
    # relative_power = np.sum(psd[idx])  # Total power in the band
    return relative_power

def compute_psd(eeg_data_single_channel, fs=FS, time_windows=TIME_WINDOWS):
    """
    Compute the Power Spectral Density (PSD) using Welch's method.

    Parameters:
    eeg_data_single_channel (np.ndarray): 1D array of EEG data for one channel.
    fs (int): Sampling frequency in Hz.
    time_windows (float): Length of each segment in seconds.

    Returns:
    tuple: Frequencies and corresponding PSD values.
    """
    nperseg = time_windows * fs  # Number of samples per segment for Welch's method
    freqs, psd = welch(eeg_data_single_channel, fs=fs, nperseg=nperseg, nfft=fs*2)
    return freqs, psd

def compute_relative_band_powers_single_channel(eeg_data_single_channel, band_range, fs=FS, time_windows=TIME_WINDOWS):
    """
    Compute the power in a given frequency band for a single EEG channel.

    Parameters:
    eeg_data_single_channel (np.ndarray): 1D array of EEG data for one channel.
    band_range (tuple): Frequency range (low, high) for the band of interest.

    Returns:
    float: Computed relative band powers for the channel.
    """
    freqs, psd = compute_psd(eeg_data_single_channel, fs=fs, time_windows=time_windows)
    power = relative_band_power(psd, freqs, band_range)
    return power

def compute_relative_band_powers_all_channels(eeg_data, num_channels=NUM_CHANNELS):
    """
    Compute the relative band powerss for all channels and frequency bands.

    Parameters:
    eeg_data (np.ndarray): 2D array of EEG data with shape (NUM_CHANNELS, n_samples).

    Returns:
    dict: Dictionary with keys:
        'pow' - List of dictionaries containing relative band powerss for each channel.
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
            # Compute relative band powers for the specific band
            power = compute_relative_band_powers_single_channel(eeg_data_single_channel, band_range)
            channel_band_powers[band_name] = power
            
            band_powers_all.append(power)
        # print(f"Channel {chan+1} relative band powerss: {channel_band_powers}")
    band_powers['pow'] = band_powers_all
    band_powers['time'] = time.time()  # Current timestamp
    return band_powers



#------------------------------------------------------
# Band Power Ratio Calculation
#------------------------------------------------------

def band_power_ratio(psd, freqs, band1, band2):
    """
    Calculate the power ratio between two specified frequency bands.

    This function computes the ratio of the power within one frequency band 
    to the power within another frequency band, based on the provided power 
    spectral density (PSD) and corresponding frequency values.

    Parameters:
        band1 (tuple): Frequency range as a tuple (low1, high1) specifying the first band of interest.
        band2 (tuple): Frequency range as a tuple (low2, high2) specifying the second band of interest.

    Returns:
        float: The ratio of the power in the first frequency band (band1) to the power in the second 
               frequency band (band2).
    """ 
    low1, high1 = band1
    low2, high2 = band2
    idx1 = np.logical_and(freqs >= low1, freqs <= high1)
    idx2 = np.logical_and(freqs >= low2, freqs <= high2)
    # Calculate the ratio of band power in the 2 specified bands
    power_ratio = np.sum(psd[idx1]) / np.sum(psd[idx2]) 
    return power_ratio


def compute_band_power_ratio_single_channel(eeg_data_single_channel, band1, band2, fs=FS, time_windows=TIME_WINDOWS):
    """
    Compute the power ratio between two frequency bands for a single EEG channel.

    Parameters:
        eeg_data_single_channel (np.ndarray): 1D array of EEG data for one channel.
        band1 (tuple): Frequency range (low, high) for the first band of interest.
        band2 (tuple): Frequency range (low, high) for the second band of interest.

    Returns:
        float: Computed power ratio for the channel.
    """
    freqs, psd = compute_psd(eeg_data_single_channel, fs=fs, time_windows=time_windows)
    power_ratio = band_power_ratio(psd, freqs, band1, band2)
    return power_ratio

def compute_band_power_ratio_all_channels(eeg_data, band_ratio_pairs=BAND_RATIO_PAIRS, num_channels=NUM_CHANNELS):
    """
    Compute the power ratio between two frequency bands for all channels.

    Parameters:
        eeg_data (np.ndarray): 2D array of EEG data with shape (NUM_CHANNELS, n_samples).
        band1 (tuple): Frequency range (low, high) for the first band of interest.
        band2 (tuple): Frequency range (low, high) for the second band of interest.

    Returns:
        dict: Dictionary with keys:
            'pow' - List of dictionaries containing power ratios for each channel.
            'time' - Timestamp when the computation was performed.
    """
    band_power_ratios = dict.fromkeys(['pow_ratio', 'time'])
    band_power_ratios_all = []
    # Iterate over each channel (assumes eeg_data is ordered by channel)
    for chan in range(num_channels):
        channel_band_power_ratios = {}
        # Extract EEG data for the current channel (using proper indexing)
        eeg_data_single_channel = eeg_data[chan, :]

        for band_ratio_name, bands_range, in band_ratio_pairs.items():
            # Compute power ratio for the specific bands
            band1_range, band2_range = bands_range
            power_ratio = compute_band_power_ratio_single_channel(eeg_data_single_channel, band1_range, band2_range)
            channel_band_power_ratios[band_ratio_name] = power_ratio
                
            band_power_ratios_all.append(power_ratio)
        # print(f"Channel {chan+1} power ratio: {channel_band_power_ratios}")
    band_power_ratios['pow_ratio'] = band_power_ratios_all
    band_power_ratios['time'] = time.time()  # Current timestamp
    return band_power_ratios



# #------------------------------------------------------
# # Simulation function to generate synthetic EEG data
# #------------------------------------------------------
# import random
# def simulate_eeg_data(duration_sec, fs, num_channels):
#     """
#     Generate simulated EEG data for testing purposes.

#     The simulated data is a combination of a sine wave (to represent a dominant rhythm)
#     and random noise. Different channels are simulated with different dominant frequencies.

#     Parameters:
#     duration_sec (int): Duration of the simulated data in seconds.

#     Returns:
#     np.ndarray: Simulated EEG data with shape (NUM_CHANNELS, duration_sec * FS).
#     """
#     n_samples = duration_sec * fs  # Number of samples

#     simulated_data = np.zeros((num_channels, n_samples))
#     t = np.linspace(0, duration_sec, n_samples, endpoint=False)
#     # Example dominant frequencies for each channel (representative of delta, theta, alpha, beta)
#     frequencies = [random.randint(1, 30) for _ in range(num_channels)]  # Random frequencies between 1 and 30 Hz
#     print(f"Simulated frequencies for channels: {frequencies}")

#     # Generate synthetic EEG data for each channel
#     for i in range(num_channels):
#         # Create a sine wave with added noise
#         simulated_data[i, :] = np.sin(2 * np.pi * frequencies[i] * t) + 0.5 * np.random.randn(n_samples)
#     return simulated_data

# # Simulation test
# if __name__ == "__main__":
#     # Simulate 2 seconds of EEG data
#     simulated_eeg = simulate_eeg_data(duration_sec=2, fs=FS, num_channels=NUM_CHANNELS)
#     print("Simulated EEG Data Shape:", simulated_eeg.shape)

#     # Compute relative band powerss for the simulated data
#     result = compute_relative_band_powers_all_channels(simulated_eeg, NUM_CHANNELS)
#     print("Computed relative band powerss:")
#     print(result)

#     # Display generated relative band powers labels
#     labels = band_powers_labels(FLEX_CHANNEL_LABELS)
#     print("relative band powers Labels:")
#     print(labels)


#     # Compute band power ratios for the simulated data
#     result_ratio = compute_band_power_ratio_all_channels(simulated_eeg, BAND_RATIO_PAIRS, NUM_CHANNELS)
#     print("Computed band power ratios:")    
#     print(result_ratio)