YOUR_APP_CLIENT_ID = 'GSOo45bslMJwYoR43a3QCiRiaslR4rKAk7vUSWtL'
YOUR_APP_CLIENT_SECRET = '7XrypYmsVO76smVIsB8KstreTTv1mKc6BQ7iqVgnYJ7r2e8GwlnQCbRnTl2oZ8W9WY1Lo0716H2cQznaZasvQW2g1spuEehZ1ZhEJ23Cj6KNmYztAhO7qUNOb1mwJvDU'

FS = 128  # Sampling frequency

NUM_CHANNELS = 32  # Number of EEG channels

FLEX_CHANNEL_LABELS = ['Cz', 'Fz', 'Fp1', 'F7', 'F3', 'FC1', 'C3', 'FC5', 'FT9', 'T7', 'CP5', 'CP1', 'P3', 'P7', 'PO9', 'O1', 'Pz', 'Oz', 'O2', 'PO10', 'P8', 'P4', 'CP2', 'CP6', 'T8', 'FT10', 'FC6', 'C4', 'FC2', 'F4', 'F8', 'Fp2']

BRAIN_REGIONS = {'frontal': ['Fz', 'Fp1', 'F3', 'FC2', 'F4'],
                    'central': ['Cz', 'FC1', 'C3', 'C4', 'FC2'],
                    'parietal': ['CP1', 'P3', 'Pz', 'P4', 'CP2'],
                    'occipital': ['PO9', 'Oz', 'O1', 'O2', 'PO10'],
                    'right_temporal': ['F7', 'FC5', 'FT9', 'T7', 'CP5', 'P7'],
                    'left_temporal': [ 'P8', 'CP6', 'T8', 'FT10', 'FC6', 'F8']}

# BRAIN_REGIONS = {'frontal': ['Fz', 'Fp1', 'FC2'],
#                     'central': ['Cz', 'FC1', 'C3', 'C4', 'FC2'],
#                     'parietal': [ 'CP1', 'P3', 'Pz', 'P4', 'CP2'],
#                     'occipital': ['PO9', 'Oz', 'O1', 'O2', 'PO10'],
#                     'right_temporal': ['F7', 'F3', 'FC5', 'FT9', 'T7', 'CP5', 'P7'],
#                     'left_temporal': [ 'P8', 'CP6', 'T8', 'FT10', 'FC6', 'F4', 'F8']}

BANDS = {
    'theta': (4, 8),
    'alpha': (8, 12),
    'beta':  (12, 30)
}

BAND_RATIO_PAIRS = {
    'theta/alpha': (BANDS['theta'], BANDS['alpha'])
}

TIME_WINDOWS = 0.125 # (seconds) For calculate band powers - 8 Hz