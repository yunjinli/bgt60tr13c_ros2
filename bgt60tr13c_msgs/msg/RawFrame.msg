builtin_interfaces/Time stamp
uint8  num_rx
uint16 num_samples_per_chirp
uint16 num_chirps_per_frame
# Row-major flattened data with shape [num_rx, num_samples_per_chirp, num_chirps_per_frame]
# Real values; publish magnitude or split real/imag in separate topics if needed
float32[] data
