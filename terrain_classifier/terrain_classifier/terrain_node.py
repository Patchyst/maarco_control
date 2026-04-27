import rclpy
from rclpy.node import Node
import pandas as pd
from scipy.signal import savgol_filter, periodogram
from scipy.stats import skew
import numpy as np
import os
import joblib
from ament_index_python.packages import get_package_share_directory
from terrain_interfaces.msg import Terrain


# Replace with your actual package name
from serial_interfaces.msg import SensorData  
from collections import deque

# --- GLOBAL CONSTANTS ---
FS = 10
S = 20   # or pass this dynamically later

FILTER_CONFIGS = {
    0: (5, 5), 1: (2, 5), 2: (2, 9), 3: (2, 9),
    4: (3, 5), 5: (3, 5), 6: (3, 5), 7: (2, 11),
    8: (2, 7), 9: (2, 5), 10: (2, 5), 11: (2, 5), 12: (2, 5)
}

SENSOR_NAMES = ['TorqL', 'TorqR', 'IavL', 'IavR', 'AccX', 'AccY', 'AccZ', 'Sonar', 'ToF', 'RPML', 'RPMR', 'rollDeg', 'pitchDeg']
SUB_NAMES = ['Var','RMS','Skew','P2P','Energy','FFTMean','FFTMax','FFT_FreqMax','FFT_Power','FFT_BW','PSDMean','PSDStd','PSDPower','PSDPeakF']


def extract_features_to_df(segment):
    """Processes segment and returns a DataFrame with named features for the model."""
    feats_dict = {}
    for s_idx in range(segment.shape[1]):
        sig = segment[:, s_idx]
        order, frame = FILTER_CONFIGS.get(s_idx, (3, 11))
        
        # 1. Start with the desired frame size or the actual data length
        window_size = min(frame, len(sig))
        
        # 2. Savgol requirement: window_size must be > order
        if window_size <= order:
            window_size = order + 1
            
        # 3. Savgol requirement: window_size must be ODD
        if window_size % 2 == 0:
            # If we are at the end of the data, we must go down; 
            # otherwise, going up is usually safer for smoothing.
            if window_size + 1 <= len(sig):
                window_size += 1
            else:
                window_size -= 1

        # Final safety: If window_size is now <= order because we subtracted, 
        # it means the buffer is simply too small for this sensor's config.
        if window_size <= order:
            sig_f = sig # Skip filtering and use raw signal for this window
        else:
            sig_f = savgol_filter(sig, window_size, order)

        # Stats
        mag = np.abs(np.fft.fft(sig_f))[:S//2 + 1]
        freqs, psd = periodogram(sig_f, fs=FS, window='boxcar')
        freq_axis = np.linspace(0, FS/2, len(mag))
        
        # Calculate the 14 features
        vals = [
            #Time Domain
            np.var(sig_f), #Variance
            np.sqrt(np.mean(sig_f**2)), #Root Mean Square 
            skew(sig_f), #Skewness
            np.ptp(sig_f), #Peak 2 Peak
            np.sum(sig_f**2), #Energy
            
            #Frequency Domain
            np.mean(mag), # FFT Mean
            np.max(mag),  # FFT Max
            np.argmax(mag)*(FS/S), # FFT Freq at Max
            np.sum(mag**2)/S, # FFT Power
            np.sqrt(np.sum((freq_axis - (np.argmax(mag)*(FS/S)))**2 * mag)/np.sum(mag)) if np.sum(mag)!=0 else 0, # FFT Bandwidth
            
            # Ppower Spectral Density (PSD)
            np.mean(psd), # PSD Mean
            np.std(psd), # PSD Std
            np.sum(psd),# PSD Power
            freqs[np.argmax(psd)] # PSD Peak Frequency
        ]
        
        # Map to names
        for i, val in enumerate(vals):
            feat_name = f"{SENSOR_NAMES[s_idx]}_{SUB_NAMES[i]}"
            feats_dict[feat_name] = [val]
            
    return pd.DataFrame(feats_dict)


class TerrainNode(Node):
    def __init__(self):
        super().__init__('terrain_node')

        # --- Load model ---
        package_share_dir = get_package_share_directory('terrain_classifier')

        model_path = os.path.join(
            package_share_dir,
            'models',
            'terrain_rf_model_80_20_All.pkl'
        )

        self.get_logger().info(f"Loading model from: {model_path}")

        self.model = joblib.load(model_path)

        # Get expected feature order (very important)
        if hasattr(self.model, "feature_names_in_"):
            self.expected_features = self.model.feature_names_in_
            self.get_logger().info(
                f"Model expects {len(self.expected_features)} features"
            )
        else:
            self.get_logger().warn("Model missing feature_names_in_")

        self.subscription = self.create_subscription(
            SensorData,
            '/sensor_data',   # confirm this matches your system
            self.listener_callback,
            10
        )
        self.FS = 10;
        self.FS = 10
        self.WINDOW_SEC = 2
        self.OVERLAP_SEC = 1

        self.S = int(self.WINDOW_SEC * self.FS)
        self.STEP = self.S - int(self.OVERLAP_SEC * self.FS)

        self.buffer = deque(maxlen=self.S)
        self.new_data_count = 0

        self.get_logger().info("Terrain node started")

    def listener_callback(self, msg):
        try:
            #iav_l = (msg.curr_m_left / 51.2) - 10.0
            #iav_r = (msg.curr_m_right / 51.2) - 10.0

            rpm_l = msg.rpm_left
            rpm_r = msg.rpm_right

            kt = 0.3366

            t_l = kt * msg.curr_m_left
            t_r = kt * msg.curr_m_right

            pitch_correct = 8.5
            roll_correct = -9.4

            roll_deg = -msg.roll - roll_correct
            pitch_deg = msg.pitch - pitch_correct

            ml_vector = [
                t_l, t_r,
                msg.curr_m_left, msg.curr_m_right,
                msg.acc_x, msg.acc_y, msg.acc_z,
                msg.sonar_mm,
                msg.tof_mm,
                rpm_l, rpm_r,
                roll_deg, pitch_deg
            ]
            #----(2) Update buffer ------
            self.buffer.append(ml_vector)
            self.new_data_count +=1

            self.get_logger().info(f"Buffer size: {len(self.buffer)}")
            #----(3) Window trigger -----
            if len(self.buffer) == self.S and self.new_data_count >= self.STEP:
                self.get_logger().info("Window ready for feature extraction")

                # (next step will go here: feature extraction)
                segment = np.array(self.buffer)
                df_features = extract_features_to_df(segment)
                self.get_logger().info(f"Features shape: {df_features.shape}")

                 # --- ALIGN FEATURES (VERY IMPORTANT) ---
                df_final = df_features[self.expected_features] \
                    .fillna(0) \
                    .replace([np.inf, -np.inf], 0)

                # --- MODEL INFERENCE ---
                prediction = self.model.predict(df_final)[0]
                confidence = np.max(self.model.predict_proba(df_final)) * 100

                # --- LOG OUTPUT ---
                self.get_logger().info(
                f"Terrain: {prediction} | Confidence: {confidence:.2f}%"
                )


                self.new_data_count = 0
        except Exception as e:
            self.get_logger().error(f"Error: {e}")    
    

def main(args=None):
    rclpy.init(args=args)
    node = TerrainNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
