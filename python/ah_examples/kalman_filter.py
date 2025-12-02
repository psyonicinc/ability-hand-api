#!/usr/bin/env python3
"""
Auto Hand Validation Data Plotter
Automatically parses and plots P (position) and C (current) values from hand_validation_data.txt
Plots data for fingers 0-3 only
"""

import json
import matplotlib.pyplot as plt
import numpy as np
from typing import List, Dict, Any
import os

class FloatKalmanFilter:
    """
    Python implementation of the FloatKalmanFilter from C code.
    """
    def __init__(self, initial_estimate: float = 0.0, initial_error_cov: float = 0.0, process_var: float = 0.003, meas_var: float = 0.015):
        """
        Initialize the Kalman filter.
        
        Args:
            initial_estimate: Initial state estimate (x̂)
            initial_error_cov: Initial error covariance (P)
            process_var: Process variance (Q)
            meas_var: Measurement variance (R)
        """
        self.x_estimate = initial_estimate      # Estimated state (x̂)
        self.p_error_cov = initial_error_cov    # Error covariance (P)
        self.Q_process_var = process_var        # Process variance (Q)
        self.R_meas_var = meas_var              # Measurement variance (R)
        self.kalman_gain = 0.0                  # Kalman gain (K)
    
    def update(self, measurement: float) -> float:
        """
        Update the Kalman filter with a new measurement.
        
        Args:
            measurement: New measurement value
            
        Returns:
            Updated state estimate
        """
        # Prediction step
        self.p_error_cov += self.Q_process_var
        
        # Update step
        self.kalman_gain = self.p_error_cov / (self.p_error_cov + self.R_meas_var)
        self.x_estimate = self.x_estimate + self.kalman_gain * (measurement - self.x_estimate)
        self.p_error_cov = (1 - self.kalman_gain) * self.p_error_cov
        
        return self.x_estimate
    
    def reset(self, initial_estimate: float = 0.0, initial_error_cov: float = 0.0):
        """
        Reset the filter to initial values.
        
        Args:
            initial_estimate: New initial state estimate
            initial_error_cov: New initial error covariance
        """
        self.x_estimate = initial_estimate
        self.p_error_cov = initial_error_cov
        self.kalman_gain = 0.0

# def parse_hand_validation_data(file_path: str) -> Dict[str, List[List[float]]]:
#     """
#     Parse the hand validation data file and extract P and C values.
    
#     Args:
#         file_path: Path to the hand_validation_data.txt file
        
#     Returns:
#         Dictionary containing 'P' and 'C' data as lists of finger position arrays
#     """
#     p_data = []
#     c_data = []
    
#     try:
#         with open(file_path, 'r') as file:
#             for line in file:
#                 line = line.strip()
#                 if not line:
#                     continue
                    
#                 try:
#                     # Parse JSON line
#                     data = json.loads(line)
                    
#                     # Extract P data (positions)
#                     if 'data' in data and 'P' in data['data']:
#                         p_values = data['data']['P']
#                         if isinstance(p_values, list) and len(p_values) > 0:
#                             # Each element in p_values is a time point with 6 fingers
#                             # We want to extract fingers 0-3 for each time point
#                             for time_point in p_values:
#                                 if isinstance(time_point, list) and len(time_point) >= 4:
#                                     # Take only first 4 fingers (0-3)
#                                     p_data.append(time_point[:4])
                    
#                     # Extract C data (currents) 
#                     if 'data' in data and 'C' in data['data']:
#                         c_values = data['data']['C']
#                         if isinstance(c_values, list) and len(c_values) > 0:
#                             # Each element in c_values is a time point with 6 fingers
#                             # We want to extract fingers 0-3 for each time point
#                             for time_point in c_values:
#                                 if isinstance(time_point, list) and len(time_point) >= 4:
#                                     # Take only first 4 fingers (0-3)
#                                     c_data.append(time_point[:4])
                            
#                 except json.JSONDecodeError as e:
#                     print(f"Warning: Could not parse line as JSON: {e}")
#                     continue
                    
#     except FileNotFoundError:
#         print(f"Error: File {file_path} not found")
#         return {'P': [], 'C': []}
#     except Exception as e:
#         print(f"Error reading file: {e}")
#         return {'P': [], 'C': []}
    
#     return {'P': p_data, 'C': c_data}

def apply_kalman_filter_in_fingers(data: List[List[float]], process_var: float = 0.003, meas_var: float = 0.015) -> List[List[float]]:
    """
    Apply Kalman filtering to the data.
    
    Args:
        data: List of time points, each containing finger values
        process_var: Process variance (Q) - how much we expect the state to change
        meas_var: Measurement variance (R) - how much we trust the measurements
        
    Returns:
        Filtered data with same structure as input
    """
    if not data:
        return []
    
    # Initialize filters for each finger (6 motors)
    filters = []
    for finger_idx in range(6):
        # Initialize with first measurement if available
        initial_estimate = data[0][finger_idx] if data and len(data[0]) > finger_idx else 0.0
        kf = FloatKalmanFilter(
            initial_estimate=initial_estimate,
            initial_error_cov=0.0,
            process_var=process_var,
            meas_var=meas_var
        )
        filters.append(kf)
    
    # Apply filtering to each time point
    filtered_data = []
    for time_point in data:
        filtered_time_point = []
        for finger_idx in range(4):
            if finger_idx < len(time_point):
                # Update filter with measurement
                filtered_value = filters[finger_idx].update(time_point[finger_idx])
                filtered_time_point.append(filtered_value)
            else:
                filtered_time_point.append(0.0)
        filtered_data.append(filtered_time_point)
    
    return filtered_data

# def plot_combined_data(data: Dict[str, List[List[float]]], apply_filter: bool = True):
#     """
#     Plot both P and C data in a combined subplot view with optional Kalman filtering.
    
#     Args:
#         data: Dictionary containing both 'P' and 'C' data
#         apply_filter: Whether to apply Kalman filtering to the data
#     """
#     finger_names = ['Finger 0', 'Finger 1', 'Finger 2', 'Finger 3']
#     colors = ['blue', 'red', 'green', 'orange']
    
#     # Apply Kalman filtering if requested
#     if apply_filter:
#         print("Applying Kalman filtering to data...")
#         filtered_p_data = apply_kalman_filter(data['P']) if data['P'] else []
#         filtered_c_data = apply_kalman_filter(data['C']) if data['C'] else []
#     else:
#         filtered_p_data = data['P']
#         filtered_c_data = data['C']
    
#     fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(15, 10))
    
#     # Plot P data (positions)
#     if data['P']:
#         print(f"Plotting {len(data['P'])} position data points...")
        
#         # Extract raw data for each finger across all time points
#         raw_finger_data_p = [[] for _ in range(4)]
#         for time_point in data['P']:
#             for finger_idx in range(4):
#                 if finger_idx < len(time_point):
#                     raw_finger_data_p[finger_idx].append(time_point[finger_idx])
        
#         # Extract filtered data for each finger across all time points
#         filtered_finger_data_p = [[] for _ in range(4)]
#         for time_point in filtered_p_data:
#             for finger_idx in range(4):
#                 if finger_idx < len(time_point):
#                     filtered_finger_data_p[finger_idx].append(time_point[finger_idx])
        
#         # Create time axis
#         time_axis = np.arange(len(data['P']))
        
#         # Plot both raw and filtered data
#         for finger_idx in range(4):
#             if raw_finger_data_p[finger_idx]:
#                 # Raw data (thin, transparent)
#                 ax1.plot(time_axis, filtered_finger_data_p[finger_idx], 
#                         color=colors[finger_idx], 
#                         linewidth=0.5,
#                         alpha=0.9,
#                         linestyle='-')
                
#                 # # Filtered data (thick, solid)
#                 # ax1.plot(time_axis, filtered_finger_data_p[finger_idx], 
#                 #         color=colors[finger_idx], 
#                 #         label=finger_names[finger_idx],
#                 #         linewidth=2.0,
#                 #         alpha=0.9)
        
#         # Add vertical separation lines (10 sections)
#         max_time = len(data['P']) - 1
#         for i in range(1, 10):
#             x_sep = (max_time * i) / 10
#             ax1.axvline(x=x_sep, color='red', linestyle='--', alpha=0.5, linewidth=0.8)
    
#     ax1.set_xlabel('Time Points')
#     ax1.set_ylabel('Position Values')
#     filter_text = " (Kalman Filtered)" if apply_filter else ""
#     ax1.set_title(f'Hand Validation - Position Data (Fingers 0-3) - 10 Sections{filter_text}')
#     ax1.legend()
#     ax1.grid(True, alpha=0.3)
    
#     # Plot C data (currents)
#     if data['C']:
#         print(f"Plotting {len(data['C'])} current data points...")
        
#         # Extract raw data for each finger across all time points
#         raw_finger_data_c = [[] for _ in range(4)]
#         for time_point in data['C']:
#             for finger_idx in range(4):
#                 if finger_idx < len(time_point):
#                     raw_finger_data_c[finger_idx].append(time_point[finger_idx])
        
#         # Extract filtered data for each finger across all time points
#         filtered_finger_data_c = [[] for _ in range(4)]
#         for time_point in filtered_c_data:
#             for finger_idx in range(4):
#                 if finger_idx < len(time_point):
#                     filtered_finger_data_c[finger_idx].append(time_point[finger_idx])
        
#         # Create time axis
#         time_axis = np.arange(len(data['C']))
        
#         # Plot both raw and filtered data
#         for finger_idx in range(4):
#             if raw_finger_data_c[finger_idx]:
#                 # Raw data (thin, transparent)
#                 ax2.plot(time_axis, raw_finger_data_c[finger_idx], 
#                         color=colors[finger_idx], 
#                         linewidth=0.5,
#                         alpha=0.2,
#                         linestyle='-')
                
#                 # Filtered data (thick, solid)
#                 ax2.plot(time_axis, filtered_finger_data_c[finger_idx], 
#                         color=colors[finger_idx], 
#                         label=finger_names[finger_idx],
#                         linewidth=2.0,
#                         alpha=0.9)
        
#         # Add vertical separation lines (10 sections)
#         max_time = len(data['C']) - 1
#         for i in range(1, 10):
#             x_sep = (max_time * i) / 10
#             ax2.axvline(x=x_sep, color='red', linestyle='--', alpha=0.5, linewidth=0.8)
    
#     ax2.set_xlabel('Time Points')
#     ax2.set_ylabel('Current Values')
#     filter_text = " (Kalman Filtered)" if apply_filter else ""
#     ax2.set_title(f'Hand Validation - Current Data (Fingers 0-3) - 10 Sections{filter_text}')
#     ax2.legend()
#     ax2.grid(True, alpha=0.3)
    
#     plt.tight_layout()
    
#     # Save the combined plot
#     output_filename = 'hand_validation_combined_plot.png'
#     # plt.savefig(output_filename, dpi=300, bbox_inches='tight')
#     # print(f"Combined plot saved as: {output_filename}")
    
#     plt.show()

# def print_data_summary(data: Dict[str, List[List[float]]]):
#     """
#     Print a summary of the parsed data.
    
#     Args:
#         data: Dictionary containing the parsed data
#     """
#     print("=== Hand Validation Data Summary ===")
#     print(f"Position (P) time points: {len(data['P'])}")
#     print(f"Current (C) time points: {len(data['C'])}")
    
#     if data['P']:
#         print(f"Position fingers per time point: {len(data['P'][0])}")
#         print(f"Position data range: {min(data['P'][0]) if data['P'][0] else 'N/A'} to {max(data['P'][0]) if data['P'][0] else 'N/A'}")
    
#     if data['C']:
#         print(f"Current fingers per time point: {len(data['C'][0])}")
#         print(f"Current data range: {min(data['C'][0]) if data['C'][0] else 'N/A'} to {max(data['C'][0]) if data['C'][0] else 'N/A'}")

# def main():
#     """Main function to run the hand validation data plotter."""
#     # Get the script directory
#     script_dir = os.path.dirname(os.path.abspath(__file__))
#     data_file = os.path.join(script_dir, 'hand_validation_data.txt')
    
#     print("Auto Hand Validation Data Plotter with Kalman Filtering")
#     print("=" * 60)
#     print(f"Reading data from: {data_file}")
    
#     # Parse the data
#     data = parse_hand_validation_data(data_file)
    
#     # Print summary
#     print_data_summary(data)
    
#     if not data['P'] and not data['C']:
#         print("No data found to plot!")
#         return
    
#     # Ask user about filtering
#     print("\nFiltering options:")
#     print("1. Plot with Kalman filtering (recommended)")
#     print("2. Plot without filtering (raw data only)")
#     print("3. Plot both (raw + filtered)")
    
#     try:
#         choice = 3
        
#         if choice == '2':
#             # No filtering
#             plot_combined_data(data, apply_filter=False)
#         elif choice == '3':
#             # Both plots
#             print("\nGenerating plot with Kalman filtering...")
#             plot_combined_data(data, apply_filter=True)

#         else:
#             # Default: with filtering
#             plot_combined_data(data, apply_filter=True)
            
#     except KeyboardInterrupt:
#         print("\nPlotting cancelled by user")
#     except Exception as e:
#         print(f"Error during plotting: {e}")
#         # Fallback to default
#         plot_combined_data(data, apply_filter=True)

# if __name__ == "__main__":
#     main()
