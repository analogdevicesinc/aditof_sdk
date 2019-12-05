import pytest
import os

@pytest.fixture(scope="session")
def calibParams():
   calibParams = {
                  "config_file" : os.path.join('config', 'sweep_config.json'),
                  "firmware-path" : os.path.join('config', 'ADDI9043'),
                  "min-dist" : '0',
                  "max-dist" : '2000',
                  "verify-sweep" : True,
                  "target-distance" : '304.8', # 1 ft
                  "latest_results_path" : 'saved_results/latest',
                  "linear_offset_csv" : 'linear_offset.csv',
                  "depth_stats_csv" : 'depth_stats_pre_calibration.csv',
                  "depth_stats_calib_csv" : 'depth_stats_calibrated.csv',
                  "metrics_csv" : 'calibration_metrics.csv'
                  }
   return calibParams