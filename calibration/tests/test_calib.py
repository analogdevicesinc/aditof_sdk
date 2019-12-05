import pytest
import os
import subprocess
import sys
import numpy as np
import pandas as pd
sys.path.append(os.path.join(os.getcwd(), 'tof_calib'))
import metrics_calculator

def test_run(calibParams):
        # Execute run_calibration.py
        subprocess.check_call(['python', 'run_calibration.py', calibParams["config_file"], 
                                '--firmware-path', calibParams["firmware-path"],
                                '--min-dist', calibParams["min-dist"],
                                '--max-dist', calibParams["max-dist"],
                                '--target-distance', calibParams["target-distance"],
                                '--verify-sweep'
                                ], cwd=os.getcwd())
        # Test output results
        metrics_calculator.calculate_latest_metrics()