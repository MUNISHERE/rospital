import numpy as np
import torch
from neupan.neupan import NeuPAN  # assuming NeuPAN installed via pip or source

class NeuPANWrapper:
    def __init__(self, model_path, device='cpu'):
        self.device = device
        self.model = NeuPAN.load_from_checkpoint(model_path, map_location=device)
        self.model.eval()

    def compute_cmd(self, laser_data, pose, path):
        """
        laser_data: np.array of ranges
        pose: [x, y, yaw]
        path: list of [x, y]
        """
        # Convert data to torch tensor
        inputs = {
            'laser': torch.tensor(laser_data, dtype=torch.float32).unsqueeze(0),
            'pose': torch.tensor(pose, dtype=torch.float32).unsqueeze(0),
            'path': torch.tensor(path, dtype=torch.float32).unsqueeze(0)
        }
        with torch.no_grad():
            v, w = self.model.predict(inputs)
        return float(v), float(w)
