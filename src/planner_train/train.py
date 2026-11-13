import os
import random
import time
from datetime import datetime

import h5py
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from sklearn.model_selection import train_test_split
from torch.optim.lr_scheduler import CosineAnnealingLR
from torch.utils.data import DataLoader, Dataset
from torch.utils.tensorboard import SummaryWriter
from tqdm import tqdm

# ========================== Training params ==========================
ADD_NOISE_STATES = True
ADD_NOISE_CLOUD = True
STATE_NOISE_RATIO = 0.03  # 0.03
CLOUD_NOISE_RATIO = 0.04  # 0.04
P_PRO = 0.6  # 0.6
R_NUM_MIN = 0.5  # 0.5

MAX_PCD_POINTS = 666  # 666

VAL_SIZE = 0.3
BATCH_SIZE_TRAIN = 128
BATCH_SIZE_VAL = 64
NUM_WORKERS = 8

NUM_EPOCHS = 600
LR = 1e-3
T_MAX = 0.3 * NUM_EPOCHS
WEIGHT_DECAY = 1e-4
ETA_MIN = 0.1 * LR
PATIENCE = 20


# ========================== Device configuration ==========================
def set_seed(seed=42):
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed(seed)
        torch.cuda.manual_seed_all(seed)
    torch.backends.cudnn.deterministic = True
    torch.backends.cudnn.benchmark = False
    os.environ["PYTHONHASHSEED"] = str(seed)


set_seed(42)

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"Using device: {device}")


# ========================== Dataset class ==========================
class Npe2eDataset(Dataset):
    def __init__(
        self,
        file_list,
        max_pcdPoints=666,
        add_noise_states=False,
        add_noise_cloud=False,
    ):
        self.file_list = file_list
        self.max_pcdPoints = max_pcdPoints
        self.add_noise_states = add_noise_states
        self.add_noise_cloud = add_noise_cloud
        self.data = []
        self.load_data()

    def load_data(self):
        durations = []
        for h5_file in tqdm(self.file_list, desc="Loading dataset in HDF5 files"):
            with h5py.File(h5_file, "r") as f:
                for group_name in tqdm(
                    f.keys(),
                    desc=f"Loading groups in {os.path.basename(h5_file)}",
                    leave=False,
                ):
                    group = f[group_name]
                    try:
                        vel = np.array([group["vel_magnitude"][()]], dtype=np.float32)
                        acc = group["acc_velFrame"][:].astype(np.float32)
                        dir_target = group["dir_target_from_drone"][:].astype(
                            np.float32
                        )
                        cloud = group["processed_sensedCloud_velFrame"][:].T.astype(
                            np.float32
                        )
                        coeffs = group["coeffs"][:].T.astype(np.float32)
                        duration = float(group["duration"][()])
                        coeffs_non_fixed = coeffs[:, 0:3].flatten()

                        original_points = cloud.shape[0]
                        if original_points > 1:
                            if np.random.rand() < P_PRO:
                                sample_ratio = np.random.uniform(R_NUM_MIN, 1.0)
                                target_points = max(
                                    1, int(original_points * sample_ratio)
                                )
                                idx = np.random.choice(
                                    original_points, target_points, replace=False
                                )
                                cloud = cloud[idx]

                        if cloud.shape[0] > self.max_pcdPoints:
                            idx = np.random.choice(
                                cloud.shape[0], self.max_pcdPoints, replace=False
                            )
                            cloud = cloud[idx]
                        elif cloud.shape[0] < self.max_pcdPoints:
                            pad_size = self.max_pcdPoints - cloud.shape[0]
                            cloud = np.pad(
                                cloud,
                                ((0, pad_size), (0, 0)),
                                mode="constant",
                                constant_values=100.0,
                            )

                        if self.add_noise_states:
                            vel_abs = np.abs(vel)
                            vel_noise_std = np.maximum(
                                STATE_NOISE_RATIO * vel_abs, 1e-4
                            )
                            vel += np.random.normal(0, vel_noise_std, vel.shape)

                            acc_abs = np.abs(acc)
                            acc_noise_std = np.maximum(
                                STATE_NOISE_RATIO * acc_abs, 1e-4
                            )
                            acc += np.random.normal(0, acc_noise_std, acc.shape)

                            dir_abs = np.abs(dir_target)
                            dir_noise_std = np.maximum(
                                STATE_NOISE_RATIO * dir_abs, 1e-4
                            )
                            dir_target += np.random.normal(
                                0, dir_noise_std, dir_target.shape
                            )

                        if self.add_noise_cloud:
                            cloud_abs = np.abs(cloud)
                            cloud_noise_std = CLOUD_NOISE_RATIO * cloud_abs
                            cloud_noise_std = np.maximum(cloud_noise_std, 1e-3)
                            cloud_noise_std = np.where(
                                cloud_abs >= 50.0, 0.0, cloud_noise_std
                            )
                            cloud += np.random.normal(0, cloud_noise_std, cloud.shape)

                        state_features = np.concatenate([vel, acc, dir_target], axis=0)
                        durations.append(duration)

                        self.data.append(
                            {
                                "state_features": state_features,
                                "cloud": cloud,
                                "coeffs_non_fixed": coeffs_non_fixed,
                                "vel": vel,
                                "acc": acc,
                            }
                        )
                    except Exception as e:
                        print(f"Error loading {group_name} in {h5_file}: {e}")
                        continue

        if not self.data:
            raise RuntimeError("No valid samples were loaded from the dataset.")

        self.duration_mean = float(np.mean(durations))

    def __getitem__(self, idx):
        d = self.data[idx]
        return (
            torch.tensor(d["state_features"], dtype=torch.float32),
            torch.tensor(d["cloud"], dtype=torch.float32),
            torch.tensor(d["coeffs_non_fixed"], dtype=torch.float32),
            torch.tensor(d["vel"], dtype=torch.float32),
            torch.tensor(d["acc"], dtype=torch.float32),
        )

    def __len__(self):
        return len(self.data)


# ========================== PointMLP class ==========================
class PointMLP(nn.Module):
    def __init__(self, point_dim=3, hidden_dim=128, output_dim=32):
        super(PointMLP, self).__init__()
        self.point_mlp = nn.Sequential(
            nn.Linear(point_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim // 2),
        )
        self.global_mlp = nn.Sequential(
            nn.Linear(hidden_dim // 2, hidden_dim // 2),
            nn.ReLU(),
            nn.Linear(hidden_dim // 2, output_dim),
        )

    def forward(self, x):
        point_features = self.point_mlp(x)
        global_feature = torch.max(point_features, dim=1)[0]
        return self.global_mlp(global_feature)


# ========================== Npe2eNet class ==========================
class Npe2eNet(nn.Module):
    def __init__(self, duration_mean):
        super(Npe2eNet, self).__init__()
        self.point_mlp = PointMLP(point_dim=3, hidden_dim=128, output_dim=32)
        self.cat_mlp = nn.Sequential(
            nn.Linear(32 + 7, 256),
            nn.ReLU(),
            nn.Linear(256, 512),
            nn.ReLU(),
            nn.Linear(512, 256),
            nn.ReLU(),
        )
        self.coeff_mlp = nn.Sequential(
            nn.Linear(256, 128), nn.ReLU(), nn.Linear(128, 128), nn.ReLU()
        )

        self.fc_coeffs_out = nn.Linear(128, 9)

        self.register_buffer(
            "duration_mean", torch.tensor(duration_mean, dtype=torch.float32)
        )

    def forward(self, state_features, cloud, ini_vel, ini_acc):
        cloud_features = self.point_mlp(cloud)
        x = torch.cat([cloud_features, state_features], dim=1)
        x = self.cat_mlp(x)
        coeffs_non_fixed_out = self.coeff_mlp(x)
        coeffs_non_fixed_out = self.fc_coeffs_out(coeffs_non_fixed_out)

        batch_size = state_features.size(0)
        duration_out = self.duration_mean.expand(batch_size)
        coeffs_fixed = torch.zeros(batch_size, 9, device=state_features.device)
        coeffs_fixed[:, 0] = ini_acc[:, 0] / 2
        coeffs_fixed[:, 3] = ini_acc[:, 1] / 2
        coeffs_fixed[:, 6] = ini_acc[:, 2] / 2
        coeffs_fixed[:, 1] = ini_vel[:, 0]
        coeffs_x = torch.cat(
            [coeffs_non_fixed_out[:, 0:3], coeffs_fixed[:, 0:3]], dim=1
        )
        coeffs_y = torch.cat(
            [coeffs_non_fixed_out[:, 3:6], coeffs_fixed[:, 3:6]], dim=1
        )
        coeffs_z = torch.cat(
            [coeffs_non_fixed_out[:, 6:9], coeffs_fixed[:, 6:9]], dim=1
        )
        coeffs_out = torch.cat([coeffs_x, coeffs_y, coeffs_z], dim=1).view(
            batch_size, 3, 6
        )
        return coeffs_non_fixed_out, duration_out, coeffs_out


# ========================== Training function ==========================
def train_model(model, train_loader, val_loader, num_epochs, lr, weight_decay):
    log_base_dir = os.path.join("runs", datetime.now().strftime("%Y%m%d-%H%M%S"))
    os.makedirs(log_base_dir, exist_ok=True)
    model_path = os.path.join(log_base_dir, "policy_model.pt")
    train_writer = SummaryWriter(log_dir=os.path.join(log_base_dir, "train"))
    val_writer = SummaryWriter(log_dir=os.path.join(log_base_dir, "val"))
    print(f"TensorBoard logs saved to: {log_base_dir}")

    # optimizer
    optimizer = optim.AdamW(model.parameters(), lr=lr, weight_decay=weight_decay)
    scheduler = CosineAnnealingLR(optimizer, T_max=T_MAX, eta_min=ETA_MIN)
    criterion = nn.MSELoss()
    best_val_loss = float("inf")
    patience = PATIENCE
    patience_counter = 0

    for epoch in range(num_epochs):
        # ---------- training ----------
        model.train()
        train_loss = 0

        for batch_idx, (
            state_features,
            cloud,
            coeffs_non_fixed,
            vel,
            acc,
        ) in enumerate(tqdm(train_loader, desc=f"Epoch {epoch + 1}/{num_epochs}")):
            (
                state_features,
                cloud,
                coeffs_non_fixed,
                vel,
                acc,
            ) = (
                state_features.to(device),
                cloud.to(device),
                coeffs_non_fixed.to(device),
                vel.to(device),
                acc.to(device),
            )
            optimizer.zero_grad()
            pred_coeffs_non_fixed, _, _ = model(state_features, cloud, vel, acc)
            loss = criterion(pred_coeffs_non_fixed, coeffs_non_fixed)
            loss.backward()
            optimizer.step()

            global_step = epoch * len(train_loader) + batch_idx
            train_writer.add_scalar("Batch/Loss", loss.item(), global_step)

            train_loss += loss.item() * state_features.size(0)

        train_loss /= len(train_loader.dataset)

        # ---------- validation ----------
        model.eval()
        val_loss = 0
        with torch.no_grad():
            for v_batch_idx, (
                state_features,
                cloud,
                coeffs_non_fixed,
                vel,
                acc,
            ) in enumerate(val_loader):
                (
                    state_features,
                    cloud,
                    coeffs_non_fixed,
                    vel,
                    acc,
                ) = (
                    state_features.to(device),
                    cloud.to(device),
                    coeffs_non_fixed.to(device),
                    vel.to(device),
                    acc.to(device),
                )
                pred_coeffs_non_fixed, _, _ = model(state_features, cloud, vel, acc)
                v_loss = criterion(pred_coeffs_non_fixed, coeffs_non_fixed)

                val_step = epoch * len(val_loader) + v_batch_idx
                val_writer.add_scalar("Batch/Loss", v_loss.item(), val_step)

                val_loss += v_loss.item() * state_features.size(0)

        val_loss /= len(val_loader.dataset)

        train_writer.add_scalar("Epoch/Loss", train_loss, epoch)
        val_writer.add_scalar("Epoch/Loss", val_loss, epoch)
        train_writer.add_scalar(
            "Params/Learning_Rate", optimizer.param_groups[0]["lr"], epoch
        )

        print(
            f"  Results of epoch {epoch + 1}/{num_epochs}:\n"
            f"  Train Loss: {train_loss:.6f}\n"
            f"  Val Loss: {val_loss:.6f}"
        )

        should_stop = False
        if val_loss < best_val_loss:
            best_val_loss = val_loss
            patience_counter = 0
        else:
            patience_counter += 1
            if patience_counter >= patience:
                should_stop = True

        scheduler.step()

        model.eval()
        scripted_model = torch.jit.script(model)
        scripted_model.save(model_path)
        print(f"  Latest model saved to: {model_path}")
        model.train()

        if should_stop:
            print("Early stopping triggered.")
            break

    train_writer.close()
    val_writer.close()


# ========================== run_train function ==========================
def run_train():
    dataSet_dir = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "dataSetFile"
    )
    if not os.path.exists(dataSet_dir):
        print(f"Error: {dataSet_dir} does not exist.")
        return

    h5_files = sorted(
        [
            os.path.join(dataSet_dir, f)
            for f in os.listdir(dataSet_dir)
            if f.endswith((".h5", ".hdf5"))
        ]
    )
    if not h5_files:
        print("No HDF5 files found.")
        return

    train_files, val_files = train_test_split(
        h5_files, test_size=VAL_SIZE, random_state=42
    )

    train_dataset = Npe2eDataset(
        train_files,
        max_pcdPoints=MAX_PCD_POINTS,
        add_noise_states=ADD_NOISE_STATES,
        add_noise_cloud=ADD_NOISE_CLOUD,
    )
    val_dataset = Npe2eDataset(
        val_files,
        max_pcdPoints=MAX_PCD_POINTS,
        add_noise_states=ADD_NOISE_STATES,
        add_noise_cloud=ADD_NOISE_CLOUD,
    )

    model = Npe2eNet(duration_mean=train_dataset.duration_mean).to(device)

    train_loader = DataLoader(
        train_dataset,
        batch_size=BATCH_SIZE_TRAIN,
        shuffle=True,
        num_workers=NUM_WORKERS,
        pin_memory=True,
    )
    val_loader = DataLoader(
        val_dataset,
        batch_size=BATCH_SIZE_VAL,
        shuffle=False,
        num_workers=NUM_WORKERS,
        pin_memory=True,
    )

    print("Starting training...")
    start = time.time()
    train_model(
        model,
        train_loader,
        val_loader,
        num_epochs=NUM_EPOCHS,
        lr=LR,
        weight_decay=WEIGHT_DECAY,
    )
    end = time.time()
    tot = end - start
    print(
        f"Total training time: {int(tot // 3600)}h {int((tot % 3600) // 60)}m {int(tot % 60)}s"
    )


# ========================== Main ==========================
if __name__ == "__main__":
    run_train()
