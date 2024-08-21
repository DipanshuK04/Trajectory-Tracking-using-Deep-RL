# import numpy as np
# import gymnasium as gym
# from omnitrainrlTRIAL import OMNIDIRECTIONAL
# from stable_baselines3 import A2C
# import os

# model_dir = "models/A2C"
# logdir = "logs"

# if not os.path.exists(model_dir):
#     os.makedirs(model_dir)
# if not os.path.exists(logdir):
#     os.makedirs(logdir)

# import torch
# torch.device('cuda' if torch.cuda.is_available() else 'cpu')
# # print(torch.device)


# env = OMNIDIRECTIONAL()
# env.reset()

# model = A2C("MlpPolicy",env,verbose = 1,tensorboard_log=logdir)
# TOTALTIMESTEPS = 1000
# for i in range(1,65):
#     model.learn(total_timesteps = TOTALTIMESTEPS,reset_num_timesteps=False,tb_log_name="A2C")
#     model.save(f"{model_dir}/{TOTALTIMESTEPS*i}")
 
# env.close()


import numpy as np
import gymnasium as gym
from omnitrainrlTRIAL import OMNIDIRECTIONAL  # Ensure this import matches your file structure
from stable_baselines3 import A2C
import os
import torch

model_dir = "models/A2C"
logdir = "logs"

# Create directories if they do not exist
if not os.path.exists(model_dir):
    os.makedirs(model_dir)
if not os.path.exists(logdir):
    os.makedirs(logdir)

# Check device (CPU or GPU)
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
print(f"Using device: {device}")

# Initialize environment
env = OMNIDIRECTIONAL()
env.reset()

# Initialize model
model = A2C("MlpPolicy", env, verbose=1, tensorboard_log=logdir)

# Train model
TOTAL_TIMESTEPS = 1000
for i in range(1,50):
    model.learn(total_timesteps=TOTAL_TIMESTEPS, reset_num_timesteps=False, tb_log_name="A2C")
    model.save(f"{model_dir}/A2C_{TOTAL_TIMESTEPS*i}")

env.close()

