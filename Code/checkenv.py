from stable_baselines3.common.env_checker import check_env
from omnitrainrlTRIAL import OMNIDIRECTIONAL
env = OMNIDIRECTIONAL()
# It will check your custom environment and output additional warnings if needed
print('CHECK ENV : ')
check_env(env)
print('............')
