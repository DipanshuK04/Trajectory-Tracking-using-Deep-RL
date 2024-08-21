import gymnasium as gym
import pybullet as p
from gymnasium import spaces
import numpy as np
import pybullet_data

class OMNIDIRECTIONAL(gym.Env):
    def __init__(self):
        super(OMNIDIRECTIONAL, self).__init__()

        # Define observation and action space
        self.observation_space = spaces.Box(low=np.array([-1, -1, -1, -1, -1, -1, -np.pi, -np.pi, -np.pi, -1, -1, -1, -1]),
                                            high=np.array([1, 1, 1, 1, 1, 1, np.pi, np.pi, np.pi, 1, 1, 1, 1]),
                                            shape=(13,), dtype=np.float64)
        
        self.action_space = spaces.Box(low=np.array([-1, -1, -1, -1]),
                                       high=np.array([1, 1, 1, 1]),
                                       shape=(4,), dtype=np.float32)
        
        # Initialize PyBullet
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)
        self.planeId = p.loadURDF("plane.urdf")
        cubeStartPos = [0, 0, 0]
        cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
        self.robotId = p.loadURDF("OMNIDIRECTIONAL.urdf", cubeStartPos, cubeStartOrientation, flags=p.URDF_USE_INERTIA_FROM_FILE)
        
        self.target_pos = np.array([1, 1, 0])
        self.done = False
        self.reward = 0.0
        
        # Define wheel and roller joints
        front_left_wheel = 0
        back_left_wheel = 9
        front_right_wheel = 18
        back_right_wheel = 27
        self.wheel_ids = [front_left_wheel, back_left_wheel, front_right_wheel, back_right_wheel]
        self.roller_joints = [i for i in range(p.getNumJoints(self.robotId)) if i not in self.wheel_ids]

    def step(self, action):
        p.setJointMotorControlArray(self.robotId, self.wheel_ids, p.VELOCITY_CONTROL, targetVelocities=10 * action)
        for joint in self.roller_joints:
            p.setJointMotorControl2(self.robotId, joint, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
        p.stepSimulation()
  
        n_car_pos, n_car_orientation = p.getBasePositionAndOrientation(self.robotId)
        n_car_vel, n_angular_vel = p.getBaseVelocity(self.robotId)
        n_car_orientation_euler = p.getEulerFromQuaternion(n_car_orientation)

        distance_to_target = np.linalg.norm(np.array(n_car_pos) - self.target_pos)
        direction_vector = self.target_pos - np.array(n_car_pos)
        
        # Normalize the velocity vector and direction vector
        if np.linalg.norm(n_car_vel) != 0:
            n_car_vel_norm = np.array(n_car_vel[:2]) / np.linalg.norm(np.array(n_car_vel[:2]))
        else:
            n_car_vel_norm = np.zeros(2)
        
        if np.linalg.norm(direction_vector) != 0:
            direction_vector_norm = direction_vector[:2] / np.linalg.norm(direction_vector[:2])
        else:
            direction_vector_norm = np.zeros(2)

        self.reward = 10 * np.dot(n_car_vel_norm, direction_vector_norm)
        self.init_dis = np.linalg.norm(np.array(self.target_pos))
        self.reward += -10 * distance_to_target

        if distance_to_target < 0.15:
            self.reward += 500
            self.terminated = True
        else:
            self.terminated = False

        if distance_to_target > 1.5 * self.init_dis:
            self.reward += -500
            self.truncated = True
        else:
            self.truncated = False
        
        n_car_pos_norm = np.array(n_car_pos[:2]) / np.linalg.norm(np.array(n_car_pos[:2]))
        joint_motor_torques = [p.getJointState(self.robotId, joint_id)[3] for joint_id in self.wheel_ids]
        
        # Normalize joint motor torques if needed
        joint_motor_torques = np.tanh(joint_motor_torques)  # Using tanh to normalize the torques

        self.new_state = np.concatenate((n_car_pos_norm, n_car_vel_norm, direction_vector_norm, n_car_orientation_euler, joint_motor_torques))
        
        info = {}
        
        return self.new_state, self.reward, self.terminated, self.truncated, info
    
    def reset(self, seed=0):
        p.setGravity(0, 0, -9.8)
        self.car_start_pos = [0, 0, 0.2]
        self.car_start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        p.resetBasePositionAndOrientation(self.robotId, self.car_start_pos, self.car_start_orientation)
        p.stepSimulation()
        
        car_pos, n_car_orientation = p.getBasePositionAndOrientation(self.robotId)
        car_vel, angular_vel = p.getBaseVelocity(self.robotId)
        
        direction_vector = self.target_pos - np.array(car_pos)
        direction_vector_norm = direction_vector[:2] / np.linalg.norm(direction_vector[:2])
        n_car_orientation_euler = p.getEulerFromQuaternion(n_car_orientation)
        
        car_pos_n = np.array(car_pos[:2]) / np.linalg.norm(np.array(car_pos[:2]))
        joint_motor_torques = [p.getJointState(self.robotId, joint_id)[3] for joint_id in self.wheel_ids]
        joint_motor_torques = np.tanh(joint_motor_torques)  # Using tanh to normalize the torques

        self.state = np.concatenate((car_pos_n, car_vel[:2], direction_vector_norm, n_car_orientation_euler, joint_motor_torques))
        
        return self.state, {}

    def render(self, mode='human'):
        pass
    
    def close(self):
        p.disconnect()
