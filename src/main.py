#!/usr/bin/env python
import numpy as np
import torch
import gym
import gym_mobilerobot
import argparse
import os

import utils
import TD3
import OurDDPG
import DDPG

import rospy
import sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

#---Directory Path---#
dirPath = os.path.dirname(os.path.realpath(__file__))
#---Functions to make network updates---#

if __name__ == "__main__":
	rospy.init_node('ddpg_stage_1',argv=sys.argv, anonymous=False, disable_signals=True, log_level=rospy.INFO)
	sys.argv = rospy.myargv(argv=sys.argv)

	parser = argparse.ArgumentParser()
	parser.add_argument("--policy_name", default="OurDDPG")					# Policy name
	parser.add_argument("--env_name", default="Mobilerobot-v0")			# OpenAI gym environment name
	parser.add_argument("--seed", default=0, type=int)					# Sets Gym, PyTorch and Numpy seeds
	parser.add_argument("--start_timesteps", default=2e3, type=int)		# How many time steps purely random policy is run for
	parser.add_argument("--max_timesteps", default=500, type=int)		# max no of timesteps for each environment
	parser.add_argument("--max_episodes", default=500, type=int)		# max no of timesteps for each environment
	parser.add_argument("--save_models", action="store_true")			# Whether or not models are saved
	parser.add_argument("--expl_noise", default=0.2, type=float)		# Std of Gaussian exploration noise
	parser.add_argument("--batch_size", default=128, type=int)			# Batch size for both actor and critic
	parser.add_argument("--discount", default=0.99, type=float)			# Discount factor
	parser.add_argument("--tau", default=0.001, type=float)				# Target network update rate
	parser.add_argument("--policy_noise", default=0.2, type=float)		# Noise added to target policy during critic update
	parser.add_argument("--noise_clip", default=0.5, type=float)		# Range to clip target policy noise
	parser.add_argument("--policy_freq", default=2, type=int)			# Frequency of delayed policy updates
	args = parser.parse_args()


	file_name = "%s_%s_%s" % (args.policy_name, args.env_name, str(args.seed))
	print "---------------------------------------"
	print "Settings: %s" % (file_name)
	print "---------------------------------------"

	env = gym.make(args.env_name)
	torch.manual_seed(args.seed)
	np.random.seed(args.seed)

	state_dim = env.observation_space.shape[0]
	action_dim = env.action_space.shape[0]
	# max_action = float(env.action_space.high[0])
	max_action = [0.22,2.]

	# Initialize policy
	if args.policy_name == "TD3":policy = TD3.TD3(state_dim, action_dim, max_action)
	elif args.policy_name == "OurDDPG": policy = OurDDPG.DDPG(state_dim, action_dim, max_action)
	elif args.policy_name == "DDPG": policy = DDPG.DDPG(state_dim, action_dim, max_action)

	replay_buffer = utils.ReplayBuffer()

	total_timesteps = 0
	var_v = max_action[0]* args.expl_noise
	var_w = max_action[1]* args.expl_noise *2

	for episode_num in range(args.max_episodes):
		obs = env.reset()
		done = False
		episode_reward = 0

		for episode_timesteps in range(args.max_timesteps):

			if replay_buffer.length() >= 2*args.max_timesteps:
				var_v = max([var_v*0.99999, 0.10*max_action[0]])
				var_w = max([var_w*0.99999, 0.10*max_action[1]])

				if args.policy_name == "TD3":
					policy.train(replay_buffer, episode_timesteps, args.batch_size, args.discount, args.tau, args.policy_noise, args.noise_clip, args.policy_freq)
				else:
					policy.train(replay_buffer, episode_timesteps, args.batch_size, args.discount, args.tau)


            # Check if episode terminated or step-limit reached
			if done or episode_timesteps==args.max_timesteps-1:
				print("Episode Num: %d Total T: %d Episode T: %d Reward: %f") % (episode_num, total_timesteps, episode_timesteps, episode_reward)
				if episode_num%10==0 : policy.save("%s" % (str(episode_num)+ '_actor.pt'), directory=dirPath + '/Models/')

				break

            # Select action
			action = policy.select_action(np.array(obs))
			if args.expl_noise != 0:
				action[0] = (action[0] + np.random.normal(0, var_v, size=1)).clip(0., max_action[0])
				action[1] = (action[1] + np.random.normal(0, var_w, size=1)).clip(-2., max_action[1])

            # Perform action
			new_obs, reward, done, _ = env.step(action)
			done_bool = 0 if episode_timesteps + 1 == args.max_timesteps else float(done)
			episode_reward += reward

			# Store data in replay buffer
			replay_buffer.add((obs, new_obs, action, reward, done_bool))
			obs = new_obs

			total_timesteps += 1
