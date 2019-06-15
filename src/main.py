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

# Runs policy for X episodes and returns average reward
# def evaluate_policy(policy, eval_episodes=10):
# 	avg_reward = 0.
# 	for _ in xrange(eval_episodes):
# 		obs = env.reset()
# 		done = False
# 		while not done:
# 			action = policy.select_action(np.array(obs))
# 			obs, reward, done, _ = env.step(action)
# 			avg_reward += reward
#
# 	avg_reward /= eval_episodes
#
# 	print "---------------------------------------"
# 	print "Evaluation over %d episodes: %f" % (eval_episodes, avg_reward)
# 	print "---------------------------------------"
# 	return avg_reward


if __name__ == "__main__":
	rospy.init_node('ddpg_stage_1',argv=sys.argv, anonymous=False, disable_signals=True, log_level=rospy.INFO)
	sys.argv = rospy.myargv(argv=sys.argv)

	parser = argparse.ArgumentParser()
	parser.add_argument("--policy_name", default="DDPG")					# Policy name
	parser.add_argument("--env_name", default="Mobilerobot-v0")			# OpenAI gym environment name
	parser.add_argument("--seed", default=0, type=int)					# Sets Gym, PyTorch and Numpy seeds
	parser.add_argument("--start_timesteps", default=1e4, type=int)		# How many time steps purely random policy is run for
	parser.add_argument("--eval_freq", default=5e3, type=float)			# How often (time steps) we evaluate
	parser.add_argument("--max_timesteps", default=500, type=float)		# max no of timesteps for each environment
	parser.add_argument("--save_models", action="store_true")			# Whether or not models are saved
	parser.add_argument("--expl_noise", default=0.2, type=float)		# Std of Gaussian exploration noise
	parser.add_argument("--batch_size", default=100, type=int)			# Batch size for both actor and critic
	parser.add_argument("--discount", default=0.99, type=float)			# Discount factor
	parser.add_argument("--tau", default=0.005, type=float)				# Target network update rate
	parser.add_argument("--policy_noise", default=0.2, type=float)		# Noise added to target policy during critic update
	parser.add_argument("--noise_clip", default=0.5, type=float)		# Range to clip target policy noise
	parser.add_argument("--policy_freq", default=2, type=int)			# Frequency of delayed policy updates
	args = parser.parse_args()


	file_name = "%s_%s_%s" % (args.policy_name, args.env_name, str(args.seed))
	print "---------------------------------------"
	print "Settings: %s" % (file_name)
	print "---------------------------------------"

	if not os.path.exists("./results"):
		os.makedirs("./results")

	env = gym.make(args.env_name)

#---Functions to make network updates---#
	env.seed(args.seed)
	torch.manual_seed(args.seed)
	np.random.seed(args.seed)

	state_dim = env.observation_space.shape[0]
	action_dim = env.action_space.shape[0]
	max_action = float(env.action_space.high[0])

	# Initialize policy
	if args.policy_name == "TD3":policy = TD3.TD3(state_dim, action_dim, max_action)
	elif args.policy_name == "OurDDPG": policy = OurDDPG.DDPG(state_dim, action_dim, max_action)
	elif args.policy_name == "DDPG": policy = DDPG.DDPG(state_dim, action_dim, max_action)

	replay_buffer = utils.ReplayBuffer()

	total_timesteps = 0
	timesteps_since_eval = 0
	episode_num = 0
	done = True

	while True:

		for  episode_timesteps in range(args.max_timesteps):
			if done or episode_timesteps == (args.max_timesteps-1):

				if episode_num%10==0 : policy.save("%s" % (str(episode_num)+ '_actor.pt'), directory=dirPath + '/Models/')

				if total_timesteps != 0:
					print("Episode Num: %d Total T: %d Episode T: %d Reward: %f") % (episode_num, total_timesteps, episode_timesteps, episode_reward)
					if args.policy_name == "TD3":
						policy.train(replay_buffer, episode_timesteps, args.batch_size, args.discount, args.tau, args.policy_noise, args.noise_clip, args.policy_freq)
					else:
						policy.train(replay_buffer, episode_timesteps, args.batch_size, args.discount, args.tau)

				break

				# Reset environment

				obs = env.reset()
				done = False
				episode_reward = 0

				episode_num += 1

			# Select action randomly or according to policy
			if total_timesteps < args.start_timesteps:
				action = env.action_space.sample()
			else:
				action = policy.select_action(np.array(obs))
				if args.expl_noise != 0:
					action[0] = (action[0] + np.random.normal(0, args.expl_noise, size=env.action_space.shape[0])).clip(env.action_space.low[0], env.action_space.high[0])
					action[1] = (action[1] + np.random.normal(0, args.expl_noise, size=env.action_space.shape[0])).clip(env.action_space.low[1], env.action_space.high[1])

			# Perform action
			print('Episode timesteps:',episode_timesteps)
			new_obs, reward, done, _ = env.step(action)
			done_bool = 0 if episode_timesteps + 1 == args.max_timesteps else float(done)
			episode_reward += reward

			# Store data in replay buffer
			replay_buffer.add((obs, new_obs, action, reward, done_bool))

			obs = new_obs

			total_timesteps += 1
			timesteps_since_eval += 1

	# Final evaluation
	# evaluations.append(evaluate_policy(policy))
	# np.save("./results/%s" % (file_name), evaluations)
