#!/usr/bin/env python3

from os import makedirs
from os.path import exists, join
import numpy as np
from collections import Counter
import pandas as pd
import random

def repeat_shuffle_seq(input_list=[1,2,3,4], repeat=2, seed=None):
	# Seed the random number generator if a seed is provided
	if seed is not None:
		random.seed(seed)

	# Create a sequence by sampling from the input list
	sequence = []
	for i in range(repeat):
		sequence.append(random.sample(input_list, len(input_list)))

	return np.concatenate(sequence)

def shuffle(array):
	# ================================ #
	## Fisher–Yates shuffle algorithm ##
	# ================================ #

	# Function to shuffle elements of an array
	currentIndex = len(array)
	while currentIndex > 0:
		# Pick a random index
		randomIndex = random.randint(0, currentIndex-1)
		currentIndex -= 1
		# Swap the current element with the random element
		array[currentIndex], array[randomIndex] = array[randomIndex], array[currentIndex]
	return array

def seqID_to_seq(seqID):
	sequences = np.array([
		[3, 2, 4, 5, 1],
		[3, 5, 1, 2, 4],
		[1, 3, 2, 5, 4],
		[1, 4, 5, 2, 3]
	])

	return sequences[seqID]

def trialstate_to_node(trialstate=(0,0)):
	# Define 8 trials states (sequence ID, Cue type, 0: letter, 1: spatial)
	ts = np.array([[0,0],[0,1],[1,0],[1,1],[2,0],[2,1],[3,0],[3,1]])

	# get the index from ts
	node = ts[np.array(trialstate)]
	
	return node

def generate_trial_states(init_seq=0):
	# args- init_seq: int, the initial node (from 0 to 7)

	# Infinite loop until a valid sequence is generated
	while True: 
		# Initialize an empty sequence
		seq = []
		# Generate a sequence of states
		while len(seq) < 65: # Make sure that length of the sequence is 65
			seq.clear()
			numbers = list(range(8))
			# Generate random paths between states and shuffle them
			paths = [shuffle([[a, b] for b in numbers]) for a in numbers]
			# Choose initial state based on input argument
			if len(seq) == 0:
				currNum = init_seq
			else:
				currNum = random.randint(0, 7)
			# Traverse the graph until all states are visited
			while True:
				seq.append(currNum)
				# If there are no more paths from current state, break the loop
				if len(paths[currNum - 1]) <= 0:
					break
				# Choose a random path from the current state and move to the next state
				path = paths[currNum - 1].pop()
				# If there are more than one path from current state and no path from the next state, continue
				if len(paths[currNum - 1]) >= 1 and len(paths[path[1] - 1]) <= 0:
					continue
				currNum = path[1]
		# Convert the sequence of states into binary sequences
		trial_states = trialstate_to_node(np.array(seq))
		# Generate transitions between states
		trial_transitions = [(trial_states[i], trial_states[i + 1]) for i in range(len(trial_states) - 1)]
		# Convert transitions to tuples of tuples for uniqueness check
		trial_transitions = [tuple(map(tuple, transition)) for transition in trial_transitions]
		# Check the number of unique transitions
		unique_transitions = set(trial_transitions)
		num_unique_transitions = len(unique_transitions)
		# If there are exactly 64 unique transitions, break the loop and return the sequence
		if num_unique_transitions == 64:
			break
		
	return trial_states, seq

def generate_tgt_file(trial_states, subj_id=1, run_id=1):
	# trial_states[k][0]: sequence ID (1,2,3,4), 
	# trial_states[k][1]: condition (Letter cue: 0, Spatial visual cue: 1)

	# Initialize
	starttime = 5000
	preptime = 1000
	seq_f = seqID_to_seq(0)

	lines = {
		'startTime':[], 'PrepTime':[], 'cueType':[],
		'press1':[], 'press2':[], 'press3':[], 'press4':[], 'press5':[],
		'iti':[]
	}
	for t, ts in enumerate(trial_states):
		lines['startTime'].append(starttime)
		lines['PrepTime'].append(preptime)

		cueType = ts[1]
		lines['cueType'].append(cueType)

		seqID = ts[0]
		seq_i = seqID_to_seq(seqID)
		iti = 1000
		if t%17 == 16:	# For the last trial of each block consisting of 16 trials, not the first trial
			iti = 16000
		elif t%17 == 0 and t != 0:# For the last trial of each block consisting of 16 trials, not the first trial
			seq_i = seq_f

		lines['press1'].append(seq_i[0])
		lines['press2'].append(seq_i[1])
		lines['press3'].append(seq_i[2])
		lines['press4'].append(seq_i[3])
		lines['press5'].append(seq_i[4])
		lines['iti'].append(iti)

		starttime = starttime + preptime + 3000 + iti
		seq_f = seq_i

	# Create a DataFrame using pandas
	df = pd.DataFrame(lines)

	# Save the DataFrame to a CSV file with .tgt extension
	dir_output = join('./target')
	makedirs(dir_output, exist_ok=True)

	fname = join(dir_output,'sslb2_s%02d_r%02d.tgt'%(subj_id,run_id))
	df.to_csv(fname, sep='\t', index=False)
	print('Saved %s'%fname)

if __name__ == "__main__":
	# Generate a code line to use subj_id and run_id for input arguments for this script
	# Example: python generate_trials_fmri.py --subj_id 1 --run_id 1
	import argparse
	parser = argparse.ArgumentParser()
	parser.add_argument('--subj_id', type=int, required=False, default=1, help='Subject ID')
	parser.add_argument('--run_id', type=int, required=False, default=1, help='Run ID')
	args = parser.parse_args()
	subj_id = args.subj_id
	run_id = args.run_id	
	# Generate trial states and save to .tgt file
	# Generate trial states
	# init_seq can be any integer from 0 to 7
	init_seq = random.randint(0, 7)
	trial_states, seq = generate_trial_states(init_seq)
	# Save to .tgt file
	generate_tgt_file(trial_states, subj_id, run_id)
	print(f"Generated trial states sequence ({len(seq)}): {seq}")
