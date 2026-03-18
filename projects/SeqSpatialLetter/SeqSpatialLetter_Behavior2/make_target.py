#!/usr/bin/env python
# coding: utf-8

# In[1]:


from os import makedirs, getcwd
from os.path import exists, join, expanduser
import numpy as np
from collections import Counter
import pandas as pd
import random


# In[2]:


def repeat_shuffle_seq(input_list=[1,2,3,4], repeat=2, seed=None):
	# Seed the random number generator if a seed is provided
	if seed is not None:
		random.seed(seed)

	# Create a sequence by sampling from the input list
	sequence = []
	for i in range(repeat):
		sequence.append(random.sample(input_list, len(input_list)))

	return np.concatenate(sequence)


# In[3]:


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


# In[4]:


def seqID_to_seq(seqID):
	# e.g.) 0 -> [3,2,4,5,1]
	sequences = np.array([
		[3, 2, 4, 5, 1],
		[3, 5, 1, 2, 4],
		[1, 3, 2, 5, 4],
		[1, 4, 5, 2, 3]
	])

	return sequences[seqID]


# In[5]:


def node_to_trialstate(node=[0,0,0,0]):
	# Define 8 trials states (sequence ID, Cue type, 0: letter, 1: spatial)
	list_ts = np.array([[0,0],[0,1],[1,0],[1,1],[2,0],[2,1],[3,0],[3,1]])

	# get the index from ts
	trialstate = list_ts[np.array(node)]
	
	return trialstate


# In[12]:


def generate_circuit(init_node=0):
	# args- init_node: int, the initial node (from 0 to 7)

	# Infinite loop until a valid sequence is generated
	while True: 
		# Initialize an empty sequence
		circuit_node = []
		# Generate a sequence of states
		while len(circuit_node) < 65: # Make sure that length of the sequence is 65
			circuit_node.clear()
			numbers = list(range(8))
			# Generate random paths between states and shuffle them
			paths = [shuffle([[a, b] for b in numbers]) for a in numbers]
			# Choose initial state based on input argument
			if len(circuit_node) == 0:
				currNum = init_node
			else:
				currNum = random.randint(0, 7)
			# Traverse the graph until all states are visited
			while True:
				circuit_node.append(currNum)
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
		circuit_ts = node_to_trialstate(np.array(circuit_node))
		# Generate transitions between states
		list_transition = [(ts, circuit_ts[i+1]) for i, ts in enumerate(circuit_ts[:-1])]
		# Convert transitions to tuples of tuples for uniqueness check
		list_transition = [tuple(map(tuple, transition)) for transition in list_transition]
		# Check the number of unique transitions
		unique_transitions = set(list_transition)
		num_unique_transitions = len(unique_transitions)
		# If there are exactly 64 unique transitions, break the loop and return the sequence
		if num_unique_transitions == 64:
			break
		
	return np.array(circuit_ts), np.array(circuit_node)


# In[7]:


# def find_Euler_circuit(adjcency_matrix=np.ones((8,8)), init_node=0):
# 	# adjcency_matrix: The adjcency matrix of directed and unweighted graph
# 	mat_A = np.copy(adjcency_matrix)

# 	cnt = 0
# 	for ways in mat_A:
# 		if sum(ways)%2!=0:
# 			cnt += 1
#  #	if not cnt==0 or cnt==2: # Eulerian path doesn't exist
# 	if not cnt==0: # Eulerian circuit doesn't exist
# 		return None

# 	stack = [init_node]
# 	circuit = []
# 	while stack:
# 		i = stack[-1]
# 		has_edge = True
# 		for ways in mat_A:
# 			f = np.choice(ways>0)...
# 			if f != None:
# 				stack.append(f)
# 				mat_A[i][f] -= 1
# 				has_edge = False
# 		if not has_edge:
# 			circuit.append(stack.pop())

# 	return np.array(circuit[::-1])


# In[52]:


def check_validation(circuit_node):
	valid = False
	adjcency_matrix = np.zeros((8,8)).astype(int)
	answer = np.ones((8,8)).astype(int)
	for ii, node_i in enumerate(circuit_node[:-1]):
		jj = ii+1
		node_j = circuit_node[jj]
		adjcency_matrix[node_i,node_j] += 1

	if np.array_equal(adjcency_matrix, answer) and (circuit_node[0]==circuit_node[-1]):
		valid = True
		
	return valid


# In[9]:


def generate_tgt_file(circuit_ts, subj_id=1, run_id=1, long_iti=False):
	# circuit_ts[k][0]: sequence ID (1,2,3,4), 
	# circuit_ts[k][1]: condition (Letter cue: 0, Spatial visual cue: 1)
	
	# Initialize
	starttime = 5000
	preptime = 1000
	seq_f = seqID_to_seq(0)
	
	lines = {
		'startTime':[], 'PrepTime':[], 'cueType':[],
		'press1':[], 'press2':[], 'press3':[], 'press4':[], 'press5':[],
		'iti':[]
	}
	for t, ts in enumerate(circuit_ts):
		lines['startTime'].append(starttime)
		lines['PrepTime'].append(preptime)
	
		cueType = ts[1]
		lines['cueType'].append(cueType)
	
		seqID = ts[0]
		seq_i = seqID_to_seq(seqID)
		iti = 1000
		if long_iti:
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
	dir_output = join(getcwd(),'target')
	makedirs(dir_output, exist_ok=True)
	
	fname = join(dir_output,'sslb2_s%02d_r%02d.tgt'%(subj_id,run_id))
	df.to_csv(fname, sep='\t', index=False)
	print('Saved %s'%fname)


# In[ ]:


if __name__ == "__main__":
	# Generate a code line to use subj_id and run_id for input arguments for this script
	# Example: python generate_trials_fmri.py --subj_id 1 --run_id 1
	import argparse
	parser = argparse.ArgumentParser()
	parser.add_argument('--subj_id', type=int, required=False, default=1, help='Subject ID')
	parser.add_argument('--seed', type=int, required=False, default=45, help='The random seed to create initial nodes for each Run')
	args = parser.parse_args()
	subj_id = args.subj_id
	seed = args.seed
	# Generate trial states and save to .tgt file
	# Generate trial states
	# init_seq can be any integer from 0 to 7
	for ii, init_node in enumerate(repeat_shuffle_seq(input_list=[i for i in range(8)], repeat=1, seed=seed)):
		valid = False
		while not valid:
			circuit_ts, circuit_node = generate_circuit(init_node)
			valid = check_validation(circuit_node)
		# Save to .tgt file
		generate_tgt_file(circuit_ts=circuit_ts, subj_id=subj_id, run_id=ii+1)
		print(f"Generated trial states sequence ({len(circuit_node)}) for Run #{ii}: \n{circuit_node}")


# ---

# In[ ]:




