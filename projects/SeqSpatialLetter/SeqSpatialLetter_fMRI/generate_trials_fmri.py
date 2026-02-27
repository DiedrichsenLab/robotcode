#!/usr/bin/env python3

import numpy as np
import random
from collections import Counter
import pandas as pd

# Define the sequences and conditions
sequences = [0, 1, 2, 3]
conditions = [0, 1]
nTr = 65 # number of trials
def generate_random_sequence(numbers_list, seq_rep, seed=None):
    # Seed the random number generator if a seed is provided
    if seed is not None:
        random.seed(seed)

    # Create a sequence by sampling from the numbers list
    random_sequence = []
    for i in range(seq_rep):
        random_sequence.append(random.sample(numbers_list, len(numbers_list)))

    return np.concatenate(random_sequence)

# Define the list of numbers
numbers_list = [1, 2, 3, 4]
#numbers_list = [0, 1]

# Define the desired length of the sequence
sequence_length = 10

# Generate the random sequence
random_sequence = generate_random_sequence(numbers_list, sequence_length)
print(random_sequence)


## Final solution for generating an Euler cycle


def shuffle(array):
    # Function to shuffle elements of an array
    currentIndex = len(array)
    while currentIndex > 0:
        # Pick a random index
        randomIndex = random.randint(0, currentIndex - 1)
        currentIndex -= 1
        # Swap the current element with the random element
        array[currentIndex], array[randomIndex] = array[randomIndex], array[currentIndex]
    return array

def generate_trial_states(init_seq):
    # args- init_seq: int, the initial sequence number

    # Define 8 trials states (sequence ID, Cue type, 0: letter, 1: spatial)
    state_mapping = np.array([[0,0],[0,1],[1,0],[1,1],[2,0],[2,1],[3,0],[3,1]])
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
        trial_states = state_mapping[np.array(seq)]
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


def generate_tgt_file_fmri(subj_id, run_id, trial_states):
    # trial_states[k][0]: sequence ID (1,2,3,4), 
    # trial_states[k][1]: condition (Number cue: 0, Spatial visual cue: 1)
    # motor_sequences = [[3, 2, 4, 5, 2, 1, 5, 1, 4],[3, 5, 1, 3, 2, 4, 5, 2, 1], [1, 3, 2, 1, 2, 4, 5, 1, 3], [1, 2, 4, 3, 2, 4, 3, 5, 1]]
    motor_sequences = [[3, 2, 4, 5, 1],[3, 5, 1, 2, 4], [1, 3, 2, 5, 4], [1, 4, 5, 2, 3]]

    M = np.size(motor_sequences,1) # length of motor sequence
    # Set constant values
    # stim_time_value = 15000
    prep_time_value = 1000
    mov_time_lim_value = 3000
    startTime = 8020 # about 8 sec after a triggering signal
    # feedback_value = 1

    # Generate 27 trials
    data = []
    n_horizon = len(motor_sequences[0])
    if run_id<9:
        nTr = len(trial_states)+3 # number of trials = 65, but will add 3 additional trials due to resting periods
        k = 0
        for m in range(nTr):        
            # Create a row of data
            if (m+1)%17 == 0:  # For the last trial of each block consisting of 16 trials, not the first trial
                iti_value = 16000 # 10 s ITI for baseline activity estimation
                row = [startTime, trial_states[k][1]] + motor_sequences[trial_states[k][0]] + [''.join(map(str, motor_sequences[trial_states[k][0]])), iti_value, n_horizon, prep_time_value, mov_time_lim_value]
                k = k+1
            elif (m+1)%17 == 1 and m!=0:# For the last trial of each block consisting of 16 trials, not the first trial
                iti_value = 1000 # 10 s ITI for baseline activity estimation
                row = [startTime, trial_states[k-1][1]] + motor_sequences[trial_states[k-1][0]] + [''.join(map(str, motor_sequences[trial_states[k-1][0]])), iti_value, n_horizon, prep_time_value, mov_time_lim_value]
            else:
                iti_value = 1000 # 1 s ITI for other trials
                row = [startTime, trial_states[k][1]] + motor_sequences[trial_states[k][0]] + [''.join(map(str, motor_sequences[trial_states[k][0]])), iti_value, n_horizon, prep_time_value, mov_time_lim_value]
                k = k+1
            # Append the row to the data list
            data.append(row)
            startTime = startTime + prep_time_value + mov_time_lim_value + iti_value
            # row = [seq_type, feedback_value] + random_numbers + [''.join(map(str, random_numbers)), iti_value, random_horizons[k], stim_time_value, prep_time_value]
    else:
        nTr = 16
        seqID = [0, 3, 4, 7, 2, 1, 6, 5]*2
        iti_list = [9000, 12000, 15000, 18000, 21000, 24000, 27000, 30000]
        iti_values = generate_random_sequence(iti_list, 2)
        for m in range(nTR):
            row = [startTime, seqID[m]%2] + motor_sequences[int(seqID/2)] + [''.join(map(str, motor_sequences[int(seqID/2)])), iti_values[m], n_horizon, prep_time_value, mov_time_lim_value]
            # Append the row to the data list
            data.append(row)
            startTime = startTime + prep_time_value + mov_time_lim_value + iti_value

    # Create a DataFrame using pandas
    columns = ["startTime", "seqType"] + [f"press{i}" for i in range(1, M+1)] + ["cueP", "iti", "Horizon", "PrepTime", 'MovTimeLim']
    df = pd.DataFrame(data, columns=columns)

    # Save the DataFrame to a CSV file with .tgt extension
    run_id = str(run_id).zfill(2)
    subj_id = str(subj_id).zfill(2)
    filename = f"fmri_ssh{subj_id}_r{run_id}.tgt"
  #  filename = "fmri_ssh00_r00.tgt"
    df.to_csv(filename, sep='\t',  index=False)


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
    generate_tgt_file_fmri(subj_id, run_id, trial_states)
    print(f"Generated trial states sequence: {seq}")
    print(f"Saved to fmri_ssh{subj_id}_r{run_id}.tgt")
