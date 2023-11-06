import pandas as pd
import os

# Parameters
nBlock = 5  # number of blocks in experiment
cues = [39, 93, 12, 21, 44]  # probability cues
subNum = 100  # subject number
directory = 'target/'
experiment = 'smp0'  # experiment id

# Initialize a dictionary to hold stimFinger counts for each cue
stimFinger_counts = {cue: {} for cue in cues}

# Loop through each block and count stimFinger occurrences for each cue
for block in range(nBlock):
    filename = f"{directory}{experiment}_block{block:02}_subj{subNum:02}.tgt"
    if os.path.exists(filename):
        df = pd.read_csv(filename, sep='\t')
        for cue in cues:
            # Filter the dataframe for the current cue
            cue_data = df[df['cueID'] == cue]
            # Count the occurrences of each stimFinger for the current cue
            for finger_pattern in cue_data['stimFinger']:
                if finger_pattern not in stimFinger_counts[cue]:
                    stimFinger_counts[cue][finger_pattern] = 1
                else:
                    stimFinger_counts[cue][finger_pattern] += 1
    else:
        print(f"File not found: {filename}")

# Calculate probabilities
stimFinger_probabilities = {cue: {} for cue in cues}
for cue, counts in stimFinger_counts.items():
    total = sum(counts.values())
    for finger_pattern, count in counts.items():
        stimFinger_probabilities[cue][finger_pattern] = count / total

# Print the probabilities
for cue, probabilities in stimFinger_probabilities.items():
    print(f"Cue {cue}:")
    for finger_pattern, probability in probabilities.items():
        print(f"  {finger_pattern}: {probability:.2f}")
    print()
