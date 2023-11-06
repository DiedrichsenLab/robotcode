import pandas as pd
import matplotlib.pyplot as plt
import os
import numpy as np

# Parameters
nBlock = 5  # number of blocks in experiment
cues = [39, 93, 12, 21, 44]  # probability cues
directory = 'target/'

experiment = 'smp0'  # experiment id
subNum = 100  # subject number

# Prepare the plot
fig, axes = plt.subplots(len(cues), 1, figsize=(10, 8), sharex=True)
fig.suptitle('Distribution of planTime for Each Cue Across Blocks')

# Initialize variables to determine global min and max planTime
global_min_planTime = np.inf
global_max_planTime = -np.inf

# Collect global min and max planTime
for block in range(nBlock):
    for cue in cues:
        filename = f"{directory}{experiment}_block{block:02}_subj{subNum:02}.tgt"
        if os.path.exists(filename):
            df = pd.read_csv(filename, sep='\t')
            cue_plan_times = df[df['cueID'] == cue]['planTime']
            global_min_planTime = min(global_min_planTime, cue_plan_times.min())
            global_max_planTime = max(global_max_planTime, cue_plan_times.max())

# Define the limits for x-axis
xlims = (global_min_planTime, global_max_planTime)

# Read the data and plot
for i, cue in enumerate(cues):
    # Initialize a list to hold planTime data for the cue across all blocks
    plan_times = []

    # Loop through each block and collect planTime data
    for block in range(nBlock):
        filename = f"{directory}{experiment}_block{block:02}_subj{subNum:02}.tgt"
        if os.path.exists(filename):
            df = pd.read_csv(filename, sep='\t')
            # Filter the dataframe for the current cue and collect the planTimes
            plan_times.extend(df[df['cueID'] == cue]['planTime'].tolist())
        else:
            print(f"File not found: {filename}")
            continue

    # Plot the distribution of planTimes for the current cue
    axes[i].hist(plan_times, bins=30, edgecolor='black')
    axes[i].set_title(f'Cue {cue}')
    axes[i].set_ylabel('Frequency')
    axes[i].set_xlabel('planTime (ms)')
    axes[i].set_xlim(xlims)

    # Draw a red vertical line indicating the mean planTime
    mean_planTime = np.mean(plan_times)
    axes[i].axvline(mean_planTime, color='red', linestyle='dashed', linewidth=2)

# Adjust layout
plt.tight_layout(rect=[0, 0.03, 1, 0.95])

# Show the plot
plt.show()
