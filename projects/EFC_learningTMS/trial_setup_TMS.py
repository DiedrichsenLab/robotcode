import csv
import random
import sys

# Read chord list from CSV
chord_file = 'chord_list_single_visit.csv'
chords = []
with open(chord_file, newline='') as f:
    reader = csv.reader(f)
    header = next(reader)
    for row in reader:
        chords.append(row)

# Parse learning, control, and washout chords
learning_chords = []  # Each is a list of 5 values (TF, IF, MF, RF, LF)
control_chords = []
washout_chords = []
for row in chords:
    # Learning chords: columns 0-4, Control chords: columns 5-9, Washout: columns 10-14
    if any(cell.strip() for cell in row[0:5]):
        learning_chords.append([row[i].strip() if row[i].strip() else '' for i in range(5)])
    if len(row) > 9 and any(cell.strip() for cell in row[5:10]):
        control_chords.append([row[i].strip() if row[i].strip() else '' for i in range(5, 10)])
    if len(row) > 14 and any(cell.strip() for cell in row[10:15]):
        washout_chords.append([row[i].strip() if row[i].strip() else '' for i in range(10, 15)])

# Remove empty learning/control/washout chords
learning_chords = [chord for chord in learning_chords if any(x != '' for x in chord)]
control_chords = [chord for chord in control_chords if any(x != '' for x in chord)]
washout_chords = [chord for chord in washout_chords if any(x != '' for x in chord)]

# Prompt for participant and durations
participant = input("Enter Participant number: ")
print("\n--- Trial Durations (in seconds) ---")
prep_dur = float(input("Preparation duration: "))
ex_dur = float(input("Execution duration: "))
match_dur = float(input("Matching duration: "))

# Select 3 learning chords (randomly from available)
if len(learning_chords) < 3:
    print("Not enough learning chords in chord_list_single_visit.csv.")
    sys.exit(1)
learning_chord_indices = random.sample(range(len(learning_chords)), 3)
selected_learning_chords = [learning_chords[i] for i in learning_chord_indices]

# Select 2 control chords (randomly from available)
if len(control_chords) < 2:
    print("Not enough control chords in chord_list_single_visit.csv.")
    sys.exit(1)
control_chord_indices = random.sample(range(len(control_chords)), 2)
selected_control_chords = [control_chords[i] for i in control_chord_indices]

# Select 1 washout chord (randomly from available)
if len(washout_chords) < 1:
    print("No washout chord found in chord_list_single_visit.csv.")
    sys.exit(1)
washout_chord = random.choice(washout_chords)

# Prepare all blocks
blocks = []
trial_counter = 2  # Start from 2, as trial 1 will be the zeroing trial

# Add zeroing trial at the start
zero_trial = {
    'Participant': participant,
    'Chord': 'ZERO',
    'TF': -5,
    'IF': -5,
    'MF': -5,
    'RF': -5,
    'LF': -5,
    'Prep_dur': prep_dur,
    'Ex_dur': ex_dur,
    'Match_dur': match_dur,
    'Trial': 1,
    'Block': 0,
    'Rep': 1,
    'Type': 'zero'
}
blocks.append(zero_trial)

# Blocks 1-10: learning + control
for block_num in range(1, 11):
    block_trials = []
    # 10 reps per learning chord per block (for 100 total)
    for chord_idx, chord in enumerate(selected_learning_chords):
        for _ in range(10):
            block_trials.append({
                'Participant': participant,
                'Chord': f'L{chord_idx+1}',
                'TF': chord[0],
                'IF': chord[1],
                'MF': chord[2],
                'RF': chord[3],
                'LF': chord[4],
                'Prep_dur': prep_dur,
                'Ex_dur': ex_dur,
                'Match_dur': match_dur,
                'Trial': None,  # to be filled after shuffling
                'Block': block_num,
                'Rep': None,    # to be assigned after shuffling
                'Type': 'learning'
            })
    # 2 control chords, 1 rep each per block
    for ctrl_idx, chord in enumerate(selected_control_chords):
        block_trials.append({
            'Participant': participant,
            'Chord': f'C{ctrl_idx+1}',
            'TF': chord[0],
            'IF': chord[1],
            'MF': chord[2],
            'RF': chord[3],
            'LF': chord[4],
            'Prep_dur': prep_dur,
            'Ex_dur': ex_dur,
            'Match_dur': match_dur,
            'Trial': None,
            'Block': block_num,
            'Rep': None,
            'Type': 'control'
        })
    # Shuffle block
    random.shuffle(block_trials)
    # Assign Rep numbers in order of appearance for each chord in the block
    rep_counters = {}
    for t in block_trials:
        chord_id = t['Chord']
        rep_counters.setdefault(chord_id, 0)
        rep_counters[chord_id] += 1
        t['Rep'] = rep_counters[chord_id]
    # Assign trial numbers within block
    for t in block_trials:
        t['Trial'] = trial_counter
        trial_counter += 1
    blocks.extend(block_trials)

# Block 11: washout (1 washout chord, 1 rep)
block_num = 11
washout_block = [{
    'Participant': participant,
    'Chord': 'W1',
    'TF': washout_chord[0],
    'IF': washout_chord[1],
    'MF': washout_chord[2],
    'RF': washout_chord[3],
    'LF': washout_chord[4],
    'Prep_dur': prep_dur,
    'Ex_dur': ex_dur,
    'Match_dur': match_dur,
    'Trial': trial_counter,
    'Block': block_num,
    'Rep': 1,
    'Type': 'washout'
}]
blocks.extend(washout_block)

# Write to CSV
filename = f"participant_{participant}_trials.csv"
with open(filename, mode="w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow([
        'Participant', 'Chord', 'TF', 'IF', 'MF', 'RF', 'LF',
        'Prep_dur', 'Ex_dur', 'Match_dur',
        'Trial', 'Block', 'Rep', 'Type', 'Success'
    ])
    for t in blocks:
        writer.writerow([
            t['Participant'], t['Chord'], t['TF'], t['IF'], t['MF'], t['RF'], t['LF'],
            t['Prep_dur'], t['Ex_dur'], t['Match_dur'],
            t['Trial'], t['Block'], t['Rep'], t['Type'], ''
        ])

print(f"\n✅ Trials successfully written to {filename}")