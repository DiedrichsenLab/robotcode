import numpy as np
import pandas as pd


def gen_targets(cues, column_names, nRep, subNum, pt1, pt2, execMaxTime, feedbackTime, iti, fileNameBase='fmri', block=1):
    # function to generate target files for the experiment

    # Initialize the list for stimFinger and trialLabel
    stimFinger = []
    trialLabel = []
    cueID = []

    # Generate stimFinger and trialLabel based on cueID
    for cue in cues:
        if cue == 39:
            cueID.extend(['39'] * nRep)
            stimFinger.extend(['91999'] * nRep)
            trialLabel.extend(['I100R000'] * nRep)
        elif cue == 93:
            cueID.extend(['93'] * nRep)
            stimFinger.extend(['99919'] * nRep)
            trialLabel.extend(['I000R100'] * nRep)
        elif cue == 12:
            cueID.extend(['12'] * nRep)
            stimFinger.extend(['91999'] * (nRep // 4) + ['99919'] * (3 * nRep // 4))
            trialLabel.extend(['I025R075'] * nRep)
        elif cue == 21:
            cueID.extend(['21'] * nRep)
            stimFinger.extend(['99919'] * (nRep // 4) + ['91999'] * (3 * nRep // 4))
            trialLabel.extend(['I075R025'] * nRep)
        elif cue == 44:
            cueID.extend(['44'] * nRep)
            stimFinger.extend(['91999'] * (nRep // 2) + ['99919'] * (nRep // 2))
            trialLabel.extend(['I050R050'] * nRep)

    # Shuffle the stimFinger list while keeping the trialLabel in sync
    combined = list(zip(cueID, stimFinger, trialLabel))
    np.random.shuffle(combined)
    cueID[:], stimFinger[:], trialLabel[:] = zip(*combined)

    # # Repeat and shuffle chords
    # cues = np.repeat(cues, nRep)
    # np.random.shuffle(cues)

    # Generate random planTime values within the range [pt1, pt2]
    planTime = np.random.uniform(pt1, pt2, len(cueID)).round().astype(int)

    # Generate the columns
    col_subNum = subNum * np.ones(len(cueID), dtype=int)
    col_execMaxTime = execMaxTime * np.ones(len(cueID), dtype=int)
    col_feedbackTime = feedbackTime * np.ones(len(cueID), dtype=int)
    col_iti = iti * np.ones(len(cueID), dtype=int)

    # Build the dataframe
    df = pd.DataFrame({
        'subNum': col_subNum,
        'cueID': cueID,
        'stimFinger': stimFinger,
        'planTime': planTime,
        'execMaxTime': col_execMaxTime,
        'feedbackTime': col_feedbackTime,
        'iti': col_iti,
        'trialLabel': trialLabel
    }, columns=column_names)

    # Save the dataframe (fmri_ssl<subj_id>_r<block_num>.tgt)
    fname = fileNameBase + '_' + f"{subNum:02}" + '_' + f"{block + 1:02}" + '.tgt'
    df.to_csv(fname, sep='\t', index=False)

#    return df


# Example usage:

# Chords definition:
cues = [39, 93, 12, 21, 44]  # probability cues

# Params:
nBlock = 10  # number of blocks in experiment
nRep = 4  # number of repetitions of each cue

directory = 'target/'

experiment = 'ssl0'  # experiment id
subNum = 101  # subject number

pt1 = 1500  # minimum time for planning (ms)
pt2 = 2500  # maximum time for planning (ms)
execMaxTime = 3000  # maximum time for execution
feedbackTime = 2000  # time to present feedback
iti = 1000  # inter-trial interval

# column names:
column_names = ['subNum', 'cueID', 'stimFinger', 'planTime', 'execMaxTime', 'feedbackTime', 'iti', 'trialLabel']

# setting the file name base:
fileNameBase = directory + experiment

# generating and saving the target file:
for block in range(nBlock):
    gen_targets(cues, column_names, nRep, subNum, pt1, pt2, execMaxTime, feedbackTime, iti, fileNameBase, block)
