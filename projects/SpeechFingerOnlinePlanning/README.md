# SpeechFingerOnlinePlanning

Online planning of sequences produced with the fingers or by speech. On every trial the participant presses (right hand) or says (speech) a random sequence of 8 digits (1–5). Only the next `windowSize` digits are visible (the horizon); the others are shown as `*`, and each press or syllable onset reveals one more digit.

## SFOP1

Trial timeline:

1. Cues appear with the effector symbol (`#` hand, `$` speech). The first `windowSize` digits are visible (preview, `precueTime`, 1600 ms). Finger forces must stay inside the baseline area during the preview (hand and speech trials).
2. Go tone. Each press (hand) or syllable onset (speech) colours that digit and reveals the next one, so `windowSize` digits ahead stay visible.
3. The sequence ends when all fingers are released after the last press (hand) or at the offset of the last syllable (speech). The last digit's colour stays for 500 ms, then points are shown.
4. Speech trials have no time limit when `execTime` is 0. The experimenter ends a trial with space (e.g. when the classifier missed a syllable); `e` marks the trial as an error, and space then ends it.

Each block uses one effector. Speech blocks start with classifier calibration trials (`isCalib` = 1: whole sequence visible, no points), and the ITI after the last one is at least 5 s.

Speech syllables come from the speech classifier over UDP: see [ClassifierProtocol.md](ClassifierProtocol.md).

Points (non-error, non-calibration trials): +3 if ET is below the low threshold, +1 between the thresholds, 0 above. Each effector has its own two thresholds, the 25th and 75th percentiles of ET over the correct trials of a block of that effector, all window sizes pooled. After every block a threshold is replaced only by a faster one, and only if that effector's error rate in the block is below 20% (`ERthreshold`); calibration trials and trials skipped with space are left out. Until an effector has thresholds, its trials show no points. Errors get −1, moving before the go cue −5. Thresholds are saved after every block to `SpeechFingerOnlinePlanning_<subj>_thresholds.txt` in the data folder and loaded again for that subject, so they carry over to a restart and to day 2.

### Target file (`.tgt`)

Tab-separated with a header row, one trial per line:

| Column | Meaning |
|--------|---------|
| subNum | subject number |
| group | group |
| effector | 0 = speech, 2 = right hand |
| symbol | `$` speech, `#` hand |
| windowSize | horizon, 1–8 (8 = whole sequence visible) |
| seq | 8-digit cue |
| fixedDur | 1 for speech (experimenter keys and `execTime` apply, finger presses are errors), 0 for hand |
| execTime | time limit from the go tone for speech, ms; 0 = no limit |
| iti | inter-trial interval, ms |
| precueTime | preview duration, ms |
| isCalib | optional; 1 = classifier calibration trial (speech only, no points) |

Target files are named `sn<subject>_bn<block>.tgt` and are made by `TargetFileMaker.ipynb` in the SpeechFingerOnlinePlanning analysis folder.

### Running a block

```
subj 3
run 4 sn3_bn4.tgt
```

Before a block starts, the program checks:

- the subject number is set with `subj` and is a plain number (`3`, not `03` or `s3`);
- the file name matches the subject and block number (`sn3_bn4.tgt` only runs as block 4 of subject 3); other file names ask for confirmation;
- every trial in the file is for this subject, and the block has one effector (0 or 2);
- the previous block has been run (otherwise it asks for confirmation);
- the block has not been run already (otherwise it asks before overwriting; the `.dat` rows of a re-run block are appended, so the analysis has to drop the earlier run).

The command window then shows the subject, block, effector, number of trials and the current point thresholds.

### Command window

| Command | Effect |
|---------|--------|
| `sylhost <ip> <port>` | classifier address for trial/block-end messages |
| `sylport <port>` | port for syllable events (default 5005) |
| `sylend 1\|0` | speech sequence ends at the last offset (1, default) or last onset (0) |
| `sylcheck 1\|0` | wrong classified syllable is an error (1) or any syllable is accepted (0, default) |
| `Finger_ET_percentile <low> <high>` | set the hand ET thresholds (ms) for the current subject (saved to the thresholds file) |
| `Speech_ET_percentile <low> <high>` | set the speech ET thresholds (ms) for the current subject (saved to the thresholds file) |

### Data (`.dat`)

`SubNum, group, effector, windowSize, isCalib, cue, press1..8, RT, ET, MT, isError, timingError, points, response1..8, handPressed1..8, pressTime1..8, releaseTime1..8, syllableOnsetDevTime1..8, syllableOffsetDevTime1..8, ETpercentileLow, ETpercentileHigh, startTime, startTimeReal, trialDur, startTR, startTRtime, useMetronome, isCross, crossTime, beepStartReal, beepStopReal`

- Times are ms on the trial clock (from cue onset). `pressTime` is the press or syllable onset, `releaseTime` the release of that key or the syllable offset.
- `RT` is from the go tone to the first press / onset, `MT` from the first press / onset to the end of the sequence, `ET = RT + MT`.
- `timingError` = 1 when a speech trial was ended by the experimenter (space) or reached `execTime`. Such trials are not errors, but their ET is not the participant's.
- `ETpercentileLow/High` are the point thresholds used on that trial.

Data go to `C:/data/SpeechFingerOnlinePlanning/SFOP1/`, with one `.wav` per trial.
