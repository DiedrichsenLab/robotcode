# SpeechFingerOnlinePlanning

Online planning of sequences produced with the fingers or by speech. On every trial the participant presses (right hand) or says (speech) a random sequence of 8 digits (1–5). Only the next `windowSize` digits are visible (the horizon); the others are shown as `*`, and each press or syllable onset reveals one more digit.

## SFOP1

Trial timeline:

1. Cues appear with the effector symbol (`#` hand, `$` speech). The first `windowSize` digits are visible (preview, `precueTime`, 1600 ms). Finger forces must stay inside the baseline area during the preview (hand and speech trials).
2. Go tone. Each press (hand) or syllable onset (speech) colours that digit and reveals the next one, so `windowSize` digits ahead stay visible.
3. The sequence ends when all fingers are released after the last press (hand) or at the offset of the last syllable (speech). The last digit's colour stays for 500 ms, then points are shown.
4. Speech trials also end at `execTime` after the go tone or when the experimenter presses space; `e` marks the trial as an error.

Each block uses one effector. Speech blocks start with classifier calibration trials (`isCalib` = 1: whole sequence visible, no points), and the ITI after the last one is at least 5 s.

Speech syllables come from the speech classifier over UDP: see [ClassifierProtocol.md](ClassifierProtocol.md).

Points (non-error, non-calibration trials): +3 if ET is below the 25th percentile, +1 between the 25th and 75th, 0 above. Percentiles are estimated separately for each effector and window size from the correct trials of the most recent block with that effector. Errors get −1, moving before the go cue −5.

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
| fixedDur | 1 for speech (trial times out at `execTime`, finger presses are errors), 0 for hand |
| execTime | max time from go tone, ms (speech) |
| iti | inter-trial interval, ms |
| precueTime | preview duration, ms |
| isCalib | optional; 1 = classifier calibration trial (speech only, no points) |

Target files are made by `TargetFileMaker.ipynb` in the SpeechFingerOnlinePlanning analysis folder.

### Command window

| Command | Effect |
|---------|--------|
| `sylhost <ip> <port>` | classifier address for trial/block-end messages |
| `sylport <port>` | port for syllable events (default 5005) |
| `sylend 1\|0` | speech sequence ends at the last offset (1, default) or last onset (0) |
| `sylcheck 1\|0` | wrong classified syllable is an error (1) or any syllable is accepted (0, default) |
| `Finger_ET_percentile <w> <low> <high>` | set the hand ET thresholds for window size w |
| `Speech_ET_percentile <w> <low> <high>` | set the speech ET thresholds for window size w |

### Data (`.dat`)

`SubNum, group, effector, windowSize, isCalib, cue, press1..8, RT, ET, MT, isError, timingError, points, response1..8, handPressed1..8, pressTime1..8, releaseTime1..8, syllableOnsetDevTime1..8, syllableOffsetDevTime1..8, ETpercentileLow, ETpercentileHigh, startTime, startTimeReal, trialDur, startTR, startTRtime, useMetronome, isCross, crossTime, beepStartReal, beepStopReal`

- Times are ms on the trial clock (from cue onset). `pressTime` is the press or syllable onset, `releaseTime` the release of that key or the syllable offset.
- `RT` is from the go tone to the first press / onset, `MT` from the first press / onset to the end of the sequence, `ET = RT + MT`.
- `timingError` = 1 when a speech trial reached `execTime`; such trials are not errors.
- `ETpercentileLow/High` are the point thresholds used on that trial.

Data go to `C:/data/SpeechFingerOnlinePlanning/SFOP1/`, with one `.wav` per trial.
