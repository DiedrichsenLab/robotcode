# Speech classifier UDP protocol

The speech classifier runs on a separate computer. It tells the experiment (SFOP1, also SFS1) when each syllable starts and ends, and the experiment tells the classifier when each trial and block ends. Code: `include/SyllableReceiver.h`, `source/SyllableReceiver.cpp`.

| Direction | Destination | Default port | Set in the command window |
|-----------|-------------|--------------|---------------------------|
| classifier → experiment | experiment PC | 5005 | `sylport <port>` |
| experiment → classifier | classifier PC | 5006 | `sylhost <ip> <port>` |

Every message is one UDP datagram of 8 bytes, little-endian, Python `struct` format `'<BBBBI'`. Byte 0 is always `0xAB`; datagrams of another size or with another first byte are ignored.

## Classifier → experiment: syllable events

| Byte | Field | Value |
|------|-------|-------|
| 0 | magic | `0xAB` |
| 1 | type | `1` = onset (syllable starts), `0` = offset (syllable ends) |
| 2 | syllable | spoken digit, 1–5 |
| 3 | index | position of the syllable in the trial, 1-based (1 for the first syllable after the go cue). The onset and offset of the same syllable carry the same index. `0` = not given |
| 4–7 | devTimeMs | classifier clock in ms (uint32), written to the `.dat` file only |

Send one onset and one offset for every syllable, each as soon as it is detected:

```
onset  syllable 1:  AB 01 03 01 <t>     # "three" starts
offset syllable 1:  AB 00 03 01 <t>     # "three" ends
onset  syllable 2:  AB 01 05 02 <t>
...
```

```python
import socket, struct
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
EXP = ('169.254.x.x', 5005)                                   # experiment PC
sock.sendto(struct.pack('<BBBBI', 0xAB, 1, digit, k, t_ms), EXP)  # onset of k-th syllable
sock.sendto(struct.pack('<BBBBI', 0xAB, 0, digit, k, t_ms), EXP)  # offset of k-th syllable
```

How the experiment uses them (speech trials only):

- The experiment times every message by its own trial clock on arrival (`pressTime` for onsets, `releaseTime` for offsets in the `.dat` file, same clock as finger presses). `devTimeMs` is stored as `syllableOnsetDevTime` / `syllableOffsetDevTime`.
- Each onset fills the next position of the sequence, in arrival order, and in SFOP1 reveals the next masked digit. Onsets after the last position are ignored.
- An offset with index *k* is stored for position *k*. An offset with index 0 is stored for the most recent onset. Offsets for positions that have not had an onset yet are ignored.
- The trial ends at the offset of the last syllable (SFOP1 default; `sylend 0` ends it at the last onset instead), when the experimenter presses space, or at `execTime` after the go cue if a time limit is set.
- Messages that arrive before the cues of a trial appear are discarded, so the index restarts at 1 every trial. The classifier should reset its count when it receives a trial-end message.

## Experiment → classifier: trial and block end

| Byte | Field | Value |
|------|-------|-------|
| 0 | magic | `0xAB` |
| 1 | type | `2` = trial end, `3` = block end |
| 2 | reason | trial end: `0` completed, `1` timeout (`execTime` reached), `2` experimenter pressed space, `3` aborted (movement before the go cue, or block aborted with `q`). Block end: `0` all trials ran, `1` aborted with `q` |
| 3 | calib | trial end: `1` calibration trial, `0` normal trial. Block end: `0` |
| 4–7 | number | trial end: trial number within the block, 1-based. Block end: block number |

A trial end is sent once per speech trial; hand trials send none. A block end is sent after every block.

## Testing without the classifier

`tools/fake_classifier.py` plays the classifier: type a digit sequence and it sends the onsets and offsets, and it prints the trial- and block-end messages it receives.

```
python tools/fake_classifier.py --host <experiment PC ip> --ioi 300 --dur 180
```
