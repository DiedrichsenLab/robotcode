"""Stand-in for the speech classifier, for testing SFOP1 (and SFS1) without it.

Type a digit sequence (e.g. 13524135) and press Enter: the script sends one
onset and one offset per digit to the experiment PC, using the message format
in ../ClassifierProtocol.md. It also prints the trial-end and block-end
messages the experiment sends back.

    python fake_classifier.py --host 127.0.0.1
    python fake_classifier.py --host 169.254.10.2 --ioi 350 --dur 200 --no-index

On the experiment PC, point SFOP1 at this machine with `sylhost <ip> 5006`.
"""
import argparse
import socket
import struct
import threading
import time

MAGIC = 0xAB
OFFSET, ONSET = 0, 1
TRIAL_END, BLOCK_END = 2, 3
TRIAL_REASONS = {0: 'completed', 1: 'timeout', 2: 'space', 3: 'aborted'}
BLOCK_REASONS = {0: 'completed', 1: 'aborted (q)'}
FMT = '<BBBBI'  # magic, type, syllable/reason, index/calib, devTimeMs/number


def now_ms(t0):
    return int((time.monotonic() - t0) * 1000) & 0xFFFFFFFF


def listen(sock):
    while True:
        try:
            data, addr = sock.recvfrom(64)
        except OSError:
            return
        if len(data) != 8 or data[0] != MAGIC:
            print(f'\n[recv] ignored {len(data)} bytes from {addr}')
            continue
        _, typ, reason, calib, number = struct.unpack(FMT, data)
        if typ == TRIAL_END:
            print(f'\n[recv] trial end: trial {number}, {TRIAL_REASONS.get(reason, reason)}, calib={calib}')
        elif typ == BLOCK_END:
            print(f'\n[recv] block end: block {number}, {BLOCK_REASONS.get(reason, reason)}')
        else:
            print(f'\n[recv] type {typ}: {data.hex()}')


def send_sequence(sock, dest, seq, ioi, dur, with_index, t0):
    for k, digit in enumerate(seq, start=1):
        index = k if with_index else 0
        sock.sendto(struct.pack(FMT, MAGIC, ONSET, int(digit), index, now_ms(t0)), dest)
        time.sleep(dur / 1000)
        sock.sendto(struct.pack(FMT, MAGIC, OFFSET, int(digit), index, now_ms(t0)), dest)
        time.sleep(max(ioi - dur, 0) / 1000)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--host', default='127.0.0.1', help='experiment PC IP')
    p.add_argument('--port', type=int, default=5005, help='experiment PC syllable port (sylport)')
    p.add_argument('--listen-port', type=int, default=5006, help='port for trial/block-end messages (sylhost)')
    p.add_argument('--ioi', type=float, default=300, help='onset-to-onset interval, ms')
    p.add_argument('--dur', type=float, default=180, help='syllable duration (onset to offset), ms')
    p.add_argument('--no-index', action='store_true', help='send index 0 (offset pairs with the latest onset)')
    args = p.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', args.listen_port))
    threading.Thread(target=listen, args=(sock,), daemon=True).start()
    dest = (args.host, args.port)
    t0 = time.monotonic()
    print(f'Sending to {dest[0]}:{dest[1]}, listening on {args.listen_port}. Type digits 1-5 and Enter; Ctrl+C quits.')
    try:
        while True:
            seq = input('> ').strip()
            if not seq:
                continue
            if not all(c in '12345' for c in seq):
                print('digits 1-5 only')
                continue
            send_sequence(sock, dest, seq, args.ioi, args.dur, not args.no_index, t0)
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        sock.close()


if __name__ == '__main__':
    main()
