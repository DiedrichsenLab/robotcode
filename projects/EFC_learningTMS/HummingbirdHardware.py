import os
import numpy as np
import time
import threading
import logging
import hid
from datetime import datetime

logging.basicConfig(level=logging.DEBUG)

class HummingbirdHardware:
    def __init__(self, side: str = "left", replay_from_file=None):
        self._hid_device = None
        if side not in ["left", "right"]:
            logging.warning("Unhandled side value, possible values are ['left', 'right']")
            side = "left"
        self._side = side
        self._is_streaming = False
        self._num_fingers = 5
        self._num_dofs = 5
        self.finger_data = []
        for i in range(self._num_fingers):
            self.finger_data.append({"force": [0, 0, 0], "torque": [0, 0]})
        self._thread = None
        self._data = []
        self._replay_file = open(replay_from_file, "rb") if replay_from_file else None
        self._current_state = 0
        self._state_lock = threading.Lock()
        self._current_trial_success = True  # New: Trial success flag
        self._trial_success_lock = threading.Lock()  # New: Lock for success flag

    def connect(self) -> bool:
        vendor_id = 0x483
        product_id = 0x5750 if self._side == "left" else 0x5751
        try:
            self._hid_device = hid.Device(vendor_id, product_id)
            logging.info("Successfully connected to the HID device.")
            return True
        except hid.HIDException as e:
            logging.error(f"Failed to connect to HID device: {e}")
            return False

    def _send_message(self, message: str) -> bool:
        if self._replay_file is not None:
            return True
        if self._hid_device is None:
            return False
        payload = message.encode("ascii")
        data_size = len(payload).to_bytes(2, "big")
        try:
            self._hid_device.write(data_size + payload)
            logging.info(f"Message sent: {message}")
            return True
        except Exception as e:
            logging.error(f"Failed to write message '{message}' to HID device: {e}")
            return False

    def start_stream(self) -> bool:
        if self._is_streaming:
            logging.info("Stream already started.")
            return True
        logging.info("Starting stream...")
        self._is_streaming = True
        self._data = []
        self._thread = threading.Thread(target=self._read_threading)
        self._thread.start()
        success = self._send_message("printmode v")
        if not success:
            logging.error("Failed to send start stream command.")
            self._is_streaming = False
            return False
        logging.info("Stream started successfully.")
        return True

    def stop_stream(self) -> bool:
        if not self._is_streaming:
            return True
        logging.info("Stopping stream...")
        self._is_streaming = False
        self._thread.join()
        self._save_data_to_csv()
        return self._send_message("printmode n")

    def _save_data_to_csv(self):
        if not self._data:
            logging.warning("No data to save.")
            return
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        # Save the file in a temporary folder.
        temp_folder = "temp_data"
        if not os.path.exists(temp_folder):
            os.makedirs(temp_folder)
        filename = os.path.join(temp_folder, f"stream_data_{timestamp}.csv")
        data_array = np.vstack(self._data)
        header = ["timestamp"]
        for i in range(self._num_fingers):
            header.extend([
                f"finger_{i+1}_force_x", f"finger_{i+1}_force_y", f"finger_{i+1}_force_z",
                f"finger_{i+1}_torque_x", f"finger_{i+1}_torque_y"
            ])
        header.append("state")
        header.append("trial_success")
        np.savetxt(filename, data_array, delimiter=",", header=",".join(header), 
                   comments='', fmt='%.6f')
        logging.info(f"Data saved to {filename}")


    def _append_data(self):
        timestamp = time.time()
        row = [timestamp]
        for finger in self.finger_data:
            row.extend(finger["force"] + finger["torque"])
        with self._state_lock:
            row.append(self._current_state)
        # New: Add trial success status
        with self._trial_success_lock:
            row.append(int(self._current_trial_success))
        self._data.append(np.array(row))

    def _read_message(self) -> bytes:
        if self._replay_file is not None:
            time.sleep(0.02)
            header = self._replay_file.read(6)
            canary = [0, 42]
            if list(header[0:2]) != canary:
                logging.warning("Invalid canary value in replay file.")
                return None
            count = int.from_bytes(header[2:6], byteorder="little")
            return self._replay_file.read(count) if count > 0 else None
        else:
            return self._hid_device.read(64) if self._hid_device else None

    def _parse_message(self, message: bytes):
        for finger_idx in range(self._num_fingers):
            raw_data = []
            for dof_idx in range(self._num_dofs):
                data_idx = dof_idx * 2 + finger_idx * self._num_dofs * 2
                raw_data.append(
                    int.from_bytes(
                        message[data_idx:data_idx + 2], byteorder="big", signed=True
                    )
                )
            self.finger_data[finger_idx] = {
                "force": raw_data[:3],
                "torque": raw_data[3:],
            }

    def _read_threading(self):
        while self._is_streaming:
            msg = self._read_message()
            if msg:
                self._parse_message(msg)
                self._append_data()

    def set_current_state(self, state: int):
        """Update the current trial state."""
        with self._state_lock:
            self._current_state = state

    def set_trial_success(self, success: bool):
        """Update trial success status (thread-safe)"""
        with self._trial_success_lock:
            self._current_trial_success = success

    def zero(self) -> bool:
        """Send a 'zero' command to reset sensor readings."""
        if self._hid_device is None:
            logging.error("Cannot zero sensors: Device not connected.")
            return False
        return self._send_message("zero")