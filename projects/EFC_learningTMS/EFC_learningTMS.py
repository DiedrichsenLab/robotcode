import time
import sys
import os
import shutil
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWidgets import QGraphicsRectItem
import numpy as np
from PyQt5.QtWidgets import (
    QVBoxLayout, QHBoxLayout, QWidget, QLabel, QPushButton, QFileDialog, QFormLayout,
    QMessageBox
)
import pyqtgraph as pg
import pandas as pd
import serial
import serial.tools.list_ports

from HummingbirdHardware import HummingbirdHardware

def setup_arduino(baudrate=500000):
    our_port = None
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if ("Arduino" in port.description or "Arduino" in str(port.manufacturer)):
            our_port = port.device

    # connect to arduino:
    if not our_port:
        print("PORT NO FOUND")
        return False

    try:
        ser = serial.Serial(our_port, baudrate, timeout=1)
        time.sleep(2)
        ser.reset_input_buffer()
        print("ARDUINO CONNECTED SUCCESSFULLY")
        return ser
    except serial.SerialException as e:
        print("Error: Could not connect to {our_port}. {e}")
        return False


class ControlWindow(QtWidgets.QMainWindow):
    def __init__(self, main_window, *args, **kwargs):
        super(ControlWindow, self).__init__(*args, **kwargs)
        self.setWindowTitle("Hummingbird Controls")
        self.main_window = main_window
        
        # Create central widget and layout
        self._control_widget = QWidget()
        self._control_layout = QFormLayout()
        self._control_widget.setLayout(self._control_layout)
        self.setCentralWidget(self._control_widget)
        
        # Add control buttons
        self._load_csv_button = QPushButton("Load Trials")
        self._load_csv_button.clicked.connect(self.main_window._load_csv_file)
        self._control_layout.addRow(QLabel("Load Trials:"), self._load_csv_button)
        
        self._streaming_button = QPushButton("Start Streaming")
        self._streaming_button.clicked.connect(self.main_window._toggle_streaming)
        self._control_layout.addRow(QLabel("Streaming:"), self._streaming_button)
        
        self._start_trial_button = QPushButton("Start Trial")
        self._start_trial_button.clicked.connect(lambda: self.main_window._start_trial(show_prepare_dialog=True))
        self._control_layout.addRow(QLabel("Start Trial:"), self._start_trial_button)
        
        self._zero_button = QPushButton("Zero Sensors")
        self._zero_button.clicked.connect(self.main_window._zero_sensors)
        self._control_layout.addRow(QLabel("Zero Sensors:"), self._zero_button)
        
        # Add navigation buttons
        nav_layout = QHBoxLayout()
        self._prev_trial_button = QPushButton("← Previous Trial")
        self._next_trial_button = QPushButton("Next Trial →")
        self._prev_trial_button.clicked.connect(self._previous_trial)
        self._next_trial_button.clicked.connect(self._next_trial)
        nav_layout.addWidget(self._prev_trial_button)
        nav_layout.addWidget(self._next_trial_button)
        self._control_layout.addRow(QLabel("Navigate:"), nav_layout)
        
        # Add status labels
        self._recording_status_label = QLabel("Recording: Not Started")
        self._recording_status_label.setAlignment(QtCore.Qt.AlignCenter)
        self._recording_status_label.setStyleSheet("font-size: 12px;")
        self._control_layout.addRow(self._recording_status_label)
        
        self._condition_label = QLabel("No trial loaded")
        self._condition_label.setAlignment(QtCore.Qt.AlignCenter)
        self._condition_label.setStyleSheet("font-size: 14px; font-weight: bold;")
        self._control_layout.addRow(self._condition_label)
        
        # Set window size
        self.resize(300, 280)  # Increased height to accommodate new buttons
        
        # Initialize navigation buttons state
        self._update_navigation_buttons()

    def _previous_trial(self):
        """Navigate to the previous trial in block-based structure"""
        if self.main_window._current_trial_in_block_index > 0:
            self.main_window._current_trial_in_block_index -= 1
            self.main_window._select_target_set(None)
            self.main_window._sync_target_set_index()
        elif self.main_window._current_block_index > 0:
            self.main_window._current_block_index -= 1
            self.main_window._current_trial_in_block_index = len(self.main_window._blocks[self.main_window._current_block_index]) - 1
            self.main_window._select_target_set(None)
            self.main_window._sync_target_set_index()
        self._update_navigation_buttons()

    def _next_trial(self):
        """Navigate to the next trial in block-based structure"""
        block = self.main_window._blocks[self.main_window._current_block_index]
        if self.main_window._current_trial_in_block_index < len(block) - 1:
            self.main_window._current_trial_in_block_index += 1
            self.main_window._select_target_set(None)
            self.main_window._sync_target_set_index()
        elif self.main_window._current_block_index < len(self.main_window._blocks) - 1:
            self.main_window._current_block_index += 1
            self.main_window._current_trial_in_block_index = 0
            self.main_window._select_target_set(None)
            self.main_window._sync_target_set_index()
        self._update_navigation_buttons()
    
    def _update_navigation_buttons(self):
        """Update the state of navigation buttons based on current trial position"""
        has_trials = hasattr(self.main_window, '_target_sets') and len(self.main_window._target_sets) > 0
        self._prev_trial_button.setEnabled(has_trials and self.main_window._current_target_set_index > 0)
        self._next_trial_button.setEnabled(has_trials and self.main_window._current_target_set_index < len(self.main_window._target_sets) - 1)

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        self.setWindowTitle("Hummingbird Chord Visualization")
        
        # Trial timing
        self.trial_start_time = None
        self.trial_end_time = None

        self._main_widget = QWidget()
        self._main_layout = QVBoxLayout()
        self._main_widget.setLayout(self._main_layout)
        self.setCentralWidget(self._main_widget)
        
        # Initialize plot-related lists
        self._force_lines = []  # Line segments showing current force
        self._hold_zones = []  # Rectangles for hold zones (centered at zero)
        self._target_zones = []  # Rectangles for target zones (centered at target)
        
        # Setup GUI components
        self._setup_graphs()
        self._main_layout.addWidget(self._graph_widget)
        
        # State variables
        self._target_sets = []
        self._current_target_set_index = 0
        self._trial_to_row_index = {}  # Maps (Participant, Trial) to row index in output dataframe
        self._current_state_durations = []
        self._actual_prep_duration = 0  # Actual prep duration used (includes 2s extension for TMS)
        self._trial_start_time = 0
        self._is_streaming = False
        self._trial_streaming_active = False  # Track if current streaming is for a trial
        self._device = None
        
        # Trial success tracking
        self._current_trial_success = True
        self._match_start_time = 0
        self._match_duration = 0
        
        # Force buffers for each finger
        self._force_buffers = [[] for _ in range(5)]
        self._buffer_size = 10
        
        # CSV data storage
        self._csv_file_path = None  # Input file path (read-only)
        self._output_csv_file_path = None  # Output file path (with Success column)
        self._targets_df = None
        self._output_df = None  # Output dataframe (with Success column)
        
        # Point tracking
        self._block_points = 0  # Points for current block
        
        # Timer for real-time updates
        self.timer = QtCore.QTimer()
        self.timer.setInterval(20)
        self.timer.timeout.connect(self._update_plots)
        self.timer.start()

        #Setup arduino
        self.ser = setup_arduino()

        self._last_state = None
        self._sent_arduino_on_this_trial = False
        
        # Create control window
        self.control_window = ControlWindow(self)
        self.control_window.show()

        self.setFixedSize(1000, 800)

    def _setup_graphs(self):
        self._graph_widget = QWidget()
        layout = QVBoxLayout()
        self._graph_widget.setLayout(layout)
        
        # Create top row with countdown and points
        top_row = QWidget()
        top_row_layout = QHBoxLayout()
        top_row_layout.setContentsMargins(0, 0, 0, 0)
        top_row.setLayout(top_row_layout)
        
        # Add countdown label (centered)
        self._countdown_label = QLabel()
        self._countdown_label.setAlignment(QtCore.Qt.AlignCenter)
        self._countdown_label.setStyleSheet("""
            QLabel {
                font-size: 22px;
                font-weight: bold;
                margin-bottom: 10px;
            }
        """)
        top_row_layout.addWidget(self._countdown_label)
        
        # Add points label (right-aligned)
        self._points_label = QLabel("Points: 0")
        self._points_label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        self._points_label.setStyleSheet("""
            QLabel {
                font-size: 18px;
                font-weight: bold;
                margin-bottom: 10px;
                padding-right: 20px;
            }
        """)
        top_row_layout.addWidget(self._points_label)
        
        layout.addWidget(top_row)
        
        # Create single plot for all fingers
        plot_widget = QWidget()
        plot_layout = QVBoxLayout()
        plot_widget.setLayout(plot_layout)
        
        # Create force plot
        self._force_plot = pg.PlotWidget()
        self._force_plot.setBackground('w')
        self._force_plot.getAxis('left').setPen('k')
        self._force_plot.getAxis('bottom').setPen('k')
        self._force_plot.setXRange(-1, 5)
        self._force_plot.setYRange(-1.5, 1.5)
        
        # Add zero line
        zero_line = pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen('k', width=1, style=QtCore.Qt.DashLine))
        self._force_plot.addItem(zero_line)
        
        # Create force lines, hold zones, and target zones
        self._force_lines = []
        self._target_lines = []
        self._hold_zones = []
        self._target_zones = []
        colors = ['r', 'g', 'b', 'c', 'm']
        finger_names = ["Thumb", "Index", "Middle", "Ring", "Little"]
        line_width = 0.8  # Width of the line segments
        tolerance = 0.4  # Tolerance for zones
        
        for i, (name, color) in enumerate(zip(finger_names, colors)):
            # Create hold zone rectangle (centered at zero, ±tolerance)
            # Use PyQt5's QGraphicsRectItem directly
            hold_zone = QGraphicsRectItem(
                QtCore.QRectF(i - line_width/2, -tolerance, line_width, 2 * tolerance)
            )
            hold_zone.setPen(pg.mkPen('gray', width=1, style=QtCore.Qt.DashLine))
            hold_zone.setBrush(pg.mkBrush('lightgray', alpha=100))
            hold_zone.setVisible(True)  # Always visible
            self._force_plot.addItem(hold_zone)
            self._hold_zones.append(hold_zone)
            
            # Create target zone rectangle (will be positioned at target, ±tolerance)
            target_zone = QGraphicsRectItem(
                QtCore.QRectF(i - line_width/2, -tolerance, line_width, 2 * tolerance)
            )
            target_zone.setPen(pg.mkPen('lightgreen', width=1, style=QtCore.Qt.DashLine))
            target_zone.setBrush(pg.mkBrush('lightgreen', alpha=100))
            target_zone.setVisible(False)  # Initially hidden
            self._force_plot.addItem(target_zone)
            self._target_zones.append(target_zone)
            
            # Create target line segment (center of target zone)
            target_line = pg.PlotDataItem(
                x=[i - line_width/2, i + line_width/2],
                y=[0, 0],
                pen=pg.mkPen('r', width=2)
            )
            target_line.setVisible(False)  # Initially hidden
            self._force_plot.addItem(target_line)
            self._target_lines.append(target_line)
            
            # Create force line segment (will move up/down with force)
            force_line = pg.PlotDataItem(
                x=[i - line_width/2, i + line_width/2],
                y=[0, 0],
                pen=pg.mkPen('r', width=5)
            )
            force_line.setVisible(True)  # Always visible when streaming
            self._force_plot.addItem(force_line)
            self._force_lines.append(force_line)
            
            # Add finger name label
            text = pg.TextItem(text=name, color=color)
            text.setPos(i, -4.5)
            self._force_plot.addItem(text)
        
        plot_layout.addWidget(self._force_plot)
        layout.addWidget(plot_widget)

    def _update_plots(self):
        if self._is_streaming and self._device:
            line_width = 0.8  # Width of the line segments
            
            # Always update force lines when streaming
            for i, finger in enumerate(["TF", "IF", "MF", "RF", "LF"]):
                try:
                    # Get force components
                    fx = self._device.finger_data[i]["force"][0] / 1000.0
                    fy = self._device.finger_data[i]["force"][1] / 1000.0
                    fz = self._device.finger_data[i]["force"][2] / 1000.0
                    
                    # Calculate force magnitude and direction
                    if i == 0:  # Thumb uses X force for flexion/extension
                        current_force = fx  # Use X force for thumb
                    else:  # Other fingers use Y force
                        current_force = -fy  # Negative because of coordinate system
                    
                    # Update force buffer
                    self._force_buffers[i].append(current_force)
                    if len(self._force_buffers[i]) > self._buffer_size:
                        self._force_buffers[i].pop(0)
                    smoothed_force = np.mean(self._force_buffers[i])
                    
                    # Update force line segment
                    # Reverse sign for visualization: extension positive, flexion negative
                    if i == 0:
                        vis_force = -smoothed_force  # Thumb: reverse sign
                    else:
                        vis_force = -smoothed_force  # Fingers: reverse sign (since original is -fy)
                    
                    # Update force line position
                    self._force_lines[i].setData(
                        x=[i - line_width/2, i + line_width/2],
                        y=[vis_force, vis_force]
                    )
                    # Ensure force lines are visible
                    self._force_lines[i].setVisible(True)
                except Exception as e:
                    pass
            
            # Update countdown display (shows "Streaming..." if not in trial)
            if not self._target_sets:
                self._update_countdown(None, 0)
            
            # Only do trial-specific logic if we have a trial running AND trial was explicitly started
            if self._target_sets and self._trial_start_time != 0:
                try:
                    current_trial = self._target_sets[self._current_target_set_index]
                    all_fingers_valid = True
                    
                    # Calculate current state based on elapsed time
                    current_time = time.time()
                    elapsed = current_time - self._trial_start_time
                    
                    # State transition logic - handle TMS trials differently
                    is_tms = current_trial.get('Type', '') == 'TMS'
                    prep_dur = self._current_state_durations[0]
                    
                    if is_tms:
                        # TMS trials: extend prep by 2 seconds, skip execution phase
                        prep_dur_extended = prep_dur + 4.0
                        current_state = None
                        if elapsed < prep_dur_extended:
                            current_state = 0  # Prep phase (extended)
                        else:
                            # Prep phase complete - trial ends (no execution phase)
                            current_state = 2  # Prep phase expired (trial complete)
                        
                        # Send Arduino message at end of normal prep period (before 2-second extension)
                        if current_state == 0 and elapsed >= prep_dur and not self._sent_arduino_on_this_trial:
                            if self.ser:
                                self._sent_arduino_on_this_trial = True
                                command_byte = b'\xAA'
                                self.ser.write(command_byte)
                    else:
                        # Normal trials: prep then execution
                        ex_dur = self._current_state_durations[1]
                        current_state = None
                        if elapsed < prep_dur:
                            current_state = 0  # Prep phase
                        elif elapsed < prep_dur + ex_dur:
                            current_state = 1  # Execution phase
                        else:
                            current_state = 2  # Execution phase expired

                    self._last_state = current_state
                    
                    # Update visibility and color of target lines and zones based on trial phase
                    for i in range(len(self._force_lines)):
                        # Target lines and zones are visible during preparation and execution phases
                        self._target_lines[i].setVisible(current_state >= 0)
                        self._target_zones[i].setVisible(current_state >= 0)
                        
                        # Change color based on phase: red during prep (state 0), green during execution (state 1)
                        # For TMS trials, always red (no execution phase)
                        if current_state == 0 or is_tms:  # Prep phase (or TMS trial - always prep)
                            self._target_zones[i].setPen(pg.mkPen('red', width=1, style=QtCore.Qt.DashLine))
                            self._target_zones[i].setBrush(pg.mkBrush('red', alpha=100))
                        elif current_state == 1:  # Execution phase (non-TMS only)
                            self._target_zones[i].setPen(pg.mkPen('lightgreen', width=1, style=QtCore.Qt.DashLine))
                            self._target_zones[i].setBrush(pg.mkBrush('lightgreen', alpha=100))
                    
                    # Validate forces during execution phase
                    for i, finger in enumerate(["TF", "IF", "MF", "RF", "LF"]):
                        try:
                            # Get smoothed force from buffer
                            smoothed_force = np.mean(self._force_buffers[i]) if self._force_buffers[i] else 0
                            target_force = current_trial["target_forces"][finger]
                            
                            # Only validate forces during execution phase
                            # Only validate fingers that have non-zero targets
                            if current_state == 1 and abs(target_force) > 0.01:  # Only check fingers with non-zero targets
                                # Check direction using smoothed force
                                if (smoothed_force * target_force) < 0:
                                    all_fingers_valid = False
                                
                                # Check magnitude (fixed 0.4N tolerance)
                                tolerance = 0.4
                                if not (target_force - tolerance <= smoothed_force <= target_force + tolerance):
                                    all_fingers_valid = False
                            elif current_state == 1 and abs(target_force) <= 0.01:
                                # For fingers with zero targets, check that they're within tolerance of zero
                                tolerance = 0.4
                                if abs(smoothed_force) > tolerance:
                                    all_fingers_valid = False
                        except Exception as e:
                            all_fingers_valid = False
                    
                    # Only update countdown if we're still streaming
                    if self._is_streaming:
                        self._update_countdown(current_state, elapsed)
                    
                    # Update trial success based on current state
                    if current_state == 0:  # Prep phase
                        # Just show force lines, no validation needed
                        pass
                    elif current_state == 2 and is_tms:  # TMS trial: prep phase complete
                        # TMS trials complete when prep phase ends (no execution phase)
                        self._current_trial_success = True
                        self._handle_trial_success()
                        return
                    elif current_state == 1:  # Execution phase (non-TMS trials only)
                        # Calculate elapsed time in execution phase (elapsed time minus prep duration)
                        prep_dur = self._current_state_durations[0]
                        elapsed_ex_time = max(0, elapsed - prep_dur)  # Ensure non-negative
                        
                        # Check if we've exceeded execution time
                        if elapsed_ex_time >= self._current_state_durations[1]:
                            # Trial failed - execution time expired without meeting match duration requirement
                            self._current_trial_success = False
                            self._handle_trial_failure()
                            return
                        
                        # During execution phase, check if forces are valid
                        if all_fingers_valid:
                            if self._match_start_time == 0:
                                self._match_start_time = time.time()
                            self._match_duration = time.time() - self._match_start_time
                            match_dur_required = self._current_state_durations[2]
                            self._current_trial_success = True
                            # If match duration requirement is met, trial succeeds immediately
                            if self._match_duration >= match_dur_required:
                                self._handle_trial_success()
                                return
                        else:
                            # Reset match duration if forces become invalid
                            self._match_start_time = 0
                            self._match_duration = 0
                            self._current_trial_success = False
                    elif current_state >= 2:  # Execution phase expired
                        # Trial failed because execution time expired without meeting match duration requirement
                        self._current_trial_success = False
                        self._handle_trial_failure()
                        return
                except Exception as e:
                    # If there's an error, stop the trial
                    self._current_trial_success = False
                    self._handle_trial_failure()
                    return

    def _handle_trial_failure(self):
        """Handle trial failure and move to next trial (no retries)"""
        # Show point message (+0)
        self._countdown_label.setText("+0")
        self._countdown_label.setStyleSheet("color: red; font-size: 48px; font-weight: bold;")
        
        # Update points (no change for failure)
        self._update_points_display()
        
        # Stop streaming to save hardware file (trial end)
        self._stop_streaming(is_trial_end=True)
        
        # Reset trial timing
        self._trial_start_time = 0
        self._match_start_time = 0
        self._match_duration = 0
        
        # Hide target lines and zones when trial ends
        for i in range(len(self._force_lines)):
            self._target_lines[i].setVisible(False)
            self._target_zones[i].setVisible(False)
        
        current_trial = self._targets_df.iloc[self._current_target_set_index]
        
        # Get participant number and create folder structure
        participant = self._target_sets[self._current_target_set_index]['Participant']
        participant_folder = f"p{participant}"
        raw_data_folder = os.path.join(participant_folder, "raw_data")
        plotted_data_folder = os.path.join(participant_folder, "plotted_data")
        
        # Create folders if they don't exist
        os.makedirs(raw_data_folder, exist_ok=True)
        os.makedirs(plotted_data_folder, exist_ok=True)
        
        # Find the most recent CSV file in temp_data
        temp_data_folder = "temp_data"
        if os.path.exists(temp_data_folder):
            csv_files = [f for f in os.listdir(temp_data_folder) if f.endswith('.csv')]
            if csv_files:
                # Sort by modification time, most recent first
                csv_files.sort(key=lambda x: os.path.getmtime(os.path.join(temp_data_folder, x)), reverse=True)
                temp_file = os.path.join(temp_data_folder, csv_files[0])
                
                # Create new filename based on participant and trial number
                trial = self._target_sets[self._current_target_set_index]['Trial']
                new_filename = f"p{participant}_trial{trial}.csv"
                new_filepath = os.path.join(raw_data_folder, new_filename)
                
                # Move and rename the file
                try:
                    shutil.move(temp_file, new_filepath)
                except Exception as e:
                    pass
        
        # Store trial data
        trial_data = {
            'trial_number': self._target_sets[self._current_target_set_index]['Trial'],
            'participant': participant,
            'chord': self._target_sets[self._current_target_set_index]['Chord'],
            'rep': self._target_sets[self._current_target_set_index]['Rep'],
            'type': self._target_sets[self._current_target_set_index]['Type'],
            'attempt': self._target_sets[self._current_target_set_index].get('Attempt', ''),
            'success': 'fail',
            'start_time': self.trial_start_time,
            'end_time': self.trial_end_time,
            'durations': {
                'prep': self._actual_prep_duration,  # Actual prep duration (includes 2s extension for TMS)
                'execution': self._current_state_durations[1],
                'match': self._current_state_durations[2]
            },
            'target_forces': self._target_sets[self._current_target_set_index]['target_forces']
        }
        
        # Save trial data to CSV in plotted_data folder
        trial_data_file = os.path.join(plotted_data_folder, f"trial_data_p{participant}.csv")
        trial_data_df = pd.DataFrame([trial_data])
        if os.path.exists(trial_data_file):
            trial_data_df.to_csv(trial_data_file, mode='a', header=False, index=False)
        else:
            trial_data_df.to_csv(trial_data_file, index=False)
        
        # Mark current trial as failed and move to next trial
        if self._output_df is not None:
            current_trial_data = self._target_sets[self._current_target_set_index]
            row_idx = self._trial_to_row_index.get((current_trial_data["Participant"], current_trial_data["Trial"]))
            if row_idx is not None and row_idx < len(self._output_df):
                self._output_df.loc[row_idx, "Success"] = "fail"
                self._output_df.to_csv(self._output_csv_file_path, index=False)
        
        # Determine if we are at the end of the block
        block = self._blocks[self._current_block_index]
        if self._current_trial_in_block_index + 1 < len(block):
            # Auto-advance to next trial after 2 second delay
            self._current_trial_in_block_index += 1
            self._sync_target_set_index()
            self._select_target_set(None)
            QtCore.QTimer.singleShot(2000, lambda: self._start_trial(show_prepare_dialog=False))  # Show result for 2s, then start next trial
        else:
            # End of block: pause and wait for user to start next block
            self._between_blocks = True
            block_points = self._block_points
            if self._current_block_index + 1 < len(self._blocks):
                self._current_block_index += 1
                self._current_trial_in_block_index = 0
                QMessageBox.information(self, "Block Complete", f"Block complete. Points: {block_points}\n\nPress Start Trial to continue to the next block.")
                # Reset points for next block
                self._block_points = 0
                self._update_points_display()
                self._select_target_set(None)
            else:
                QMessageBox.information(self, "Experiment Complete", f"All blocks and trials are complete.\n\nFinal block points: {block_points}")
                # Reset points
                self._block_points = 0
                self._update_points_display()

    def _handle_trial_success(self):
        """Handle successful trial completion and block-based flow"""
        try:
            # Show point message (+1)
            self._countdown_label.setText("+1")
            self._countdown_label.setStyleSheet("color: green; font-size: 48px; font-weight: bold;")
            
            # Update points
            self._block_points += 1
            self._update_points_display()
        
            # Stop streaming to save hardware file (trial end)
            self._stop_streaming(is_trial_end=True)
            
            # Reset trial timing
            self._trial_start_time = 0
            self._match_start_time = 0
            self._match_duration = 0
            
            # Hide target lines and zones when trial ends
            for i in range(len(self._force_lines)):
                self._target_lines[i].setVisible(False)
                self._target_zones[i].setVisible(False)
            
            # Update CSV success status
            try:
                if self._output_df is not None:
                    current_trial_data = self._target_sets[self._current_target_set_index]
                    row_idx = self._trial_to_row_index.get((current_trial_data["Participant"], current_trial_data["Trial"]))
                    if row_idx is not None and row_idx < len(self._output_df):
                        self._output_df.loc[row_idx, "Success"] = "success"
                        self._output_df.to_csv(self._output_csv_file_path, index=False)
            except Exception as e:
                pass
        
            # Get participant number and create folder structure
            try:
                participant = self._target_sets[self._current_target_set_index]['Participant']
                participant_folder = f"p{participant}"
                raw_data_folder = os.path.join(participant_folder, "raw_data")
                plotted_data_folder = os.path.join(participant_folder, "plotted_data")
        
                # Create folders if they don't exist
                os.makedirs(raw_data_folder, exist_ok=True)
                # os.makedirs(plotted_data_folder, exist_ok=True)
            except Exception as e:
                QMessageBox.warning(self, "Warning", "Could not create data folders. Data may not be saved properly.")
        
            # Find and move the most recent CSV file
            try:
                temp_data_folder = "temp_data"
                if os.path.exists(temp_data_folder):
                    csv_files = [f for f in os.listdir(temp_data_folder) if f.endswith('.csv')]
                    if csv_files:
                        # Sort by modification time, most recent first
                        csv_files.sort(key=lambda x: os.path.getmtime(os.path.join(temp_data_folder, x)), reverse=True)
                        temp_file = os.path.join(temp_data_folder, csv_files[0])
        
                        # Create new filename based on participant and trial number
                        trial = self._target_sets[self._current_target_set_index]['Trial']
                        new_filename = f"p{participant}_trial{trial}.csv"
                        new_filepath = os.path.join(raw_data_folder, new_filename)
        
                        # Move and rename the file
                        shutil.move(temp_file, new_filepath)
            except Exception as e:
                QMessageBox.warning(self, "Warning", "Could not save trial data file. Please check the temp_data folder.")
        
            # Store trial data
            try:
                trial_data = {
                    'trial_number': self._target_sets[self._current_target_set_index]['Trial'],
                    'participant': participant,
                    'chord': self._target_sets[self._current_target_set_index]['Chord'],
                    'rep': self._target_sets[self._current_target_set_index]['Rep'],
                    'type': self._target_sets[self._current_target_set_index]['Type'],
                    'success': 'success',
                    'start_time': self.trial_start_time,
                    'end_time': self.trial_end_time,
                    'durations': {
                        'prep': self._actual_prep_duration,  # Actual prep duration (includes 2s extension for TMS)
                        'execution': self._current_state_durations[1],
                        'match': self._current_state_durations[2]
                    },
                    'target_forces': self._target_sets[self._current_target_set_index]['target_forces']
                }
        
                # Save trial data to CSV in plotted_data folder
                trial_data_file = os.path.join(plotted_data_folder, f"trial_data_p{participant}.csv")
                trial_data_df = pd.DataFrame([trial_data])
                if os.path.exists(trial_data_file):
                    trial_data_df.to_csv(trial_data_file, mode='a', header=False, index=False)
                else:
                    trial_data_df.to_csv(trial_data_file, index=False)
            except Exception as e:
                QMessageBox.warning(self, "Warning", "Could not save trial summary data.")
        
            # Block-based flow: auto-advance within block, pause between blocks
            try:
                block = self._blocks[self._current_block_index]
                if self._current_trial_in_block_index + 1 < len(block):
                    # Auto-advance to next trial after 2 second delay
                    self._current_trial_in_block_index += 1
                    self._sync_target_set_index()
                    self._select_target_set(None)
                    QtCore.QTimer.singleShot(2000, lambda: self._start_trial(show_prepare_dialog=False))  # Show result for 2s, then start next trial
                else:
                    # End of block: pause and wait for user to start next block
                    self._between_blocks = True
                    block_points = self._block_points
                    if self._current_block_index + 1 < len(self._blocks):
                        self._current_block_index += 1
                        self._current_trial_in_block_index = 0
                        QMessageBox.information(self, "Block Complete", f"Block complete. Points: {block_points}\n\nPress Start Trial to continue to the next block.")
                        # Reset points for next block
                        self._block_points = 0
                        self._update_points_display()
                        self._select_target_set(None)
                    else:
                        QMessageBox.information(self, "Experiment Complete", f"All blocks and trials are complete.\n\nFinal block points: {block_points}")
                        # Reset points
                        self._block_points = 0
                        self._update_points_display()
            except Exception as e:
                QMessageBox.warning(self, "Warning", "Trial completed but there was an error moving to the next trial.")
        except Exception as e:
            QMessageBox.warning(self, "Error", "An unexpected error occurred while handling trial success.")

    def _update_countdown(self, current_state, elapsed):
        """Update countdown display - now just hides it"""
        # No countdowns shown - just hide the label
        self._countdown_label.setText("")

    def _toggle_streaming(self):
        """Toggle streaming on/off independently of trials"""
        if not self._is_streaming:
            self._device = HummingbirdHardware("right")
            if not self._device.connect():
                QMessageBox.warning(self, "Error", "Failed to connect to Hummingbird device.")
                return
            if not self._device.start_stream():
                QMessageBox.warning(self, "Error", "Failed to start streaming.")
                self._device = None
                return
            
            # Initialize finger data structure (already done in HummingbirdHardware)
            self._is_streaming = True
            self.control_window._streaming_button.setText("Stop Streaming")
            self._update_recording_status("Streaming")
            
            # Initialize force buffers
            for i in range(len(self._force_buffers)):
                self._force_buffers[i] = []
            
            # Note: _trial_streaming_active is set in _start_trial(), not here
        else:
            # Stop streaming
            self._stop_streaming()

    def _stop_streaming(self, is_trial_end=False):
        """Stop streaming. If is_trial_end is True, this file should be saved/renamed."""
        if self._is_streaming and self._device:
            try:
                self.trial_end_time = time.time()
                try:
                    self._is_streaming = False
                    self._device.stop_stream()
                except Exception as e:
                    pass
                # Hide force lines when streaming stops
                for force_line in self._force_lines:
                    force_line.setVisible(False)
                self.control_window._streaming_button.setText("Start Streaming")
                self._update_recording_status("Stopped")
                
                # If this was not a trial end, delete the file (it's just cleanup)
                if not is_trial_end and not self._trial_streaming_active:
                    # Delete the cleanup file (not part of a trial)
                    temp_data_folder = "temp_data"
                    if os.path.exists(temp_data_folder):
                        csv_files = [f for f in os.listdir(temp_data_folder) if f.endswith('.csv')]
                        if csv_files:
                            # Sort by modification time, most recent first
                            csv_files.sort(key=lambda x: os.path.getmtime(os.path.join(temp_data_folder, x)), reverse=True)
                            cleanup_file = os.path.join(temp_data_folder, csv_files[0])
                            try:
                                os.remove(cleanup_file)
                            except Exception as e:
                                pass
                
                self._trial_streaming_active = False
            except Exception as e:
                pass
            finally:
                self._device = None

    def _zero_sensors(self):
        """Zero sensors. Starts streaming if not already active."""
        # Start streaming if not already active
        if not self._is_streaming:
            self._toggle_streaming()
            if not self._is_streaming:
                # Failed to start streaming
                return
        
        if self._device:
            try:
                self._device.zero()
                # Clear force buffers after zeroing
                for i in range(len(self._force_buffers)):
                    self._force_buffers[i] = [0.0] * self._buffer_size
            except Exception as e:
                QMessageBox.warning(self, "Error", "Failed to zero sensors. Please try again.")

    def _update_points_display(self):
        """Update the points display label"""
        try:
            if hasattr(self, '_points_label'):
                self._points_label.setText(f"Points: {self._block_points}")
        except Exception as e:
            pass

    def _update_recording_status(self, message):
        """Update the recording status label"""
        try:
            if hasattr(self, 'control_window') and hasattr(self.control_window, '_recording_status_label'):
                self.control_window._recording_status_label.setText(f"Recording: {message}")
                try:
                    if "Error" in message or "Failed" in message:
                        self.control_window._recording_status_label.setStyleSheet("QLabel { font-size: 12px; color: red; }")
                    elif "Recording" in message:
                        self.control_window._recording_status_label.setStyleSheet("QLabel { font-size: 12px; color: green; }")
                    else:
                        self.control_window._recording_status_label.setStyleSheet("QLabel { font-size: 12px; }")
                except Exception as style_error:
                    # Fallback to basic styling
                    self.control_window._recording_status_label.setStyleSheet("QLabel { font-size: 12px; }")
        except Exception as e:
            pass

    def _select_target_set(self, index):
        # Overridden to use block/trial indices
        if self._blocks:
            block = self._blocks[self._current_block_index]
            if 0 <= self._current_trial_in_block_index < len(block):
                trial = block[self._current_trial_in_block_index]

                # Update target lines and zones for each finger
                tolerance = 0.4
                line_width = 0.8
                for i, finger in enumerate(["TF", "IF", "MF", "RF", "LF"]):
                    target_force = trial["target_forces"][finger]
                    # Reverse sign to match hardware coordinate system
                    vis_target_force = -target_force
                    
                    # Update target line segment (center of target zone)
                    self._target_lines[i].setData(
                        x=[i - line_width/2, i + line_width/2],
                        y=[vis_target_force, vis_target_force]
                    )
                    self._target_lines[i].setVisible(False)  # Hidden until trial starts
                    
                    # Update target zone rectangle (centered at target, ±tolerance)
                    self._target_zones[i].setRect(
                        QtCore.QRectF(i - line_width/2, vis_target_force - tolerance, line_width, 2 * tolerance)
                    )
                    self._target_zones[i].setVisible(False)  # Hidden until trial starts

                # Update trial info display (show Chord, Rep, Type)
                trial_info = f"Trial {trial['Trial']} | Block {trial['Block']} | Rep {trial['Rep']} | Chord: {trial['Chord']} | Type: {trial['Type']}"
                trial_info += "\n"
                for finger, force in trial["target_forces"].items():
                    if force != 0:
                        direction = "Flexion" if float(force) > 0 else "Extension"
                        trial_info += f"{finger}: {abs(float(force))}N {direction}\n"

                self.control_window._condition_label.setText(trial_info)

                # Reset trial state
                self._current_state_durations = trial["state_durations"]
                # For TMS trials, prep duration is extended by 2 seconds
                is_tms = trial.get('Type', '') == 'TMS'
                if is_tms:
                    self._actual_prep_duration = trial["state_durations"][0] + 2.0
                else:
                    self._actual_prep_duration = trial["state_durations"][0]
                self._trial_start_time = 0
                self._current_trial_success = True
                self._match_start_time = 0
                self._match_duration = 0
                self._sync_target_set_index()

    def _sync_target_set_index(self):
        self._current_target_set_index = sum(len(b) for b in self._blocks[:self._current_block_index]) + self._current_trial_in_block_index

    def closeEvent(self, event):
        """Handle application close event"""
        try:
            if self._is_streaming:
                self._stop_streaming()
            if hasattr(self, 'control_window'):
                try:
                    self.control_window.close()
                except Exception as e:
                    pass
        except Exception as e:
            pass
        finally:
            event.accept()

    def _start_trial(self, show_prepare_dialog=False):
        """Start a trial. Called from button click or auto-advance timers."""
        # Reset trial state
        self._sent_arduino_on_this_trial = False
        self._last_state = None
        
        # Handle button click case (manual start)
        if show_prepare_dialog:
            if self._between_blocks:
                # Start the first trial in the current block
                # Points should already be reset in the end-of-block handler, but ensure it here too
                if self._block_points > 0:
                    self._block_points = 0
                    self._update_points_display()
                self._current_trial_in_block_index = 0
                self._between_blocks = False
                self._select_target_set(None)  # Use current block/trial indices
            
            # Show prepare dialog for manual starts
            if not self._blocks:
                QMessageBox.warning(self, "No Trials", "Please load a trials CSV file first.")
                return
            
            # Stop streaming if already active (cleanup, not trial data)
            if self._is_streaming:
                self._stop_streaming(is_trial_end=False)
            
            self._countdown_label.setStyleSheet("color: black; font-size: 22px; font-weight: bold;")
            self._countdown_label.setText("Please relax your fingers")
            QMessageBox.information(self, "Prepare", "Please relax your fingers and click OK when ready")
        else:
            # Auto-advance case: stop streaming if active (cleanup, not trial data)
            if self._is_streaming:
                self._stop_streaming(is_trial_end=False)
            self._countdown_label.setStyleSheet("color: black; font-size: 22px; font-weight: bold;")
            self._countdown_label.setText("")
        
        # Initialize trial timing and start streaming exactly when prep phase begins
        if self._blocks:
            # Zero sensors before streaming starts (if device is already connected)
            if self._device and not self._is_streaming:
                try:
                    self._device.zero()
                except Exception as e:
                    pass
            
            # Start streaming exactly when prep phase begins (before setting trial_start_time)
            if not self._is_streaming:
                self._toggle_streaming()
                if not self._is_streaming:
                    # Failed to start streaming
                    return
                self._trial_streaming_active = True  # Mark that this streaming is for a trial
            
            # Zero sensors after streaming starts (to ensure we have a connection)
            if self._device:
                try:
                    self._device.zero()
                    # Clear force buffers after zeroing
                    for i in range(len(self._force_buffers)):
                        self._force_buffers[i] = [0.0] * self._buffer_size
                except Exception as e:
                    pass
            
            # Initialize trial timing - this starts the prep phase (streaming should be active)
            self._trial_start_time = time.time()
            self._match_start_time = 0
            self._match_duration = 0

    def _load_csv_file(self):
        file_name, _ = QFileDialog.getOpenFileName(self, "Open CSV File", "", "CSV Files (*.csv)")
        if file_name:
            try:
                df = pd.read_csv(file_name)
                self._csv_file_path = file_name  # Input file (read-only)
                
                # Create output file path (same directory, with '_output' suffix or similar)
                # Extract directory and base filename
                file_dir = os.path.dirname(file_name)
                file_base = os.path.basename(file_name)
                file_name_no_ext, file_ext = os.path.splitext(file_base)
                
                # Create output filename (e.g., participant_1_trials_output.csv)
                output_base = f"{file_name_no_ext}_output{file_ext}"
                self._output_csv_file_path = os.path.join(file_dir, output_base)
                
                # Create output dataframe with Success column
                # If output file exists, load it; otherwise create new one from input
                if os.path.exists(self._output_csv_file_path):
                    output_df_loaded = pd.read_csv(self._output_csv_file_path)
                    # Verify it has the same number of rows
                    if len(output_df_loaded) == len(df):
                        self._output_df = output_df_loaded
                        # Ensure Success column exists
                        if "Success" not in self._output_df.columns:
                            self._output_df["Success"] = ""
                    else:
                        # Row count mismatch - create new output from input
                        self._output_df = df.copy()
                        self._output_df["Success"] = ""
                else:
                    # Create output dataframe from input, add Success column
                    self._output_df = df.copy()
                    self._output_df["Success"] = ""
                
                # Save output file (create or update)
                self._output_df.to_csv(self._output_csv_file_path, index=False)
                
                # Use input dataframe for target sets (no Success column)
                self._targets_df = df.copy()
                self._target_sets = []
                # --- Block grouping ---
                self._blocks = []
                # Create a mapping from trial number to original row index for output dataframe updates
                grouped = df.groupby("Block")
                self._trial_to_row_index = {}  # Maps (Participant, Trial) to row index
                for idx, row in df.iterrows():
                    self._trial_to_row_index[(row["Participant"], row["Trial"])] = idx
                
                for block_num, block_df in grouped:
                    block_trials = []
                    for idx, row in block_df.iterrows():
                        # Get Success value from output dataframe using original row index
                        orig_idx = self._trial_to_row_index.get((row["Participant"], row["Trial"]), idx)
                        success_val = self._output_df.loc[orig_idx, "Success"] if orig_idx < len(self._output_df) else ""
                        block_trials.append({
                            "Trial": row["Trial"],
                            "Participant": row["Participant"],
                            "Chord": row["Chord"],
                            "Block": row["Block"],
                            "Rep": row["Rep"],
                            "Type": row["Type"],
                            "target_forces": {
                                "TF": float(row["TF"]),
                                "IF": float(row["IF"]),
                                "MF": float(row["MF"]),
                                "RF": float(row["RF"]),
                                "LF": float(row["LF"])
                            },
                            "state_durations": [
                                float(row["Prep_dur"]),
                                float(row["Ex_dur"]),
                                float(row["Match_dur"])
                            ],
                            "Success": success_val
                        })
                    self._blocks.append(block_trials)
                # Flatten for navigation compatibility
                self._target_sets = [trial for block in self._blocks for trial in block]
                self._current_block_index = 0
                self._current_trial_in_block_index = 0
                self._between_blocks = True
                # Reset points when loading new CSV
                self._block_points = 0
                self._update_points_display()
                self._select_target_set(None)
                self.control_window._update_navigation_buttons()
            except Exception as e:
                QMessageBox.warning(self, "Error", f"Error loading CSV file: {e}")

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec_()) 