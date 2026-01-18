from PyQt5.QtCore import QObject, QTimer, pyqtSlot, pyqtSignal
import numpy as np
import time
from scipy.spatial.transform import Rotation as R
import re

# === CONFIGURATION ===
MAIN_LOOP_TIMER_MS = 20 
LINEAR_SPEED_M_S = 0.3  # Base speed at 100%

# === WORKSPACE LIMITS (mm) ===
LIMIT_X = (-500.0, 600.0)
LIMIT_Y = (-550.0, 550.0)
LIMIT_Z = (0.0, 600.0)

try:
    import robot.communication as comm
except ImportError:
    print("WARNING: Cannot import 'robot.communication'. Using mock.")
    class CommMock:
        def open_serial_port(self, p, b=115200): print(f"[MOCK] Open {p}"); return True
        def close_serial_port(self): print("[MOCK] Close")
        def read_line(self, p): return None
        def send_angles(self, p, a): pass
        def send_command(self, p, c): print(f"[MOCK] Cmd: {c}")
    comm = CommMock()

class RobotWorker(QObject):
    # Signals to GUI
    angles_updated = pyqtSignal(np.ndarray)
    status_updated = pyqtSignal(str, str)
    limit_switch_status = pyqtSignal(int, bool)
    estop_status_signal = pyqtSignal(bool) 
    program_finished = pyqtSignal(str, bool)
    program_line_highlight = pyqtSignal(int)
    tool_changed = pyqtSignal(str)

    def __init__(self, initial_angles, kinematics_engine):
        super().__init__()
        self.current_port = None
        self.previous_angles = initial_angles
        self.kinematics = kinematics_engine 
        self.standby_angles = initial_angles
        
        self.is_stop_requested = False
        self.program_queue = []
        self.program_line_index = 0
        self.program_line_map = []
        self.trajectory_queue = [] 
        self.trajectory_index = 0
        self.is_moving = False
        self.is_program_waiting_for_move = False
        
        # Default speed 50%
        self.speed_multiplier = 0.5
        
        self.main_loop_timer = QTimer(self)
        self.main_loop_timer.timeout.connect(self._update_step)
        self.main_loop_timer.start(MAIN_LOOP_TIMER_MS) 
        print(f"[WORKER] Control loop started ({MAIN_LOOP_TIMER_MS}ms).")

    # -----------------------------------------------------------------
    # CONTROL SLOTS
    # -----------------------------------------------------------------

    @pyqtSlot(float)
    def set_speed_multiplier(self, multiplier):
        """Sets speed multiplier (0.1 - 1.0)."""
        self.speed_multiplier = multiplier
        # print(f"[WORKER] Speed: {self.speed_multiplier * 100:.0f}%")

    @pyqtSlot(str)
    def change_active_tool(self, tool_name):
        if self.is_moving:
            print("Cannot change tool during movement!")
            return
        print(f"[WORKER] Changing tool to: {tool_name}")
        self.kinematics.set_tool(tool_name)
        self.tool_changed.emit(tool_name)
        self.angles_updated.emit(self.previous_angles)

    # -----------------------------------------------------------------
    # POSITION VALIDATION (SOFT LIMITS)
    # -----------------------------------------------------------------

    def _check_and_report_error(self, message):
        print(f"[WORKER ERROR] {message}")
        self.status_updated.emit(f"ERROR: {message}", "red")
        if self.current_port:
            comm.send_command(self.current_port, "ERROR_POS\n\r")

    def _validate_cartesian_target(self, target_matrix):
        """Checks if target is safe (Box + Reachable)."""
        pos_m = target_matrix[:3, 3]
        x_u, y_u, z_u = self._robot_coords_to_user(pos_m)
        
        if not (LIMIT_X[0] <= x_u <= LIMIT_X[1]):
            self._check_and_report_error(f"Limit X! ({x_u:.1f})")
            return False
        if not (LIMIT_Y[0] <= y_u <= LIMIT_Y[1]):
            self._check_and_report_error(f"Limit Y! ({y_u:.1f})")
            return False
        if not (LIMIT_Z[0] <= z_u <= LIMIT_Z[1]):
            self._check_and_report_error(f"Limit Z! ({z_u:.1f})")
            return False

        # IK Reachability Check
        start_angles = self.get_previous_angles()
        calculated_angles = self.kinematics.inverse_kinematics(
            target_matrix[:3, 3], target_matrix[:3, :3], start_angles
        )
        check_fk = self.kinematics.forward_kinematics(calculated_angles)
        dist_error = np.linalg.norm(check_fk[:3, 3] - target_matrix[:3, 3])
        
        if dist_error > 0.01: # 1cm tolerance
            self._check_and_report_error("Unreachable Position (IK)")
            return False
            
        return True

    # -----------------------------------------------------------------
    # MOVEMENTS (PTP & LINEAR)
    # -----------------------------------------------------------------

    @pyqtSlot(np.ndarray)
    def start_move(self, target_angles_rad):
        """PTP Move (Joint Space)."""
        if self.is_moving:
            self.status_updated.emit("Error: Moving", "orange")
            return
        
        if self.current_port is None or self.current_port == "No ports":
            self.status_updated.emit("Error: No Port", "red")
            self.previous_angles = target_angles_rad
            self.angles_updated.emit(target_angles_rad)
            return

        try:
            # Validate Cartesian Target
            target_fk = self.kinematics.forward_kinematics(target_angles_rad)
            if not self._validate_cartesian_target(target_fk):
                return
            
            start_angles = self.get_previous_angles()
            joint_traj = self._generate_joint_trajectory_ptp(start_angles, target_angles_rad)
            self._start_trajectory(joint_traj)
            
        except Exception as e:
            print(f"Error in start_move: {e}")
            self.status_updated.emit(f"Error: {e}", "red")

    @pyqtSlot(np.ndarray, np.ndarray)
    def start_linear_move(self, start_pose_matrix, end_pose_matrix):
        """Linear Move (XYZ)."""
        print("[WORKER] Generating linear trajectory...")
        
        if not self._validate_cartesian_target(end_pose_matrix):
            return 
        
        try:
            dt_s = MAIN_LOOP_TIMER_MS / 1000.0
            dist_lin = np.linalg.norm(end_pose_matrix[:3, 3] - start_pose_matrix[:3, 3])
            
            # Rotation
            r_start = R.from_matrix(start_pose_matrix[:3, :3])
            r_end = R.from_matrix(end_pose_matrix[:3, :3])
            try: dist_rot_rad = (r_end * r_start.inv()).magnitude()
            except: dist_rot_rad = 0.0

            # Apply Speed Multiplier
            effective_speed = LINEAR_SPEED_M_S * self.speed_multiplier
            if effective_speed < 0.01: effective_speed = 0.01
            
            time_lin = dist_lin / effective_speed
            BASE_ANG_SPEED = 1.5 
            time_rot = dist_rot_rad / (BASE_ANG_SPEED * self.speed_multiplier)
            
            travel_time_s = max(time_lin, time_rot)
            if travel_time_s < dt_s: travel_time_s = dt_s

            num_points = int(travel_time_s / dt_s)
            if num_points < 5: num_points = 5 
            
            traj_pos, traj_rot = self.kinematics.generate_linear_tcp_trajectory(
                start_pose_matrix, end_pose_matrix, num_points 
            )
            
            joint_traj = self.kinematics.joint_trajectory_from_tcp(
                traj_pos, traj_rot, initial_guess=self.get_previous_angles()
            )
            
            self._start_trajectory(joint_traj, is_program_move=False)

        except Exception as e:
            print(f"Error MOVEL: {e}")
            self.status_updated.emit(f"Error: {e}", "red")

    # -----------------------------------------------------------------
    # MAIN LOOP & TRAJECTORY EXECUTION
    # -----------------------------------------------------------------

    @pyqtSlot()
    def _update_step(self):
        self._check_serial_feedback()
        if self.is_moving:
            self._execute_trajectory_step()

    def _execute_trajectory_step(self):
        if self.is_stop_requested:
            self._handle_stop()
            return

        if self.trajectory_index >= len(self.trajectory_queue):
            self._finish_move()
            return

        try:
            angles = self.trajectory_queue[self.trajectory_index]
            
            # Handle old tuple format if present
            if isinstance(angles, tuple): angles = angles[0]

            # Padding
            angles_to_send = angles
            if len(angles) == 6:
                angles_to_send = np.concatenate(([0.0], angles))
            
            comm.send_angles(self.current_port, angles_to_send)
            self.previous_angles = angles_to_send
            self.angles_updated.emit(angles_to_send)
            self.trajectory_index += 1
            
        except Exception as e:
            print(f"Critical Trajectory Error: {e}")
            self._handle_stop()

    def _finish_move(self):
        self.is_moving = False
        self.trajectory_queue = []
        self.trajectory_index = 0
        self.status_updated.emit("Move Finished", "green")
        if self.is_program_waiting_for_move:
            self.is_program_waiting_for_move = False
            self.program_line_index += 1
            QTimer.singleShot(0, self._execute_next_program_step)

    def _handle_stop(self):
        self.is_moving = False
        self.trajectory_queue = []
        self.trajectory_index = 0
        self.status_updated.emit("Stopped", "orange")

    def _generate_joint_trajectory_ptp(self, start_angles, end_angles, speed_factor=1.0):
        if len(start_angles) == 7: s = start_angles[1:]
        else: s = start_angles
        if len(end_angles) == 7: e = end_angles[1:]
        else: e = end_angles
        
        BASE_JOINT_SPEED = 3.0 
        real_speed = BASE_JOINT_SPEED * self.speed_multiplier
        if real_speed < 0.1: real_speed = 0.1
        
        max_diff = np.max(np.abs(e - s))
        travel_time = max_diff / (real_speed * speed_factor)
        
        dt_s = MAIN_LOOP_TIMER_MS / 1000.0
        if travel_time < dt_s: travel_time = dt_s
            
        num_points = int(travel_time / dt_s)
        if num_points < 2: num_points = 2
            
        traj = []
        for i in range(num_points):
            t = i / (num_points - 1)
            t_smooth = (1 - np.cos(t * np.pi)) / 2
            val = s + (e - s) * t_smooth
            traj.append(val)
        return traj

    def _start_trajectory(self, joint_trajectory, is_program_move=False):
        if self.is_moving: return False
        if len(joint_trajectory) == 0: return False
        
        self.is_stop_requested = False 
        self.trajectory_queue = joint_trajectory
        self.trajectory_index = 0
        self.is_moving = True
        self.is_program_waiting_for_move = is_program_move
        self.status_updated.emit("Moving...", "blue")
        return True

    # -----------------------------------------------------------------
    # OTHER SLOTS
    # -----------------------------------------------------------------

    def get_previous_angles(self):
        return self.previous_angles

    @pyqtSlot(str)
    def set_com_port(self, port_name):
        self.stop_program()
        if self.current_port == port_name: return 
        comm.close_serial_port()
        self.current_port = port_name
        if port_name != "No ports":
            try:
                comm.open_serial_port(port_name)
                # self.start_move(self.previous_angles) # Optional refresh
            except Exception:
                self.current_port = None

    @pyqtSlot()
    def start_homing(self):
        if self.is_moving: return
        if self.current_port: 
            self.status_updated.emit("Homing...", "blue")
            comm.send_command(self.current_port, "HOME")

    @pyqtSlot(str)
    def set_gripper_state(self, command):
        if self.current_port: comm.send_command(self.current_port, command)

    def _check_serial_feedback(self):
        """Sprawdza czy przyszły dane z robota i przekazuje je do GUI."""
        if not self.current_port: return
        
        try:
            line = comm.read_line(self.current_port)
        except Exception:
            return

        if not line: return
        
        # WAŻNE: Usuwamy białe znaki końca linii (\r\n), żeby parsowanie było łatwiejsze
        line = line.strip() 
        if not line: return
        
        # --- DEBUGOWANIE ---
        # print(f"[WORKER RAW UART] {line}") 
        
        # 1. ESTOP (Priorytet)
        if "ESTOP_TRIGGER" in line:
            self.estop_status_signal.emit(True)
            self.stop_program()
            self.status_updated.emit("ESTOP TRIGGERED!", "red")
            
        elif "ESTOP_RELEASE" in line:
            self.estop_status_signal.emit(False)
            self.status_updated.emit("ESTOP RELEASED", "green")
            
        # 2. Status krańcówek (Legacy $H/$R)
        elif line.startswith("$H"):
            try: self.limit_switch_status.emit(int(line[2])-1, True)
            except: pass
        elif line.startswith("$R"):
            try: self.limit_switch_status.emit(int(line[2])-1, False)
            except: pass
            
        # 3. Zakończenie bazowania
        elif "HOMING_COMPLETE" in line:
            self.status_updated.emit("HOMING_COMPLETE_OK", "green")
            
        # 4. PRZEPUSZCZANIE INNYCH WIADOMOŚCI (Fix dla H1, R1, POS, etc.)
        else:
            # Emituj wszystko inne do GUI, żeby funkcja update_status_label mogła to sparsować
            # Np. "H1", "R1", "Error: X"
            self.status_updated.emit(line, "white")

    # -----------------------------------------------------------------
    # PROGRAM INTERPRETER
    # -----------------------------------------------------------------

    @pyqtSlot()
    def stop_program(self):
        self.is_stop_requested = True
        self.is_moving = False
        self.trajectory_queue = []

    @pyqtSlot(str)
    def start_program(self, program_text):
        self.is_stop_requested = False
        self.program_queue = []
        self.program_line_map = []
        lines = program_text.split('\n')
        for i, line in enumerate(lines):
            l = line.split('#')[0].strip().upper()
            if not l: continue
            try:
                if m := re.match(r"MOVE\s*\(\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)\s*\)", l):
                    self.program_queue.append(('MOVE', (float(m.group(1)), float(m.group(2)), float(m.group(3)))))
                    self.program_line_map.append(i)
                elif m := re.match(r"MOVEL\s*\(\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)\s*\)", l):
                    self.program_queue.append(('MOVEL', (float(m.group(1)), float(m.group(2)), float(m.group(3)))))
                    self.program_line_map.append(i)
                elif m := re.match(r"DELAY\s*\(\s*(\d+)\s*\)", l):
                    self.program_queue.append(('DELAY', (int(m.group(1)),)))
                    self.program_line_map.append(i)
                
                # Custom commands
                elif l == "MOVESTB": self.program_queue.append(('MOVESTB', ())); self.program_line_map.append(i)
                elif l == "VACON": self.program_queue.append(('VACON', ())); self.program_line_map.append(i)
                elif l == "VACOFF": self.program_queue.append(('VACOFF', ())); self.program_line_map.append(i)
                elif l == "VGRIPON": self.program_queue.append(('VGRIPON', ())); self.program_line_map.append(i)
                elif l == "VGRIPOFF": self.program_queue.append(('VGRIPOFF', ())); self.program_line_map.append(i)
                elif m := re.match(r"EGRIP\s*\(\s*(OPEN|CLOSE)\s*\)", l):
                    self.program_queue.append(('EGRIP', (m.group(1),))); self.program_line_map.append(i)
            except: pass

        if self.program_queue:
            self.program_line_index = 0
            self._execute_next_program_step()
        else:
            self.program_finished.emit("Empty program", False)

    def _execute_next_program_step(self):
        if self.is_stop_requested:
            self.program_finished.emit("Stopped", True)
            return
        if self.program_line_index >= len(self.program_queue):
            self.program_finished.emit("Finished", False)
            return

        cmd, args = self.program_queue[self.program_line_index]
        orig_line = self.program_line_map[self.program_line_index]
        self.program_line_highlight.emit(orig_line)
        self.status_updated.emit(f"Line {orig_line+1}: {cmd}", "blue")

        if cmd == 'MOVE' or cmd == 'MOVEL':
            x, y, z = args
            target_tf = self._get_target_tf_from_user(x, y, z)
            
            if not self._validate_cartesian_target(target_tf):
                self.program_finished.emit("POS ERROR", True)
                return 

            if cmd == 'MOVE':
                target_ik = self.kinematics.inverse_kinematics(
                    target_tf[:3, 3], target_tf[:3, :3], self.get_previous_angles()
                )
                start_angles = self.get_previous_angles()
                joint_traj = self._generate_joint_trajectory_ptp(start_angles, target_ik)
                self._start_trajectory(joint_traj, True)
                
            elif cmd == 'MOVEL':
                start_tf = self.kinematics.forward_kinematics(self.get_previous_angles())
                dist = np.linalg.norm(target_tf[:3, 3] - start_tf[:3, 3])
                
                dt_s = MAIN_LOOP_TIMER_MS / 1000.0
                current_speed = LINEAR_SPEED_M_S * self.speed_multiplier
                if current_speed < 0.01: current_speed = 0.01
                travel_time_s = dist / current_speed
                
                num_points = int(travel_time_s / dt_s)
                if num_points < 5: num_points = 5
                
                traj_pos, traj_rot = self.kinematics.generate_linear_tcp_trajectory_points(start_tf, target_tf, num_points)
                joint_traj = self.kinematics.joint_trajectory_from_tcp(traj_pos, traj_rot, self.get_previous_angles())
                self._start_trajectory(joint_traj, True)
            
        elif cmd == 'DELAY':
            QTimer.singleShot(args[0], self._on_program_delay_finished)
            
        elif cmd == 'MOVESTB':
            start_angles = self.get_previous_angles()
            joint_traj = self._generate_joint_trajectory_ptp(start_angles, self.standby_angles)
            self._start_trajectory(joint_traj, True)
            
        elif cmd == 'VACON': self.set_gripper_state("VAC_ON"); self._step_next()
        elif cmd == 'VACOFF': self.set_gripper_state("VAC_OFF"); self._step_next()
        elif cmd == 'VGRIPON': self.set_gripper_state("VGripON"); self._step_next()
        elif cmd == 'VGRIPOFF': self.set_gripper_state("VGripOFF"); self._step_next()
        elif cmd == 'EGRIP':
            if args[0] == 'OPEN': self.set_gripper_state("EGRIP_OPEN")
            else: self.set_gripper_state("EGRIP_CLOSE")
            self._step_next()

    def _step_next(self):
        self.program_line_index += 1
        QTimer.singleShot(0, self._execute_next_program_step)

    def _on_program_delay_finished(self):
        self._step_next()

    # --- HELPERS ---
    def _get_target_tf_from_user(self, x, y, z):
        pos_m = self._convert_user_coords_to_robot(x, y, z)
        start_tf = self.kinematics.forward_kinematics(self.get_previous_angles())
        target_tf = np.eye(4)
        target_tf[:3, :3] = start_tf[:3, :3] # Keep orientation
        target_tf[:3, 3] = pos_m
        return target_tf

    def _convert_user_coords_to_robot(self, X_u, Y_u, Z_u):
        # Direct Frame: X user -> X robot, Y user -> Y robot
        return np.array([X_u/1000.0, Y_u/1000.0, Z_u/1000.0])

    def _robot_coords_to_user(self, robot_pos_m):
        X_r, Y_r, Z_r = robot_pos_m
        return (X_r*1000.0, Y_r*1000.0, Z_r*1000.0)