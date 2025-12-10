from logging import root
from PyQt5.QtCore import QObject, QTimer, pyqtSlot, pyqtSignal, QThread
import numpy as np
import time
import threading

# === IMPORTY DLA IK ORAZ PROGRAMATORA ===
from scipy.spatial.transform import Rotation as R, Slerp
import re

try:
    import robot.communication as comm
except ImportError:
    print("OSTRZEŻENIE: Nie można zaimportować 'robot.communication'. Używam atrapy.")
    
    class CommMock:
        def open_serial_port(self, port_name, baud_rate=115200, timeout=0.1):
            print(f"[ATRAPA] Otwieram port {port_name}")
            return True 
        def close_serial_port(self):
            print("[ATRAPA] Zamykam port.")
        def read_line(self, port_name):
            # return "POS j1=100 j2=200 j3=300 j4=400 j5=500 j6=600"
            return None
        def send_angles(self, port_name, angles):
            # print(f"[ATRAPA] Wysyłam kąty: {angles}")
            pass 
        def send_command(self, port_name, command_string):
            print(f"[ATRAPA] Wysyłam komendę '{command_string}'")
            
    comm = CommMock()

# === USTAWIENIA GŁÓWNEJ PĘTLI ===
MAIN_LOOP_TIMER_MS = 20 

# <--- ZMIANA: PRĘDKOŚĆ RUCHU LINIOWEGO (m/s)
# 0.05 = 5 cm/s (wolno), 0.2 = 20 cm/s (szybko)
LINEAR_SPEED_M_S = 0.2  

class RobotWorker(QObject):
    # Sygnały do GUI
    angles_updated = pyqtSignal(np.ndarray)
    # USUNIĘTO: real_angles_updated
    status_updated = pyqtSignal(str, str)
    limit_switch_status = pyqtSignal(int, bool)
    
    # Sygnały Programatora
    program_finished = pyqtSignal(str, bool)
    program_line_highlight = pyqtSignal(int)

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
        
        self.main_loop_timer = QTimer(self)
        self.main_loop_timer.timeout.connect(self._update_step)
        self.main_loop_timer.start(MAIN_LOOP_TIMER_MS) 
        print(f"[WORKER] Uruchomiono główną pętlę (co {MAIN_LOOP_TIMER_MS}ms).")

    # -----------------------------------------------------------------
    # GŁÓWNA PĘTLA (NON-BLOCKING)
    # -----------------------------------------------------------------

    @pyqtSlot()
    def _update_step(self):
        self._check_serial_feedback()
        if self.is_moving:
            self._execute_trajectory_step()

    def _execute_trajectory_step(self):
        if self.is_stop_requested:
            self.is_moving = False
            self.trajectory_queue = []
            self.trajectory_index = 0
            self.status_updated.emit("Ruch zatrzymany", "orange")
            return

        if self.trajectory_index >= len(self.trajectory_queue):
            self.is_moving = False
            self.trajectory_queue = []
            self.trajectory_index = 0
            self.status_updated.emit("Ruch zakończony", "green")
            
            if self.is_program_waiting_for_move:
                print("[WORKER] Ruch liniowy programu zakończony, wznawiam program.")
                self.is_program_waiting_for_move = False
                self.program_line_index += 1
                QTimer.singleShot(0, self._execute_next_program_step)
            return

        try:
            angles = self.trajectory_queue[self.trajectory_index]
            comm.send_angles(self.current_port, angles)
            
            # Aktualizuj GUI (na podstawie wysłanej pozycji ZADANEJ)
            self.previous_angles = angles
            self.angles_updated.emit(angles) # Model 3D i etykiety J1-J6
            
            self.trajectory_index += 1
        except Exception as e:
            print(f"Krytyczny błąd podczas kroku trajektorii: {e}")
            self.is_moving = False
            self.trajectory_queue = []
            self.trajectory_index = 0
            self.status_updated.emit(f"Błąd wysyłania: {e}", "red")

    def _check_serial_feedback(self):
        """Sprawdza port szeregowy w poszukiwaniu danych (krańcówki, pozycje)."""
        if self.current_port is None or self.current_port == "Brak portów":
            return
            
        try:
            line = comm.read_line(self.current_port) 
            if not line:
                return 

            # === PARSOWANIE POZYCJI (TWOJA RAMKA) ===
            if line.startswith("POS "):
                match = re.search(r"j1=(-?\d+)\s+j2=(-?\d+)\s+j3=(-?\d+)\s+j4=(-?\d+)\s+j5=(-?\d+)\s+j6=(-?\d+)", line)
                if match:
                    try:
                        raw_counts = [int(match.group(i)) for i in range(1, 7)]
                        angles_rad = self._convert_raw_to_angles(raw_counts)
                        
                        self.previous_angles = angles_rad
                        self.angles_updated.emit(angles_rad)
                        
                    except Exception as e:
                        print(f"Błąd parsowania ramki POS: {e} (Linia: {line})")

            
            elif line.startswith("$H"): # Krańcówka WCIŚNIĘTA
                try:
                    joint_index = int(line[2]) - 1 
                    if 0 <= joint_index <= 5:
                        self.limit_switch_status.emit(joint_index, True)
                except Exception as e:
                    print(f"Błąd parsowania ramki $H: {e}")
                    
            elif line.startswith("$R"): # Krańcówka ZWOLNIONA
                try:
                    joint_index = int(line[2]) - 1
                    if 0 <= joint_index <= 5:
                        self.limit_switch_status.emit(joint_index, False)
                except Exception as e:
                    print(f"Błąd parsowania ramki $R: {e}")

        except Exception as e:
            pass # Ignoruj błędy odczytu

    # -----------------------------------------------------------------
    # PUBLICZNE SLOTY (Interfejs dla GUI)
    # -----------------------------------------------------------------

    def get_previous_angles(self):
        return self.previous_angles

    @pyqtSlot(str)
    def set_com_port(self, port_name):
        self.stop_program()
        if self.current_port == port_name:
            return 
        print(f"[WORKER] Zmieniam port na {port_name}")
        comm.close_serial_port()
        self.current_port = port_name
        if port_name != "Brak portów":
            try:
                comm.open_serial_port(port_name)
                self.start_move(self.previous_angles)
            except Exception as e:
                self.status_updated.emit(f"Błąd: Port {port_name} nie działa", "red")
                self.current_port = None
        else:
             self.current_port = None 

    @pyqtSlot(np.ndarray)
    def start_move(self, target_angles_rad):
        if self.is_moving:
            self.status_updated.emit("Błąd: Ruch liniowy w toku", "orange")
            return
            
        if self.current_port is None or self.current_port == "Brak portów":
            self.status_updated.emit("Błąd: Nie wybrano portu", "red")
            self.previous_angles = target_angles_rad
            self.angles_updated.emit(target_angles_rad)
            return

        try:
            self.status_updated.emit("Wysyłanie (PTP)...", "orange")
            comm.send_angles(self.current_port, target_angles_rad)
            self.status_updated.emit("Wysłano (PTP)", "green")
            self.previous_angles = target_angles_rad
            self.angles_updated.emit(target_angles_rad)
        except Exception as e:
            print(f"Worker: Błąd podczas wysyłania PTP: {e}")
            self.status_updated.emit(f"Błąd UART: {e}", "red")
            self.previous_angles = target_angles_rad
            self.angles_updated.emit(target_angles_rad)

    @pyqtSlot()
    def start_homing(self):
        if self.is_moving:
            self.status_updated.emit("Błąd: Ruch w toku", "orange")
            return
            
        if self.current_port is None or self.current_port == "Brak portów":
            self.status_updated.emit("Błąd: Nie wybrano portu", "red")
            return

        try:
            self.status_updated.emit("Wysyłanie $HOME...", "orange")
            comm.send_command(self.current_port, "HOME") 
            self.status_updated.emit("Wysłano HOME", "green")
        except Exception as e:
            print(f"Worker: Błąd podczas wysyłania HOME: {e}")
            self.status_updated.emit(f"Błąd UART: {e}", "red")

    @pyqtSlot(str)
    def set_gripper_state(self, command):
        if self.current_port is None or self.current_port == "Brak portów":
            self.status_updated.emit("Status: Brak portu COM", "orange")
            return 
        try:
            self.status_updated.emit(f"Wysyłanie {command}...", "orange")
            comm.send_command(self.current_port, command) 
            self.status_updated.emit(f"Wysłano {command}", "blue") 
        except Exception as e:
            print(f"Błąd wysyłania polecenia chwytaka: {e}")
            self.status_updated.emit(f"Status: Błąd {command}", "red")

    def _start_trajectory(self, joint_trajectory, is_program_move=False):
        if self.is_moving:
            self.status_updated.emit("Błąd: Ruch w toku", "orange")
            return False
        if len(joint_trajectory) == 0:
            self.status_updated.emit("Błąd: Pusta trajektoria", "red")
            return False
        print(f"[WORKER] Rozpoczynam trajektorię o długości {len(joint_trajectory)} kroków.")
        self.trajectory_queue = joint_trajectory
        self.trajectory_index = 0
        self.is_moving = True
        self.is_program_waiting_for_move = is_program_move
        self.status_updated.emit("Ruch liniowy...", "blue")
        return True

    @pyqtSlot(np.ndarray, np.ndarray)
    def start_linear_move(self, start_pose_matrix, end_pose_matrix):
        print("[WORKER] Generuję trajektorię liniową (z GUI)...")
        self.status_updated.emit("Generuję trajektorię...", "orange")
        try:
            dt_s = MAIN_LOOP_TIMER_MS / 1000.0 
            dist = np.linalg.norm(end_pose_matrix[:3, 3] - start_pose_matrix[:3, 3])
            
            # <--- ZMIANA: PRĘDKOŚĆ USTAWIANA PRZEZ STAŁĄ
            travel_time_s = dist / LINEAR_SPEED_M_S 
            
            num_points = int(travel_time_s / dt_s)
            if num_points < 10: 
                num_points = 10
            
            print(f"[WORKER] Dystans: {dist:.3f}m, Czas: {travel_time_s:.2f}s, Punkty: {num_points}")
            positions = np.linspace(start_pose_matrix[:3, 3], end_pose_matrix[:3, 3], num_points)
            start_rot = R.from_matrix(start_pose_matrix[:3, :3])
            end_rot = R.from_matrix(end_pose_matrix[:3, :3])
            key_times = [0, 1]
            key_rots = R.from_quat([start_rot.as_quat(), end_rot.as_quat()])
            slerp = Slerp(key_times, key_rots)
            orientations = slerp(np.linspace(0, 1, num_points))
            joint_traj = []
            prev_angles = self.get_previous_angles()

            for i in range(num_points):
                try:
                    joint_angles = self.kinematics.continuous_ik(
                        positions[i],
                        orientations[i].as_matrix(),
                        prev_angles
                    )
                    joint_traj.append(joint_angles)
                    prev_angles = joint_angles
                except Exception as e:
                    print(f"Błąd IK w punkcie {i}: {e}. Używam poprzednich kątów.")
                    if joint_traj:
                        joint_traj.append(joint_traj[-1])
                    else:
                        joint_traj.append(self.get_previous_angles())
            
            self._start_trajectory(np.array(joint_traj), is_program_move=False)

        except Exception as e:
            print(f"Błąd generowania trajektorii liniowej: {e}")
            self.status_updated.emit(f"Błąd trajektorii: {e}", "red")

    # -----------------------------------------------------------------
    # LOGIKA PROGRAMATORA
    # -----------------------------------------------------------------

    @pyqtSlot()
    def stop_program(self):
        if self.program_queue or self.is_moving:
            print("WORKER: Otrzymano żądanie STOP")
            self.is_stop_requested = True
            self.program_queue = []
            self.is_program_waiting_for_move = False

    @pyqtSlot(str)
    def start_program(self, program_text):
        if self.is_moving:
            self.status_updated.emit("Błąd: Ruch w toku", "orange")
            return
        print("WORKER: Rozpoczynam parsowanie programu...")
        self.is_stop_requested = False
        self.program_queue = []
        self.program_line_index = 0
        self.program_line_map = []
        lines = program_text.split('\n')
        for line_num, line in enumerate(lines):
            original_line = line
            line = line.split('#')[0].strip().upper()
            if not line:
                continue
            try:
                if match := re.match(r"MOVE\s*\(\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)\s*\)", line):
                    self.program_queue.append(('MOVE', (float(match.group(1)), float(match.group(2)), float(match.group(3)))))
                    self.program_line_map.append(line_num)
                elif match := re.match(r"MOVEL\s*\(\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)\s*\)", line):
                    self.program_queue.append(('MOVEL', (float(match.group(1)), float(match.group(2)), float(match.group(3)))))
                    self.program_line_map.append(line_num)
                elif match := re.match(r"MOVEZ\s*\(\s*(-?\d+\.?\d*)\s*\)", line):
                    self.program_queue.append(('MOVEZ', (float(match.group(1)),)))
                    self.program_line_map.append(line_num)
                elif match := re.match(r"DELAY\s*\(\s*(\d+)\s*\)", line):
                    self.program_queue.append(('DELAY', (int(match.group(1)),)))
                    self.program_line_map.append(line_num)
                elif line == "MOVESTB":
                    self.program_queue.append(('MOVESTB', ()))
                    self.program_line_map.append(line_num)
                elif line == "VACON":
                    self.program_queue.append(('VACON', ()))
                    self.program_line_map.append(line_num)
                elif line == "VACOFF":
                    self.program_queue.append(('VACOFF', ()))
                    self.program_line_map.append(line_num)
                # <--- ZMIANA: PARSOWANIE NOWYCH KOMEND VGRIP
                elif line == "VGRIPON":
                    self.program_queue.append(('VGRIPON', ()))
                    self.program_line_map.append(line_num)
                elif line == "VGRIPOFF":
                    self.program_queue.append(('VGRIPOFF', ()))
                    self.program_line_map.append(line_num)
                else:
                    raise ValueError(f"Nieznana komenda: {original_line}")
            except Exception as e:
                print(f"WORKER: Błąd parsowania w linii {line_num+1}: {e}")
                self.status_updated.emit(f"Błąd linii {line_num+1}: {e}", "red")
                self.program_finished.emit("Błąd programu", True)
                self.program_line_highlight.emit(line_num)
                return
        if self.program_queue:
            self.program_line_index = 0
            self._execute_next_program_step()
        else:
            self.program_finished.emit("Program pusty", False)

    def _execute_next_program_step(self):
        if self.is_stop_requested:
            self.is_stop_requested = False
            print("WORKER: Program zatrzymany przez użytkownika.")
            self.program_finished.emit("Zatrzymano", True)
            self.program_line_highlight.emit(-1)
            self.program_queue = []
            return

        if self.program_line_index >= len(self.program_queue):
            print("WORKER: Program zakończony pomyślnie.")
            self.program_finished.emit("Program zakończony", False)
            self.program_line_highlight.emit(-1)
            self.program_queue = []
            return

        command, args = self.program_queue[self.program_line_index]
        original_line_num = self.program_line_map[self.program_line_index]
        print(f"WORKER: Wykonuję linię {original_line_num+1}: {command} {args}")
        self.program_line_highlight.emit(original_line_num)
        self.status_updated.emit(f"Program: Linia {original_line_num+1}", "blue")

        try:
            if command == 'MOVE':
                x, y, z = args
                target_tf = self._get_target_tf_from_user(x, y, z)
                target_angles = self._calculate_ik(target_tf)
                self.start_move(target_angles)
                self.program_line_index += 1
                QTimer.singleShot(0, self._execute_next_program_step)
            elif command == 'MOVEL':
                x, y, z = args
                start_tf = self._get_current_target_transform()
                target_tf = self._get_target_tf_from_user(x, y, z)
                joint_traj = self._generate_trajectory(start_tf, target_tf)
                if not self._start_trajectory(joint_traj, is_program_move=True):
                    raise RuntimeError("Nie można uruchomić trajektorii MOVEL")
            elif command == 'MOVEZ':
                z_new_user, = args
                start_tf = self._get_current_target_transform()
                current_pos_robot_m = start_tf[:3, 3]
                x_user, y_user, _ = self._robot_coords_to_user(current_pos_robot_m)
                target_tf = self._get_target_tf_from_user(x_user, y_user, z_new_user)
                joint_traj = self._generate_trajectory(start_tf, target_tf)
                if not self._start_trajectory(joint_traj, is_program_move=True):
                    raise RuntimeError("Nie można uruchomić trajektorii MOVEZ")
            elif command == 'MOVESTB':
                self.start_move(self.standby_angles)
                self.program_line_index += 1
                QTimer.singleShot(0, self._execute_next_program_step)
            elif command == 'DELAY':
                ms, = args
                QTimer.singleShot(ms, self._on_program_delay_finished)
            elif command == 'VACON':
                self.set_gripper_state("VAC_ON")
                self.program_line_index += 1
                QTimer.singleShot(0, self._execute_next_program_step)
            elif command == 'VACOFF':
                self.set_gripper_state("VAC_OFF")
                self.program_line_index += 1
                QTimer.singleShot(0, self._execute_next_program_step)
            
            # <--- ZMIANA: WYKONANIE NOWYCH KOMEND VGRIP
            elif command == 'VGRIPON':
                self.set_gripper_state("VGripON")
                self.program_line_index += 1
                QTimer.singleShot(0, self._execute_next_program_step)
            elif command == 'VGRIPOFF':
                self.set_gripper_state("VGripOFF")
                self.program_line_index += 1
                QTimer.singleShot(0, self._execute_next_program_step)
                
            else:
                raise ValueError(f"Nieznane polecenie: {command}")
        except Exception as e:
            print(f"WORKER: Błąd wykonania w linii {original_line_num+1}: {e}")
            self.status_updated.emit(f"Błąd linii {original_line_num+1}: {e}", "red")
            self.program_finished.emit("Błąd programu", True)
            self.program_line_highlight.emit(original_line_num)
            self.program_queue = []
            self.is_moving = False
            self.trajectory_queue = []

    def _on_program_delay_finished(self):
        if self.is_stop_requested:
            self._execute_next_program_step()
            return
        self.program_line_index += 1
        self._execute_next_program_step()

    # -----------------------------------------------------------------
    # FUNKCJE POMOCNICZE
    # -----------------------------------------------------------------

    def _get_target_tf_from_user(self, x, y, z):
        pos_m_robot = self._convert_user_coords_to_robot(x, y, z)
        start_tf = self._get_current_target_transform()
        target_tf = start_tf.copy()
        target_tf[:3, 3] = pos_m_robot
        return target_tf

    def _calculate_ik(self, target_tf):
        target_pos = target_tf[:3, 3]
        target_orient = target_tf[:3, :3]
        initial_guess = self.get_previous_angles()
        try:
            target_angles = self.kinematics.inverse_kinematics(target_pos, target_orient, initial_guess)
            if target_angles is None:
                raise RuntimeError("Brak rozwiązania IK")
            return target_angles
        except Exception as ik_err:
            raise RuntimeError(f"Błąd IK: {ik_err}")

    def _generate_trajectory(self, start_tf, end_tf):
        try:
            dt_s = MAIN_LOOP_TIMER_MS / 1000.0
            dist = np.linalg.norm(end_tf[:3, 3] - start_tf[:3, 3])
            
            # <--- ZMIANA: PRĘDKOŚĆ USTAWIANA PRZEZ STAŁĄ
            travel_time_s = dist / LINEAR_SPEED_M_S 
            
            num_points = int(travel_time_s / dt_s)
            if num_points < 2: 
                num_points = 2
            positions = np.linspace(start_tf[:3, 3], end_tf[:3, 3], num_points)
            start_rot = R.from_matrix(start_tf[:3, :3])
            end_rot = R.from_matrix(end_tf[:3, :3])
            key_times = [0, 1]
            key_rots = R.from_quat([start_rot.as_quat(), end_rot.as_quat()])
            slerp = Slerp(key_times, key_rots)
            orientations = slerp(np.linspace(0, 1, num_points))
            joint_traj = []
            prev_angles = self.get_previous_angles()
            for i in range(num_points):
                try:
                    joint_angles = self.kinematics.continuous_ik(
                        positions[i],
                        orientations[i].as_matrix(),
                        prev_angles
                    )
                    joint_traj.append(joint_angles)
                    prev_angles = joint_angles
                except Exception as e:
                    if joint_traj:
                        joint_traj.append(joint_traj[-1])
                    else:
                        joint_traj.append(self.get_previous_angles())
            return np.array(joint_traj)
        except Exception as e:
            raise RuntimeError(f"Błąd generowania trajektorii: {e}")

    def _convert_raw_to_angles(self, raw_counts):
        MICROSTEPS_PER_REV_MOTOR = 51200.0
        GEAR_RATIOS = [6.4, 20.0, 18.0952381, 4, 4, 10]
        DIRECTIONS = [1.0, -1.0, -1.0, 1.0, -1.0, 1.0] 
        angles_deg = np.zeros(6)
        try:
            for i in range(6):
                target = raw_counts[i]
                ratio = GEAR_RATIOS[i]
                direction = DIRECTIONS[i]
                denominator = MICROSTEPS_PER_REV_MOTOR * ratio
                if denominator == 0:
                    angles_deg[i] = 0.0
                else:
                    angles_deg[i] = (target * 360.0) / denominator
                angles_deg[i] *= direction
            
            angles_rad_with_base = np.zeros(7)
            angles_rad_with_base[1:] = np.deg2rad(angles_deg)
            return angles_rad_with_base 
        except Exception as e:
            print(f"Błąd konwersji kątów: {e}")
            return self.previous_angles

    def _get_current_target_transform(self):
        try:
            start_angles = self.get_previous_angles() 
            return self.kinematics.forward_kinematics(start_angles)
        except Exception as e:
            print(f"Błąd krytyczny w _get_current_target_transform: {e}")
            return np.eye(4)

    def _convert_user_coords_to_robot(self, X_user_mm, Y_user_mm, Z_user_mm):
        OFFSET_X = 0.0 
        OFFSET_Y = 0
        OFFSET_Z = 100.0 
        X_user_mm += OFFSET_X
        Y_user_mm += OFFSET_Y
        Z_user_mm += OFFSET_Z
        X_robot_m =  Y_user_mm / 1000.0
        Y_robot_m = -X_user_mm / 1000.0
        Z_robot_m =  Z_user_mm / 1000.0
        return np.array([X_robot_m, Y_robot_m, Z_robot_m])

    def _robot_coords_to_user(self, robot_pos_m):
        OFFSET_X = 0.0 
        OFFSET_Y = 0
        OFFSET_Z = 100.0 
        X_robot_m, Y_robot_m, Z_robot_m = robot_pos_m
        X_user_mm_raw = -Y_robot_m * 1000.0
        Y_user_mm_raw =  X_robot_m * 1000.0
        Z_user_mm_raw =  Z_robot_m * 1000.0
        return (
            X_user_mm_raw - OFFSET_X,
            Y_user_mm_raw - OFFSET_Y,
            Z_user_mm_raw - OFFSET_Z
        )
    
    @pyqtSlot()
    def poll_joint6_position(self):
        pass