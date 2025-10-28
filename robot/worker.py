from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot, QThread
import numpy as np
import serial.tools.list_ports
from ikpy.chain import Chain
from scipy.spatial.transform import Rotation as R
import robot.communication as comm
import time

class RobotWorker(QObject):
    # Sygnały do komunikacji z GUI
    angles_calculated = pyqtSignal(np.ndarray)
    movement_finished = pyqtSignal()
    error_occurred = pyqtSignal(str)

    def __init__(self, robot_chain, standby_angles_rad, parent=None):
        super().__init__(parent)
        self.chain = robot_chain
        self.standby_angles_rad = standby_angles_rad.copy()
        
        self._current_angles_rad = standby_angles_rad.copy()
        self._current_effector_pos_m = None
        self._target_effector_pos_m = None
        
        self._serial_port_name = None
        self._serial_handle = None
        self._is_running = False

        # Parametry ruchu liniowego (płynność)
        self.anim_steps_total = 300   # liczba kroków interpolacji
        self.anim_step_delay_ms = 5   # czas opóźnienia między krokami (ms)

    @pyqtSlot(str)
    def set_serial_port(self, port_name):
        if self._serial_port_name != port_name:
            self._close_port_if_open()
            self._serial_port_name = port_name

    @pyqtSlot(np.ndarray)
    def set_current_angles(self, angles_rad):
        self._current_angles_rad = angles_rad.copy()
        # Aktualizacja pozycji efektora końcowego
        fk_result = self.chain.forward_kinematics(self._current_angles_rad, full_kinematics=True)
        self._current_effector_pos_m = fk_result[-1][:3, 3]

    @pyqtSlot(np.ndarray)
    def start_linear_movement(self, target_pos_m):
        if self._is_running:
            self.error_occurred.emit("Robot jest już w ruchu.")
            return

        if self._current_effector_pos_m is None:
            fk_result = self.chain.forward_kinematics(self._current_angles_rad, full_kinematics=True)
            self._current_effector_pos_m = fk_result[-1][:3, 3]

        if np.allclose(target_pos_m, self._current_effector_pos_m, atol=1e-3):
            self.error_occurred.emit("Robot jest już w tej pozycji.")
            return

        self._target_effector_pos_m = target_pos_m.copy()
        self._is_running = True
        self._execute_linear_movement()

    def _execute_linear_movement(self):
        if not self._open_port_if_needed():
            self._is_running = False
            return

        start_pos_m = self._current_effector_pos_m.copy()
        target_pos_m = self._target_effector_pos_m.copy()

        try:
            for step in range(1, self.anim_steps_total + 1):
                if not self._is_running:
                    break

                t = step / self.anim_steps_total
                intermediate_pos_m = start_pos_m + t * (target_pos_m - start_pos_m)

                try:
                    intermediate_angles_rad = self._try_inverse_with_seeds(
                        intermediate_pos_m, self._current_angles_rad
                    )
                except Exception as e:
                    self.error_occurred.emit(f"Błąd IK na kroku {step}: {e}")
                    self._is_running = False
                    break

                self._current_angles_rad = intermediate_angles_rad.copy()
                self._current_effector_pos_m = intermediate_pos_m.copy()

                # Aktualizacja GUI
                self.angles_calculated.emit(intermediate_angles_rad)

                # Wysyłaj kąty co kilka kroków, aby nie zamulać UART
                if step % 5 == 0 and self._serial_handle and self._serial_handle.is_open:
                    try:
                        angles_to_send_deg = np.degrees(intermediate_angles_rad[1:])
                        comm.send_angles_on_open_port(self._serial_handle, angles_to_send_deg)
                    except Exception as e:
                        self.error_occurred.emit(f"Błąd wysyłania UART: {e}")
                        self._is_running = False
                        break

                QThread.msleep(self.anim_step_delay_ms)

        finally:
            self._is_running = False
            self.movement_finished.emit()
            self._close_port_if_open()

    @pyqtSlot()
    def stop_movement(self):
        self._is_running = False

    @pyqtSlot()
    def send_standby(self):
        if self._is_running:
            self.stop_movement()
            QThread.msleep(50)

        self._is_running = True
        if not self._open_port_if_needed():
            self._is_running = False
            return

        try:
            angles_rad = self.standby_angles_rad.copy()
            if self._serial_handle and self._serial_handle.is_open:
                angles_deg = np.degrees(angles_rad[1:])
                comm.send_angles_on_open_port(self._serial_handle, angles_deg)

            self._current_angles_rad = angles_rad.copy()
            fk_result = self.chain.forward_kinematics(angles_rad, full_kinematics=True)
            self._current_effector_pos_m = fk_result[-1][:3, 3]

            self.angles_calculated.emit(angles_rad)
            self.movement_finished.emit()

        except Exception as e:
            self.error_occurred.emit(f"Błąd ustawiania standby: {e}")
        finally:
            self._is_running = False
            self._close_port_if_open()

    def _try_inverse_with_seeds(self, target_position, initial_seed_angles):
        seeds = [
            self._clamp_to_limits(initial_seed_angles),
            self._clamp_to_limits(self.standby_angles_rad),
            np.zeros(len(self.chain.links))
        ]
        last_exc = None
        for seed in seeds:
            try:
                angles = self.chain.inverse_kinematics(
                    target_position=target_position,
                    orientation_mode=None,
                    initial_position=seed
                )
                return np.array(angles)
            except Exception as e:
                last_exc = e
        raise last_exc

    def _clamp_to_limits(self, arr):
        a = np.array(arr, dtype=float)
        if hasattr(self.chain, "lower_limits") and hasattr(self.chain, "upper_limits"):
            low = np.array(self.chain.lower_limits, dtype=float)
            high = np.array(self.chain.upper_limits, dtype=float)
            if low.shape == a.shape and high.shape == a.shape:
                a = np.maximum(a, low)
                a = np.minimum(a, high)
        return a

    def _open_port_if_needed(self):
        if self._serial_port_name in (None, "Brak portów"):
            self._close_port_if_open()
            return True
        if self._serial_handle and self._serial_handle.is_open and self._serial_handle.port == self._serial_port_name:
            return True
        self._close_port_if_open()
        try:
            self._serial_handle = comm.open_serial_port(self._serial_port_name)
            return True
        except Exception as e:
            self.error_occurred.emit(f"Nie można otworzyć portu {self._serial_port_name}: {e}")
            return False

    def _close_port_if_open(self):
        if self._serial_handle and self._serial_handle.is_open:
            comm.close_serial_port()
            self._serial_handle = None

    def cleanup(self):
        self._close_port_if_open()
