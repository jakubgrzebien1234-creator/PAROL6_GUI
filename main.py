import sys
from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtCore import QThread, pyqtSlot, pyqtSignal
import pyqtgraph.opengl as gl
import numpy as np
import trimesh
from PyQt5.QtGui import QMatrix4x4, QVector4D, QVector3D
from OpenGL import GL
import functools 

# === IMPORTY LOKALNYCH MODUŁÓW ===
import robot.config as config
from robot.kinematics import RobotKinematics
from robot.worker import RobotWorker
try:
    # Importuj prawdziwy moduł comm, aby przekazać go do workera
    import robot.communication as comm 
except ImportError:
    # Jeśli nie ma, worker użyje własnej atrapy
    from robot.worker import comm # Importuj atrapę z workera

# =Aplikacja korzysta również z `config.py` i `kinematics.py`,
# które wysłałem w poprzedniej wiadomości.
# ======================================

from scipy.spatial.transform import Rotation as R

# ------------------------
# Helper: konwersja macierzy 4x4 -> QMatrix4x4
# ------------------------
def matrix_to_qtransform(matrix):
    """Konwertuje macierz transformacji 4x4 NumPy na QMatrix4x4."""
    m = QMatrix4x4()
    for i in range(4):
        m.setRow(i, QVector4D(float(matrix[i, 0]), float(matrix[i, 1]), float(matrix[i, 2]), float(matrix[i, 3])))
    return m


# ------------------------
# Helper: Bezpieczne parsowanie float
# ------------------------
def parse_float_from_input(line_edit, default_val=0.0):
    """
    Bezpiecznie parsuje float z QLineEdit.
    Obsługuje puste pola, przecinki i nieprawidłowe znaki.
    """
    text = line_edit.text().strip().replace(',', '.')
    if not text or text == "-":
        return default_val
    try:
        return float(text)
    except ValueError:
        print(f"Nieprawidłowa wartość w polu: '{text}', używam domyślnej {default_val}")
        return default_val
    
# ------------------------
# Klasa Głównego Okna
# ------------------------
class MainWindow(QtWidgets.QWidget):
    
    # Sygnał do uruchomienia ruchu w wątku roboczym
    start_move_signal = pyqtSignal(np.ndarray)
    # Sygnał do ustawienia portu COM w wątku roboczym
    set_port_signal = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        
        # Inicjalizacja silnika kinematyki
        self.kinematics = RobotKinematics(config.URDF_PATH, config.ACTIVE_LINKS_MASK)
        self.joint_limits_rad = self.kinematics.get_joint_limits()
        self.visual_origins = self.kinematics.get_visual_origins()
        
        # Pozycja startowa
        self.standby_angles = np.array([0] + [np.radians(a) for a in config.STANDBY_ANGLES_DEG])
        self.current_orientation = None # Zostanie ustawione w go_standby

        # Inicjalizacja GUI
        self.init_gui()
        
        # Inicjalizacja Wątku Roboczego
        self.init_worker()
        
        print("Aplikacja gotowa.")

    def init_gui(self):
        """Buduje cały interfejs użytkownika."""
        self.setWindowTitle("PAROL6 3D GUI – sterowanie XYZ + UART")
        main_layout = QtWidgets.QHBoxLayout(self)
        
        control_widget = QtWidgets.QWidget()
        control_widget.setFixedWidth(240)
        control_layout = QtWidgets.QVBoxLayout(control_widget)

        # === UART ===
        uart_group = QtWidgets.QGroupBox("Komunikacja UART")
        uart_layout = QtWidgets.QVBoxLayout()
        self.port_combo = QtWidgets.QComboBox()
        try:
            import serial.tools.list_ports
            available_ports = [port.device for port in serial.tools.list_ports.comports()]
        except ImportError:
            available_ports = []
        self.port_combo.addItems(available_ports if available_ports else ["Brak portów"])
        uart_layout.addWidget(QtWidgets.QLabel("Port COM:"))
        uart_layout.addWidget(self.port_combo)
        self.uart_status_label = QtWidgets.QLabel("Status: Gotowy do wysyłania")
        self.uart_status_label.setStyleSheet("color: green;")
        uart_layout.addWidget(self.uart_status_label)
        uart_group.setLayout(uart_layout)
        control_layout.addWidget(uart_group)

        # === XYZ ===
        xyz_group = QtWidgets.QGroupBox("Cel efektora (mm) [Model Użytkownika]")
        xyz_layout = QtWidgets.QGridLayout()
        self.xyz_inputs = {}
        for i, label in enumerate(['X', 'Y', 'Z']):
            lbl = QtWidgets.QLabel(f"{label}:")
            inp = QtWidgets.QLineEdit()
            validator = QtGui.QDoubleValidator(config.RANGES_XYZ[i][0], config.RANGES_XYZ[i][1], 3)
            validator.setNotation(QtGui.QDoubleValidator.StandardNotation)
            inp.setValidator(validator)
            inp.setText(f"{config.DEFAULT_XYZ_GUI[i]:.1f}")
            xyz_layout.addWidget(lbl, i, 0)
            xyz_layout.addWidget(inp, i, 1)
            self.xyz_inputs[label] = inp
        self.set_xyz_btn = QtWidgets.QPushButton("Ustaw Cel (XYZ)")
        xyz_layout.addWidget(self.set_xyz_btn, 3, 0, 1, 2)
        xyz_group.setLayout(xyz_layout)
        control_layout.addWidget(xyz_group)

        # === RPY ===
        rpy_group = QtWidgets.QGroupBox("Orientacja chwytaka (°)")
        rpy_layout = QtWidgets.QGridLayout()
        self.rpy_inputs = {}
        for i, label in enumerate(['Roll', 'Pitch', 'Yaw']):
            lbl = QtWidgets.QLabel(f"{label}:")
            inp = QtWidgets.QLineEdit()
            # === POPRAWKA BŁĘDU ===
            # Poprzednio: QtGui.QDoubleValidator(config.RANGES_RPY, config.RANGES_RPY, 2)
            # Poprawnie: Rozpakuj krotkę na dwa argumenty
            validator = QtGui.QDoubleValidator(config.RANGES_RPY[0], config.RANGES_RPY[1], 2)
            # =======================
            inp.setValidator(validator)
            inp.setText("0.0")
            rpy_layout.addWidget(lbl, i, 0)
            rpy_layout.addWidget(inp, i, 1)
            self.rpy_inputs[label] = inp
        self.set_rpy_btn = QtWidgets.QPushButton("Ustaw orientację")
        rpy_layout.addWidget(self.set_rpy_btn, 3, 0, 1, 2)
        rpy_group.setLayout(rpy_layout)
        control_layout.addWidget(rpy_group)

        # === SUWAKI STAWÓW ===
        sliders_group = QtWidgets.QGroupBox("Sterowanie Stawami (°)")
        sliders_layout = QtWidgets.QGridLayout()
        self.joint_sliders = []
        self.joint_labels = []
        for i in range(6): # Dla 6 stawów
            lbl_name = QtWidgets.QLabel(f"L{i+1}:")
            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            slider.setRange(config.SLIDER_RANGE_MIN, config.SLIDER_RANGE_MAX)
            slider.setValue(self.map_rad_to_slider(0.0, i))
            lbl_value = QtWidgets.QLabel(f"{np.degrees(self.map_slider_to_rad(slider.value(), i)):.1f}°")
            
            sliders_layout.addWidget(lbl_name, i, 0)
            sliders_layout.addWidget(slider, i, 1)
            sliders_layout.addWidget(lbl_value, i, 2)
            
            self.joint_sliders.append(slider)
            self.joint_labels.append(lbl_value)
        sliders_group.setLayout(sliders_layout)
        control_layout.addWidget(sliders_group)

        # === PRZYCISKI ===
        btn_layout = QtWidgets.QHBoxLayout()
        self.standby_btn = QtWidgets.QPushButton("Pozycja standby")
        btn_layout.addWidget(self.standby_btn)
        self.reset_view_btn = QtWidgets.QPushButton("Reset kamery")
        btn_layout.addWidget(self.reset_view_btn)
        control_layout.addLayout(btn_layout)
        control_layout.addStretch()
        main_layout.addWidget(control_widget)

        # === WIDOK 3D ===
        self.view = gl.GLViewWidget()
        self.view.opts['center'] = QVector3D(0, 0, 0.2)
        self.view.opts['msaa'] = True
        ax = gl.GLAxisItem()
        ax.setSize(0.3, 0.3, 0.3)
        self.view.addItem(ax)
        self.view.setCameraPosition(distance=0.7, elevation=30, azimuth=25)
        self.view.setMinimumSize(900, 650)
        self.view.setBackgroundColor((40, 40, 40))
        main_layout.addWidget(self.view)

        # === WCZYTANIE SIATEK 3D ===
        self.links_3d = [] 
        for stl_file in config.STL_FILES:
            try:
                mesh_data = trimesh.load(f'assets/{stl_file}')
                mesh_data.process()
                # === DODATKOWA POPRAWKA na RuntimeWarning ===
                mesh_data.fill_holes() 
                # ==========================================
                mesh_data.fix_normals()
                
                if hasattr(mesh_data.visual, "vertex_colors") and mesh_data.visual.vertex_colors is not None:
                    vc = (mesh_data.visual.vertex_colors.astype(np.float32) / 255.0)[:, :3]
                else:
                    vc = np.ones((len(mesh_data.vertices), 3), dtype=np.float32) * 0.8
                    
                mesh_item = gl.GLMeshItem(
                    vertexes=np.array(mesh_data.vertices),
                    faces=np.array(mesh_data.faces),
                    vertexColors=vc,
                    shader='shaded',
                    smooth=True,
                    drawFaces=True
                )
            except Exception as e:
                print(f"Krytyczny błąd wczytywania siatki '{stl_file}': {e}. Używam placeholdera.")
                mesh_item = gl.GLMeshItem(vertexes=np.array([[0,0,0]]))
            
            self.view.addItem(mesh_item)
            self.links_3d.append(mesh_item)
        print(f"Załadowano {len(self.links_3d)} siatek.")

        # === PODPIĘCIA SYGNAŁÓW ===
        self.set_xyz_btn.clicked.connect(self.schedule_update)
        self.set_rpy_btn.clicked.connect(self.schedule_update)
        for inp in self.xyz_inputs.values():
            inp.returnPressed.connect(self.schedule_update)
        for inp in self.rpy_inputs.values():
            inp.returnPressed.connect(self.schedule_update)

        for i in range(6):
            self.joint_sliders[i].valueChanged.connect(
                functools.partial(self.update_slider_label, joint_index=i)
            )
            self.joint_sliders[i].sliderReleased.connect(self.perform_update_from_sliders)

        self.standby_btn.clicked.connect(self.go_standby)
        self.reset_view_btn.clicked.connect(self.reset_camera)

        # Harmonogram (debounce)
        self.update_pending = False

    def init_worker(self):
        """Tworzy i uruchamia wątek roboczy do obsługi animacji i UART."""
        self.worker_thread = QThread()
        self.worker = RobotWorker(self.standby_angles)
        self.worker.moveToThread(self.worker_thread)

        # === Podłącz sygnały Z workera DO GUI ===
        # Gdy worker zaktualizuje kąty, GUI przerysuje model 3D
        self.worker.angles_updated.connect(self.apply_transforms_from_angles)
        # Gdy worker zmieni status, GUI zaktualizuje etykietę
        self.worker.status_updated.connect(self.update_status_label)

        # === Podłącz sygnały Z GUI DO workera ===
        # Do uruchamiania ruchu
        self.start_move_signal.connect(self.worker.start_move)
        # Do ustawiania portu COM
        self.set_port_signal.connect(self.worker.set_com_port)
        
        # Gdy GUI zmieni port, wyślij sygnał do workera
        self.port_combo.currentTextChanged.connect(self.set_port_signal.emit)
        
        # Uruchom wątek
        self.worker_thread.start()
        
        # Natychmiast ustaw port COM w wątku roboczym
        self.set_port_signal.emit(self.port_combo.currentText())
        
        # Po uruchomieniu, idź do standby
        QtCore.QTimer.singleShot(100, self.go_standby) 

    def closeEvent(self, event):
        """Wywoływane przy zamykaniu okna."""
        print("Zamykanie aplikacji...")
        self.worker_thread.quit()
        self.worker_thread.wait(2000) # Poczekaj 2 sekundy na wątek
        comm.close_serial_port() # Zamknij port
        event.accept()

    # ------------------------
    # Mapowanie Suwaków (GUI)
    # ------------------------
    def map_slider_to_rad(self, slider_val, joint_index):
        min_rad, max_rad = self.joint_limits_rad[joint_index]
        percentage = (slider_val - config.SLIDER_RANGE_MIN) / (config.SLIDER_RANGE_MAX - config.SLIDER_RANGE_MIN)
        return min_rad + percentage * (max_rad - min_rad)

    def map_rad_to_slider(self, rad_val, joint_index):
        min_rad, max_rad = self.joint_limits_rad[joint_index]
        rad_range = max_rad - min_rad
        if rad_range == 0: return (config.SLIDER_RANGE_MAX - config.SLIDER_RANGE_MIN) // 2
        percentage = (rad_val - min_rad) / rad_range
        val = config.SLIDER_RANGE_MIN + percentage * (config.SLIDER_RANGE_MAX - config.SLIDER_RANGE_MIN)
        return int(np.clip(val, config.SLIDER_RANGE_MIN, config.SLIDER_RANGE_MAX))

    # ------------------------
    # Sercowe funkcje GUI
    # ------------------------

    @pyqtSlot(np.ndarray)
    def apply_transforms_from_angles(self, angles):
        """Slot: Aktualizuje widok 3D na podstawie kątów z workera."""
        if not self.links_3d or len(self.links_3d) != len(config.LINK_NAMES):
            return

        transforms = self.kinematics.forward_kinematics_full(angles)
        
        if len(transforms) < len(config.LINK_NAMES):
             return

        for i, name in enumerate(config.LINK_NAMES):
            mesh = self.links_3d[i]
            T = transforms[i]
            
            if name in self.visual_origins:
                xyz_offset, rpy_offset = self.visual_origins[name]
                T_origin = np.eye(4)
                T_origin[:3, :3] = R.from_euler('xyz', rpy_offset).as_matrix()
                T_origin[:3, 3] = xyz_offset
                T = T @ T_origin
                
            T_scene = config.S_MATRIX @ T
            mesh.setTransform(matrix_to_qtransform(T_scene))
        
        self.view.update()

    def read_target_pose_inputs(self):
        """
        Odczytuje XYZ z GUI (układ użytkownika) i
        transformuje do układu robota (obrót o -90 stopni).
        """
        X_user_mm = parse_float_from_input(self.xyz_inputs['X'], default_val=0.0)
        Y_user_mm = parse_float_from_input(self.xyz_inputs['Y'], default_val=0.0)
        Z_user_mm = parse_float_from_input(self.xyz_inputs['Z'], default_val=250.0) 

        # Transformacja:
        # X_robota =  Y_użytkownika
        # Y_robota = -X_użytkownika
        # Z_robota =  Z_użytkownika
        X_robot_m =  Y_user_mm / 1000.0
        Y_robot_m = -X_user_mm / 1000.0
        Z_robot_m =  Z_user_mm / 1000.0
            
        return np.array([X_robot_m, Y_robot_m, Z_robot_m])

    # ------------------------
    # Sprzężenie zwrotne (GUI)
    # ------------------------

    @pyqtSlot(str, str)
    def update_status_label(self, text, color):
        """Slot: Aktualizuje etykietę statusu."""
        self.uart_status_label.setText(text)
        self.uart_status_label.setStyleSheet(f"color: {color};")

    def update_sliders_from_angles(self, angles_rad):
        """Aktualizuje suwaki na podstawie kątów (rad)."""
        for i in range(6):
            rad_val = angles_rad[i+1] # Pomiń bazę
            slider_val = self.map_rad_to_slider(rad_val, i)
            
            self.joint_sliders[i].blockSignals(True)
            self.joint_sliders[i].setValue(slider_val)
            self.joint_sliders[i].blockSignals(False)
            
            self.joint_labels[i].setText(f"{np.degrees(rad_val):.1f}°")

    def update_rpy_inputs_from_matrix(self, matrix):
        """Aktualizuje pola RPY na podstawie macierzy."""
        try:
            r = R.from_matrix(matrix[:3, :3])
            # TODO: Obrót RPY
            roll, pitch, yaw = r.as_euler('xyz', degrees=True) 
            
            self.rpy_inputs['Roll'].blockSignals(True)
            self.rpy_inputs['Pitch'].blockSignals(True)
            self.rpy_inputs['Yaw'].blockSignals(True)
            self.rpy_inputs['Roll'].setText(f"{roll:.2f}")
            self.rpy_inputs['Pitch'].setText(f"{pitch:.2f}")
            self.rpy_inputs['Yaw'].setText(f"{yaw:.2f}")
        except Exception as e:
            print(f"Błąd konwersji macierzy na RPY: {e}")
        finally:
            self.rpy_inputs['Roll'].blockSignals(False)
            self.rpy_inputs['Pitch'].blockSignals(False)
            self.rpy_inputs['Yaw'].blockSignals(False)

    def update_gui_feedback(self, fk_matrix):
        """Aktualizuje pola XYZ i RPY na podstawie macierzy FK (z układu robota)."""
        # 1. Zaktualizuj RPY
        self.update_rpy_inputs_from_matrix(fk_matrix)
        
        # 2. Zaktualizuj XYZ
        pos_robota_m = fk_matrix[:3, 3] # Pozycja w układzie robota
        
        # 3. Transformuj z powrotem do układu użytkownika
        # X_użytkownika = -Y_robota
        # Y_użytkownika =  X_robota
        X_user_m = -pos_robota_m[1]
        Y_user_m =  pos_robota_m[0]
        Z_user_m =  pos_robota_m[2]
            
        # 4. Ustaw tekst w polach
        self.xyz_inputs['X'].blockSignals(True)
        self.xyz_inputs['Y'].blockSignals(True)
        self.xyz_inputs['Z'].blockSignals(True)
        self.xyz_inputs['X'].setText(f"{X_user_m * 1000.0:.1f}")
        self.xyz_inputs['Y'].setText(f"{Y_user_m * 1000.0:.1f}")
        self.xyz_inputs['Z'].setText(f"{Z_user_m * 1000.0:.1f}")
        self.xyz_inputs['X'].blockSignals(False)
        self.xyz_inputs['Y'].blockSignals(False)
        self.xyz_inputs['Z'].blockSignals(False)

    # ------------------------
    # Akcje Użytkownika (GUI)
    # ------------------------
    def perform_update_robot(self):
        """Akcja: Oblicz IK (z pól XYZ)."""
        target_pos_m = self.read_target_pose_inputs()

        # TODO: Logika RPY również musi być obrócona
        roll_deg = parse_float_from_input(self.rpy_inputs['Roll'], default_val=0.0)
        pitch_deg = parse_float_from_input(self.rpy_inputs['Pitch'], default_val=0.0)
        yaw_deg = parse_float_from_input(self.rpy_inputs['Yaw'], default_val=0.0)
        
        try:
            rot = R.from_euler('xyz', [np.radians(roll_deg), np.radians(pitch_deg), np.radians(yaw_deg)])
            R_mat = rot.as_matrix()
            
            target_transform = np.eye(4)
            target_transform[:3, :3] = R_mat
            target_transform[:3, 3] = target_pos_m
        except Exception as e:
            print(f"Błąd budowania macierzy rotacji: {e}")
            self.update_status_label("Status: Błąd orientacji", "red")
            return
        
        print(f"Próba IK dla celu [m] (układ robota): {np.round(target_pos_m, 3)}")
        
        try:
            # Pobierz aktualne kąty z workera jako pozycję startową
            initial_pos = self.worker.get_previous_angles()
            angles_rad = self.kinematics.inverse_kinematics(
                target_pos_m,
                target_transform[:3, :3],
                initial_pos
            )
            print("IK znalezione (rad):", np.round(angles_rad[1:], 5))
        except Exception as e:
            print(f"Błąd IK: {e}")
            self.update_status_label("Status: Błąd IK", "red")
            return

        self.current_orientation = target_transform.copy()
        
        # Zaktualizuj suwaki natychmiast
        self.update_sliders_from_angles(angles_rad)
        
        # Wyślij polecenie ruchu do workera
        self.start_move_signal.emit(angles_rad)

    def perform_update_from_sliders(self):
        """Akcja: Oblicz FK (z suwaków)."""
        angles_rad_list = [0.0]
        for i in range(6):
            slider_val = self.joint_sliders[i].value()
            rad_val = self.map_slider_to_rad(slider_val, i)
            angles_rad_list.append(rad_val)
        
        target_angles_from_sliders = np.array(angles_rad_list)
        
        fk_matrix = self.kinematics.forward_kinematics(target_angles_from_sliders)
        self.current_orientation = fk_matrix.copy()
        
        # Zaktualizuj pola XYZ/RPY
        self.update_gui_feedback(self.current_orientation)
        
        # Wyślij polecenie ruchu do workera
        self.start_move_signal.emit(target_angles_from_sliders)

    def update_slider_label(self, value, joint_index):
        """Tylko aktualizuje etykietę obok suwaka podczas przeciągania."""
        rad_val = self.map_slider_to_rad(value, joint_index)
        self.joint_labels[joint_index].setText(f"{np.degrees(rad_val):.1f}°")

    # ------------------------
    # Harmonogram debouncing (dla XYZ)
    # ------------------------
    def schedule_update(self):
        if self.update_pending: return
        self.update_pending = True
        QtCore.QTimer.singleShot(config.DEBOUNCE_MS, self.perform_update_and_clear_pending)

    def perform_update_and_clear_pending(self):
        self.perform_update_robot()
        self.update_pending = False

    # ------------------------
    # Akcje Przycisków (GUI)
    # ------------------------
    def go_standby(self):
        target_angles = self.standby_angles.copy()
        fk = self.kinematics.forward_kinematics(target_angles)
        self.current_orientation = fk.copy()
        
        # Pełne sprzężenie zwrotne
        self.update_gui_feedback(self.current_orientation) 
        self.update_sliders_from_angles(target_angles) 
        
        # Wyślij polecenie ruchu do workera
        self.start_move_signal.emit(target_angles)
        self.update_status_label("Status: Standby", "blue")

    def reset_camera(self):
        self.view.setCameraPosition(distance=0.7, elevation=30, azimuth=25)

# ------------------------
# Uruchomienie Aplikacji
# ------------------------
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())

