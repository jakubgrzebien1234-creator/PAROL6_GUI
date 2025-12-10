# Plik: robot/gui.py

import sys
import functools
import numpy as np
import trimesh
from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtCore import QThread, pyqtSlot, pyqtSignal, QTimer
from PyQt5.QtGui import QMatrix4x4, QVector4D, QVector3D, QTextFormat, QColor, QFont
from PyQt5.QtWidgets import QLabel, QGroupBox, QFormLayout
import pyqtgraph.opengl as gl
from OpenGL import GL
from scipy.spatial.transform import Rotation as R

# === IMPORTY LOKALNYCH MODUŁÓW ===
import robot.config as config
from robot.kinematics import RobotKinematics
from robot.worker import RobotWorker
try:
    import robot.communication as comm
except ImportError:
    from robot.robot_worker import comm  # Atrapa

# =Aplikacja korzysta również z `config.py` i `kinematics.py`
# ======================================

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
    
    # === NOWY SYGNAŁ DLA RUCHU LINIOWEGO ===
    start_linear_move_signal = pyqtSignal(np.ndarray, np.ndarray)

    # Sygnał do ustawienia portu COM w wątku roboczym
    set_port_signal = pyqtSignal(str)
    # Sygnał do uruchomienia Homing w wątku roboczym
    request_homing_signal = pyqtSignal()
    
    # Sygnał dla chwytaka
    request_gripper_signal = pyqtSignal(str)
    
    # === NOWE SYGNAŁY DLA PROGRAMATORA ===
    start_program_signal = pyqtSignal(str)
    stop_program_signal = pyqtSignal()


    def __init__(self):
        super().__init__()
        
        # Inicjalizacja silnika kinematyki
        self.kinematics = RobotKinematics(config.URDF_PATH, config.ACTIVE_LINKS_MASK)
        self.joint_limits_rad = self.kinematics.get_joint_limits()
        self.visual_origins = self.kinematics.get_visual_origins()
        
        # Pozycja startowa
        self.standby_angles = np.array([0] + [np.radians(a) for a in config.STANDBY_ANGLES_DEG])
        self.current_orientation = None # Zostanie ustawione w go_standby

        self.gripper_state = False # False = VAC_OFF, True = VAC_ON

        self.l6_color_state = 0 # 0 = domyślny, 1 = zmieniony
        
        self.default_link_colors = {} 

        # Lista na etykiety J1-J6 (teraz tworzone w _create_right_readout_panel)
        self.joint_value_labels = []

        # Inicjalizacja GUI
        self.init_gui()
        
        # Inicjalizacja Wątku Roboczego
        self.init_worker()
        
        # Usunięto j6_timer - jest już obsługiwany przez główny sygnał angles_updated
        
        print("Aplikacja gotowa.")

    def init_gui(self):
        """Buduje cały interfejs użytkownika."""
        self.setWindowTitle("PAROL6 3D GUI – sterowanie XYZ + UART")
        
        # === GŁÓWNY LAYOUT (POZIOMY) ===
        main_layout = QtWidgets.QHBoxLayout(self)
        
        # --- 1. LEWY PANEL (STEROWANIE) ---
        control_widget = self._create_left_control_panel()
        main_layout.addWidget(control_widget)

        # --- 2. ŚRODKOWY PANEL (WIZUALIZACJA 3D) ---
        self.view = self._create_3d_visualization_panel()
        main_layout.addWidget(self.view)

        # --- 3. PRAWY PANEL (ODCZYTY KĄTÓW) ---
        right_readout_panel = self._create_right_readout_panel()
        main_layout.addWidget(right_readout_panel)

        # === USTAWIANIE ROZCIĄGANIA ===
        main_layout.setStretch(0, 0) # Lewy panel (stały)
        main_layout.setStretch(1, 1) # Wizualizacja 3D (rozciągliwy)
        main_layout.setStretch(2, 0) # Prawy panel (stały)

        # === WCZYTANIE SIATEK 3D ===
        self._load_3d_meshes()

        # === PODPIĘCIA SYGNAŁÓW ===
        self._connect_gui_signals()

        # Harmonogram (debounce)
        self.update_pending = False

    def _create_left_control_panel(self):
        """Tworzy cały lewy panel sterowania z zakładkami."""
        
        # --- GŁÓWNY KONTENER KONTROLEK (LEWY PANEL) ---
        control_widget = QtWidgets.QWidget()
        control_widget.setFixedWidth(240)
        control_layout = QtWidgets.QVBoxLayout(control_widget)

        # --- SEKCJA PRZYCISKÓW GÓRNYCH (HOME + GRIPPER) ---
        self.home_button = QtWidgets.QPushButton("HOME")
        self.home_button.setFixedSize(60, 60)
        self.home_button.setStyleSheet(
            "font-weight: bold; background-color: #f7e1e1; border: 1px solid #c00;"
        )
        
        self.gripper_button = QtWidgets.QPushButton("VACCUM\nGRIPPER")
        self.gripper_button.setFixedSize(60, 60)
        self.gripper_button.setStyleSheet(
            "font-weight: bold; background-color: #f0f0f0; border: 1px solid #999;"
        )
        
        top_button_layout = QtWidgets.QHBoxLayout()
        top_button_layout.addStretch()
        top_button_layout.addWidget(self.gripper_button)
        top_button_layout.addWidget(self.home_button)
        
        control_layout.addLayout(top_button_layout)

        # --- SEKCJA ZAKŁADEK ---
        self.tab_widget = QtWidgets.QTabWidget()
        control_layout.addWidget(self.tab_widget)

        # === ZAKŁADKA 1: Sterowanie Ręczne ===
        manual_tab = QtWidgets.QWidget()
        manual_layout = QtWidgets.QVBoxLayout(manual_tab)
        
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
        manual_layout.addWidget(uart_group)

        # === XYZ ===
        xyz_group = QtWidgets.QGroupBox("Cel efektora (mm)")
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
        manual_layout.addWidget(xyz_group)

        # === RPY ===
        rpy_group = QtWidgets.QGroupBox("Orientacja chwytaka (°)")
        rpy_layout = QtWidgets.QGridLayout()
        self.rpy_inputs = {}
        for i, label in enumerate(['Roll', 'Pitch', 'Yaw']):
            lbl = QtWidgets.QLabel(f"{label}:")
            inp = QtWidgets.QLineEdit()
            validator = QtGui.QDoubleValidator(config.RANGES_RPY[0], config.RANGES_RPY[1], 2)
            inp.setValidator(validator)
            inp.setText("0.0")
            rpy_layout.addWidget(lbl, i, 0)
            rpy_layout.addWidget(inp, i, 1)
            self.rpy_inputs[label] = inp
        self.set_rpy_btn = QtWidgets.QPushButton("Ustaw orientację")
        rpy_layout.addWidget(self.set_rpy_btn, 3, 0, 1, 2)
        rpy_group.setLayout(rpy_layout)
        manual_layout.addWidget(rpy_group)

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
        manual_layout.addWidget(sliders_group)

        # === PRZYCISKI (dolne) ===
        btn_layout = QtWidgets.QHBoxLayout()
        self.standby_btn = QtWidgets.QPushButton("Pozycja standby")
        btn_layout.addWidget(self.standby_btn)
        self.reset_view_btn = QtWidgets.QPushButton("Reset kamery")
        btn_layout.addWidget(self.reset_view_btn)
        
        self.color_change_btn = QtWidgets.QPushButton("Zmień kolor L6")
        btn_layout.addWidget(self.color_change_btn)
        
        manual_layout.addLayout(btn_layout)
        manual_layout.addStretch()
        
        self.tab_widget.addTab(manual_tab, "Sterowanie")

        # === ZAKŁADKA 2: Programator ===
        program_tab = QtWidgets.QWidget()
        program_layout = QtWidgets.QVBoxLayout(program_tab)
        
        program_group = QtWidgets.QGroupBox("Edytor Programu")
        program_group_layout = QtWidgets.QVBoxLayout()
        
        self.program_editor = QtWidgets.QPlainTextEdit()
        self.program_editor.setFont(QtGui.QFont("Courier", 10))
        self.program_editor.setPlaceholderText(
            "MOVE(100, 50, 200)  # X, Y, Z w mm\n"
            "MOVEZ(150)\n"
            "DELAY(1000)        # Czas w ms\n"
            "MOVESTB            # Powrót do standby"
        )
        program_group_layout.addWidget(self.program_editor)
        
        program_btn_layout = QtWidgets.QHBoxLayout()
        self.run_program_btn = QtWidgets.QPushButton("Uruchom Program")
        self.run_program_btn.setStyleSheet("font-weight: bold; background-color: #e1f7e1; border: 1px solid #0c0;")
        self.stop_program_btn = QtWidgets.QPushButton("STOP")
        self.stop_program_btn.setStyleSheet("font-weight: bold; background-color: #f7e1e1; border: 1px solid #c00;")
        self.stop_program_btn.setEnabled(False)
        
        program_btn_layout.addWidget(self.run_program_btn)
        program_btn_layout.addWidget(self.stop_program_btn)
        program_group_layout.addLayout(program_btn_layout)
        
        program_group.setLayout(program_group_layout)
        program_layout.addWidget(program_group)
        program_layout.addStretch()
        
        self.tab_widget.addTab(program_tab, "Program")

        # --- KONIEC ZAKŁADEK ---
        
        control_layout.addStretch()
        return control_widget

    def _create_3d_visualization_panel(self):
        """Tworzy widget wizualizacji 3D."""
        view = gl.GLViewWidget()
        view.opts['center'] = QVector3D(0, 0, 0.2)
        view.opts['msaa'] = True
        ax = gl.GLAxisItem()
        ax.setSize(0.3, 0.3, 0.3)
        view.addItem(ax)
        view.setCameraPosition(distance=0.7, elevation=30, azimuth=25)
        view.setMinimumSize(900, 650)
        view.setBackgroundColor((40, 40, 40))
        return view

    def _create_right_readout_panel(self):
        """Tworzy prawy panel z odczytami kątów J1-J6."""
        
        readout_layout = QFormLayout()
        self.joint_value_labels = [] 
        for i in range(6):
            label_value = QLabel("0.0°")
            label_value.setFont(QFont("Monospace", 10)) # Ustaw czcionkę
            self.joint_value_labels.append(label_value)
            readout_layout.addRow(f"J{i+1}:", self.joint_value_labels[i])

        readout_groupbox = QGroupBox("Aktualne Kąty")
        readout_groupbox.setLayout(readout_layout)
        
        readout_groupbox.setFixedWidth(120) 

        v_layout = QtWidgets.QVBoxLayout()
        v_layout.addWidget(readout_groupbox)
        v_layout.addStretch()
        
        container_widget = QtWidgets.QWidget()
        container_widget.setLayout(v_layout)
        container_widget.setFixedWidth(130) 
        
        return container_widget

    def _load_3d_meshes(self):
        """Wczytuje siatki 3D i dodaje je do widoku."""
        self.links_3d = [] 
        for i, stl_file in enumerate(config.STL_FILES):
            try:
                print(f"[DEBUG] Wczytuję plik: 'assets/{stl_file}'...")
                
                mesh_data = trimesh.load(f'assets/{stl_file}')
                
                if not isinstance(mesh_data, trimesh.Trimesh):
                    print(f"KRYTYCZNY BŁĄD: {stl_file} nie jest siatką (nie ma ścian)! To chmura punktów.")
                    raise ValueError(f"{stl_file} loaded as PointCloud, not Trimesh.")

                mesh_data.process()
                mesh_data.remove_degenerate_faces()
                mesh_data.fix_normals()
                mesh_data.fill_holes()

                if len(mesh_data.faces) == 0:
                    print(f"KRYTYCZNY BŁĄD: {stl_file} nie ma żadnych poprawnych ścian po naprawie.")
                    raise ValueError(f"{stl_file} has no valid faces after repair.")

                print(f"[DEBUG]... {stl_file} OK. Wierzchołki: {len(mesh_data.vertices)}, Ściany: {len(mesh_data.faces)}")

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
                vc = np.array([[0.8, 0.8, 0.8]]) 
            
            self.default_link_colors[i] = vc 
            self.view.addItem(mesh_item)
            self.links_3d.append(mesh_item)
            
        print(f"Załadowano {len(self.links_3d)} siatek.")

    def _connect_gui_signals(self):
        """Podpina wszystkie sygnały GUI do slotów."""
        # (Sterowanie ręczne)
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
        
        self.color_change_btn.clicked.connect(self.change_link_color)

        self.home_button.clicked.connect(self.request_homing_signal.emit)
        
        self.gripper_button.clicked.connect(self.on_gripper_click)
        
        # (Programator)
        self.run_program_btn.clicked.connect(self.on_run_program)
        self.stop_program_btn.clicked.connect(self.on_stop_program)

    def init_worker(self):
        """Tworzy i uruchamia wątek roboczy do obsługi animacji i UART."""
        self.worker_thread = QThread()
        
        self.worker = RobotWorker(self.standby_angles, self.kinematics)
        self.worker.moveToThread(self.worker_thread)
        
        self.worker.angles_updated.connect(self.apply_transforms_from_angles)
        self.worker.angles_updated.connect(self.update_joint_value_labels) 
        
        self.worker.status_updated.connect(self.update_status_label)
        self.worker.limit_switch_status.connect(self.on_limit_switch_hit)
        
        self.worker.program_finished.connect(self.on_program_finished)
        self.worker.program_line_highlight.connect(self.on_program_line_highlight)

        self.start_move_signal.connect(self.worker.start_move)
        self.start_linear_move_signal.connect(self.worker.start_linear_move)
        self.set_port_signal.connect(self.worker.set_com_port)
        self.request_homing_signal.connect(self.worker.start_homing)
        self.request_gripper_signal.connect(self.worker.set_gripper_state)
        
        self.start_program_signal.connect(self.worker.start_program)
        self.stop_program_signal.connect(self.worker.stop_program)
        
        self.port_combo.currentTextChanged.connect(self.set_port_signal.emit)
        
        self.worker_thread.start()
        
        self.set_port_signal.emit(self.port_combo.currentText())
        
        QtCore.QTimer.singleShot(100, self.go_standby)

    def closeEvent(self, event):
        """Wywoływane przy zamykaniu okna."""
        print("Zamykanie aplikacji...")
        self.stop_program_signal.emit() # Zatrzymaj program jeśli działa
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
        OFFSET_X = 0.0      
        OFFSET_Y = -90.0    
        OFFSET_Z = 100.0    

        X_user_mm += OFFSET_X
        Y_user_mm += OFFSET_Y
        Z_user_mm += OFFSET_Z

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

    @pyqtSlot(np.ndarray)
    def update_joint_value_labels(self, angles_rad):
        """
        Aktualizuje QLabel J1-J6 (w prawym panelu) na podstawie kątów w radianach.
        angles_rad: np.array([0.0, J1, J2, ..., J6])
        """
        if not self.joint_value_labels: # Sprawdź czy etykiety są gotowe
            return
            
        for i in range(6):
            deg = np.degrees(angles_rad[i])  # pomijamy bazę 0
            self.joint_value_labels[i].setText(f"{deg:.1f}°")



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
        
        # 3. Transformuj 
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
        """
        Akcja: Przygotowuje ruch LINIOWY (LIN).
        Definiuje START i KONIEC ścieżki dla workera.
        """
        
        # 1. Oblicz docelową transformację (KONIEC) z pól GUI
        target_pos_m = self.read_target_pose_inputs()

        # Odczytaj RPY
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
            print(f"Błąd budowania macierzy rotacji celu: {e}")
            self.update_status_label("Status: Błąd orientacji", "red")
            return
        
        # 2. Oblicz aktualną transformację (START) na podstawie kątów z workera
        try:
            start_angles = self.worker.get_previous_angles()
            start_transform = self.kinematics.forward_kinematics(start_angles)
        except Exception as e:
            print(f"Błąd obliczania pozycji startowej (FK): {e}")
            self.update_status_label("Status: Błąd FK", "red")
            return

        # 3. Wyślij sygnał do workera, aby rozpoczął ruch LINIOWY
        print("GUI: Żądanie ruchu LIN (Start -> Cel)")
        self.start_linear_move_signal.emit(start_transform, target_transform)

    def perform_update_from_sliders(self):
        """Akcja: Oblicz FK (z suwaków) i wykonaj ruch PTP."""
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
        
        # Wyślij polecenie ruchu do workera (ruch PTP)
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
        """Przechodzi do pozycji standby (ruch PTP)"""
        target_angles = self.standby_angles.copy()
        fk = self.kinematics.forward_kinematics(target_angles)
        self.current_orientation = fk.copy()
        
        # Pełne sprzężenie zwrotne
        self.update_gui_feedback(self.current_orientation) 
        self.update_sliders_from_angles(target_angles) 
        
        # Wyślij polecenie ruchu do workera (ruch PTP)
        self.start_move_signal.emit(target_angles)
        self.update_status_label("Status: Standby", "blue")

    def reset_camera(self):
        self.view.setCameraPosition(distance=0.7, elevation=30, azimuth=25)

    @pyqtSlot()
    def on_gripper_click(self):
        """
        Przełącza stan chwytaka (VAC_ON / VAC_OFF) i wysyła sygnał do workera.
        """
        self.gripper_state = not self.gripper_state
        
        if self.gripper_state:
            command = "VAC_ON"
            self.gripper_button.setStyleSheet(
                "font-weight: bold; background-color: #e1f7e1; border: 1px solid #0c0;"
            )
        else:
            command = "VAC_OFF"
            self.gripper_button.setStyleSheet(
                "font-weight: bold; background-color: #f0f0f0; border: 1px solid #999;"
            )
            
        print(f"Wysyłanie polecenia chwytaka: {command}")
        self.request_gripper_signal.emit(command)

    def set_link_color(self, link_index_3d, color_rgb):
        """
        Ustawia kolor dla konkretnej siatki (link_index_3d) na dany kolor [R,G,B].
        """
        try:
            target_mesh = self.links_3d[link_index_3d]
        except IndexError:
            print(f"Błąd: Siatka o indeksie {link_index_3d} nie istnieje.")
            return

        try:
            current_vertexes = target_mesh.vertexes
            current_faces = target_mesh.faces
            
            if current_vertexes is None or current_faces is None:
                print("Błąd: Brak danych wierzchołków lub ścian w obiekcie.")
                return
            num_verts = current_vertexes.shape[0]
        except Exception as e:
            print(f"Błąd odczytu wierzchołków: {e}")
            return
            
        if isinstance(color_rgb, list):
            new_color_array = np.tile(color_rgb, (num_verts, 1)).astype(np.float32)
        elif isinstance(color_rgb, np.ndarray):
            new_color_array = color_rgb
            if new_color_array.shape[0] != num_verts:
                print(f"Uwaga: Niezgodność liczby kolorów ({new_color_array.shape[0]}) i wierzchołków ({num_verts}). Używam pierwszego koloru.")
                new_color = new_color_array[0, :3]
                new_color_array = np.tile(new_color, (num_verts, 1)).astype(np.float32)
        else:
            print("Błąd: Nieprawidłowy format koloru.")
            return

        target_mesh.setMeshData(
            vertexes=current_vertexes,
            faces=current_faces,
            vertexColors=new_color_array
        )

    @pyqtSlot(int, bool)
    def on_limit_switch_hit(self, joint_index, is_hit):
        """
        Slot wywoływany, gdy worker wykryje zmianę stanu krańcówki.
        joint_index: 0-5 (dla L1-L6)
        is_hit: True (wciśnięta), False (zwolniona)
        """
        
        link_index_3d = joint_index + 1
        
        if link_index_3d >= len(self.links_3d):
             print(f"Ignorowanie krańcówki: brak siatki dla indeksu {link_index_3d}")
             return

        if is_hit:
            print(f"Krańcówka L{joint_index+1} WCIŚNIĘTA. Zmieniam kolor na zielony.")
            self.set_link_color(link_index_3d, [0.1, 0.9, 0.1]) # Jasny zielony
        else:
            print(f"Krańcówka L{joint_index+1} ZWOLNIONA. Przywracam kolor.")
            default_color_data = self.default_link_colors.get(link_index_3d)
            if default_color_data is not None:
                self.set_link_color(link_index_3d, default_color_data)
            else:
                self.set_link_color(link_index_3d, [0.8, 0.8, 0.8])

    @pyqtSlot()
    def change_link_color(self):
        """
        Zmienia kolor siatki ostatniego ogniwa (L6) po naciśnięciu przycisku.
        """
        link_to_change = 6 # L6_Link (indeks siatki 6)
        
        if link_to_change >= len(self.links_3d):
            print("Błąd: Brak siatki L6 (indeks 6).")
            return
            
        if self.l6_color_state == 0:
            new_color = [1.0, 0.3, 0.3] 
            self.l6_color_state = 1
        else:
            default_color_data = self.default_link_colors.get(link_to_change)
            if default_color_data is None:
                print("Ostrzeżenie: Brak domyślnego koloru dla L6, używam szarego.")
                default_color_data = [0.8, 0.8, 0.8]
                
            new_color = default_color_data
            self.l6_color_state = 0
            
        self.set_link_color(link_to_change, new_color)
        print(f"Zmieniono kolor L6 na stan: {self.l6_color_state}")

    # ------------------------
    # NOWA SEKCJA: Obsługa Programatora
    # ------------------------
    @pyqtSlot()
    def on_run_program(self):
        """Akcja: Wysyła tekst programu do workera."""
        program_text = self.program_editor.toPlainText()
        if not program_text.strip():
            self.update_status_label("Status: Program jest pusty", "orange")
            return
            
        self.run_program_btn.setEnabled(False)
        self.stop_program_btn.setEnabled(True)
        self.tab_widget.setCurrentIndex(0) # Przełącz na zakładkę "Sterowanie"
        self.update_status_label("Status: Wykonywanie programu...", "blue")
        self.start_program_signal.emit(program_text)

    @pyqtSlot()
    def on_stop_program(self):
        """Akcja: Wysyła sygnał STOP do workera."""
        self.stop_program_signal.emit()
        self.update_status_label("Status: Zatrzymywanie...", "orange")

    @pyqtSlot(str, bool)
    def on_program_finished(self, status_msg, was_stopped):
        """Slot: Wywoływany, gdy worker skończy program (lub go zatrzyma)."""
        self.run_program_btn.setEnabled(True)
        self.stop_program_btn.setEnabled(False)
        self.tab_widget.setCurrentIndex(1) # Wróć do programatora
        
        if was_stopped:
            self.update_status_label(f"Status: {status_msg}", "orange")
        else:
            self.update_status_label(f"Status: {status_msg}", "green")
        
        self.on_program_line_highlight(-1) # Wyczyść podświetlenie

    @pyqtSlot(int)
    def on_program_line_highlight(self, line_number):
        """Slot: Podświetla aktualnie wykonywaną linię w edytorze."""
        cursor = self.program_editor.textCursor()
        
        selection = QtWidgets.QTextEdit.ExtraSelection()
        selection.format.setBackground(QtCore.Qt.transparent)
        selection.cursor = cursor
        
        selections = [selection]

        if line_number >= 0:
             # Przejdź do linii
            cursor.movePosition(QtGui.QTextCursor.Start)
            cursor.movePosition(QtGui.QTextCursor.Down, QtGui.QTextCursor.MoveAnchor, line_number)
            
            # Ustaw nowe podświetlenie
            selection = QtWidgets.QTextEdit.ExtraSelection()
            selection.format.setBackground(QColor(60, 60, 40))
            selection.format.setProperty(QTextFormat.FullWidthSelection, True)
            selection.cursor = cursor
            selections.append(selection)

        self.program_editor.setExtraSelections(selections)
        if line_number >= 0:
            self.program_editor.setTextCursor(cursor) # Przesuń widok