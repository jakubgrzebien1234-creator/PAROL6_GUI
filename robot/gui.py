# Plik: robot/gui.py

import sys
import functools
import warnings
import numpy as np
import trimesh
from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtCore import QThread, pyqtSlot, pyqtSignal, QTimer, Qt
from PyQt5.QtGui import QMatrix4x4, QVector4D, QVector3D, QTextFormat, QColor, QFont, QPixmap
from PyQt5.QtWidgets import QLabel, QGroupBox, QFormLayout, QStyle, QDialog, QProgressBar, QVBoxLayout
import pyqtgraph.opengl as gl
from OpenGL import GL
from scipy.spatial.transform import Rotation as R

# === ICON SUPPORT (QTAwesome or Fallback) ===
try:
    import qtawesome as qta
    HAS_QTA = True
except ImportError:
    HAS_QTA = False
    print("Info: Install 'pip install qtawesome' to get nice icons.")

# === LOCAL IMPORTS ===
import robot.config as config
from robot.kinematics import RobotKinematics
from robot.worker import RobotWorker
try:
    import robot.communication as comm
except ImportError:
    from robot.worker import comm  # Mock

# ------------------------
# Helper: Convert 4x4 Matrix -> QMatrix4x4
# ------------------------
def matrix_to_qtransform(matrix):
    m = QMatrix4x4()
    for i in range(4):
        m.setRow(i, QVector4D(float(matrix[i, 0]), float(matrix[i, 1]), float(matrix[i, 2]), float(matrix[i, 3])))
    return m

# ------------------------
# Helper: Safe Float Parsing
# ------------------------
def parse_float_from_input(line_edit, default_val=0.0):
    text = line_edit.text().strip().replace(',', '.')
    if not text or text == "-":
        return default_val
    try:
        return float(text)
    except ValueError:
        print(f"Invalid value in field: '{text}', using default {default_val}")
        return default_val

# ------------------------
# Class: Homing Dialog (Status Indicators)
# ------------------------
class HomingDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent_ref = parent
        self.setWindowTitle("Robot Homing Procedure")
        self.setFixedSize(400, 250)
        
        self.setWindowFlags(Qt.Dialog | Qt.WindowCloseButtonHint | Qt.WindowTitleHint)
        self.setModal(True) 

        # === STYL CSS (POPRAWIONY) ===
        # Używamy selektorów atrybutów [status="..."], co działa pewnie w PyQt
        self.setStyleSheet("""
            QDialog { background-color: #1e1e1e; color: #f0f0f0; border: 1px solid #444; }
            QLabel#TitleLabel { font-size: 20px; font-weight: bold; color: #ffffff; }
            QLabel#DescLabel { font-size: 13px; color: #aaaaaa; }
            
            QPushButton { border-radius: 8px; padding: 12px; font-weight: bold; color: white; border: none; }
            QPushButton#BtnAuto { background-color: #0288d1; }
            QPushButton#BtnAuto:hover { background-color: #039be5; }
            QPushButton#BtnManual { background-color: #2e7d32; color: #e8f5e9; }
            QPushButton#BtnManual:hover { background-color: #388e3c; }

            /* BAZOWY STYL KAFELKA OSI */
            QLabel[role="axis_indicator"] {
                background-color: #333;
                color: #777;
                border: 1px solid #555;
                border-radius: 4px;
                font-weight: bold;
                font-size: 12px;
                qproperty-alignment: AlignCenter;
            }

            /* STAN: HIT (Wciśnięta - Pomarańczowy) */
            QLabel[role="axis_indicator"][status="hit"] {
                background-color: #e65100;
                color: white;
                border: 2px solid #ff9800;
            }

            /* STAN: DONE (Gotowe - Zielony) */
            QLabel[role="axis_indicator"][status="done"] {
                background-color: #2e7d32;
                color: white;
                border: 2px solid #4caf50;
            }

            QProgressBar { border: 2px solid #444; border-radius: 5px; background-color: #2b2b2b; text-align: center; height: 10px; }
            QProgressBar::chunk { background-color: #00e5ff; }
        """)

        # Główny Layout
        self.layout = QVBoxLayout(self)
        self.layout.setSpacing(15)
        self.layout.setContentsMargins(30, 30, 30, 30)

        # Nagłówek
        self.lbl_title = QLabel("System Locked")
        self.lbl_title.setObjectName("TitleLabel")
        self.lbl_title.setAlignment(Qt.AlignCenter)
        self.layout.addWidget(self.lbl_title)

        self.lbl_desc = QLabel("Movement disabled. Initialize homing.")
        self.lbl_desc.setObjectName("DescLabel")
        self.lbl_desc.setAlignment(Qt.AlignCenter)
        self.layout.addWidget(self.lbl_desc)

        # Przyciski
        self.btn_container = QtWidgets.QWidget()
        btn_layout = QVBoxLayout(self.btn_container)
        btn_layout.setContentsMargins(0, 10, 0, 0)
        
        self.btn_start = QtWidgets.QPushButton("🏠  START AUTO-HOMING")
        self.btn_start.setObjectName("BtnAuto")
        self.btn_start.clicked.connect(self.start_auto_homing)
        btn_layout.addWidget(self.btn_start)

        self.btn_confirm = QtWidgets.QPushButton("✅  MANUAL CONFIRM")
        self.btn_confirm.setObjectName("BtnManual")
        self.btn_confirm.clicked.connect(self.manual_confirm)
        btn_layout.addWidget(self.btn_confirm)
        self.layout.addWidget(self.btn_container)

        # Sekcja Postępu
        self.spinner_container = QtWidgets.QWidget()
        spin_layout = QVBoxLayout(self.spinner_container)
        
        self.lbl_status = QLabel("Calibrating...")
        self.lbl_status.setAlignment(Qt.AlignCenter)
        self.lbl_status.setStyleSheet("color: #00e5ff; font-weight: bold;")
        
        self.spinner = QProgressBar()
        self.spinner.setRange(0, 0)
        self.spinner.setTextVisible(False)
        
        spin_layout.addWidget(self.lbl_status)
        spin_layout.addWidget(self.spinner)

        # === WSKAŹNIKI OSI ===
        self.indicators_container = QtWidgets.QWidget()
        ind_layout = QtWidgets.QHBoxLayout(self.indicators_container)
        ind_layout.setSpacing(5)
        ind_layout.setContentsMargins(0, 10, 0, 0)
        
        self.axis_labels = []
        for i in range(6):
            lbl = QLabel(f"J{i+1}")
            lbl.setFixedSize(40, 30)
            # Ustawiamy właściwość 'role', aby CSS wiedział, że to kafelek
            lbl.setProperty("role", "axis_indicator")
            # Ustawiamy stan początkowy
            lbl.setProperty("status", "idle")
            
            self.axis_labels.append(lbl)
            ind_layout.addWidget(lbl)
            
        spin_layout.addWidget(self.indicators_container)
        # =====================

        self.spinner_container.hide()
        self.layout.addWidget(self.spinner_container)
        self.layout.addStretch()

    def set_axis_state(self, axis_idx, state):
        """
        Zmienia wygląd kafelka osi.
        state: 'HIT', 'DONE', 'IDLE'
        """
        if axis_idx < 0 or axis_idx >= 6: return
        
        lbl = self.axis_labels[axis_idx]
        
        # 1. Ustaw tekst
        if state == 'HIT':
            lbl.setText(f"J{axis_idx+1}")
        elif state == 'DONE':
            lbl.setText(f"J{axis_idx+1} ✓")
        else:
            lbl.setText(f"J{axis_idx+1}")
            
        # 2. Ustaw właściwość statusu (zgodnie z CSS)
        # Zamieniamy wielkie litery na małe (HIT -> hit), bo tak zdefiniowaliśmy w CSS
        lbl.setProperty("status", state.lower())
        
        # 3. Wymuś odświeżenie stylu (ważne w PyQt!)
        lbl.style().unpolish(lbl)
        lbl.style().polish(lbl)

    def start_auto_homing(self):
        self.lbl_title.setText("Homing in Progress")
        self.lbl_desc.setText("Detecting limit switches...")
        self.btn_container.hide()
        self.spinner_container.show()
        
        # Reset wskaźników
        for i in range(6): self.set_axis_state(i, 'IDLE')

        if self.parent_ref:
            self.parent_ref.is_homed = False
            self.parent_ref.request_homing_signal.emit()

    def manual_confirm(self):
        reply = QtWidgets.QMessageBox.question(
            self, "Confirm", "Is robot at HOME?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.No
        )
        if reply == QtWidgets.QMessageBox.Yes:
            if self.parent_ref:
                self.parent_ref.is_homed = True
                self.parent_ref.update_status_label("Manual Homing Confirmed", "green")
            self.accept()

    def closeEvent(self, event):
        event.accept()                      
# ------------------------
# Class: Main Window
# ------------------------
class MainWindow(QtWidgets.QWidget):
    
    # --- SIGNALS ---
    start_move_signal = pyqtSignal(np.ndarray)
    start_linear_move_signal = pyqtSignal(np.ndarray, np.ndarray)
    set_port_signal = pyqtSignal(str)
    request_homing_signal = pyqtSignal()
    request_gripper_signal = pyqtSignal(str)
    start_program_signal = pyqtSignal(str)
    stop_program_signal = pyqtSignal()
    change_tool_signal = pyqtSignal(str)
    
    # SPEED SIGNAL
    set_speed_signal = pyqtSignal(float) 

    def __init__(self):
        super().__init__()
        
        # Initialize Kinematics
        self.kinematics = RobotKinematics(config.URDF_PATH, None)
        self.joint_limits_rad = self.kinematics.get_joint_limits()
        self.visual_origins = self.kinematics.get_visual_origins()
        
        # Start Position
        self.standby_angles = np.array([0] + [np.radians(a) for a in config.STANDBY_ANGLES_DEG])
        self.current_orientation = None 

        self.gripper_state = False 
        self.l6_color_state = 0 
        self.default_link_colors = {} 
        self.joint_value_labels = []
        self.tcp_value_labels = {}
        
        self.homing_dialog = None
        self.update_pending = False
        self.is_homed = False  # Domyślnie robot nie jest zbazowany
        # === 1. APPLY STYLE ===
        self._apply_dark_style()

        # === 2. BUILD GUI ===
        self.init_gui()
        self._create_estop_overlay()
        
        # === 3. LOGIC ===
        self.init_worker()
        self.update_sliders_from_angles(self.standby_angles)
        self.apply_transforms_from_angles(self.standby_angles)
        initial_fk = self.kinematics.forward_kinematics(self.standby_angles)
        if initial_fk is not None:
            self.current_orientation = initial_fk
            self.update_gui_feedback(initial_fk)
        print("Application ready.")
        QtCore.QTimer.singleShot(100, self.go_standby)

    def _apply_dark_style(self):
        """Modern dark CSS style."""
        dark_style = """
        QWidget {
            background-color: #2b2b2b;
            color: #e0e0e0;
            font-family: 'Segoe UI', 'Roboto', sans-serif;
            font-size: 14px;
        }
        QGroupBox {
            border: 1px solid #444;
            border-radius: 6px;
            margin-top: 6px;
            font-weight: bold;
            color: #aaa;
            padding-top: 10px;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            subcontrol-position: top left;
            padding: 0 5px;
            left: 10px;
        }
        QLineEdit {
            background-color: #1e1e1e;
            border: 1px solid #555;
            border-radius: 4px;
            padding: 6px;
            color: #fff;
            selection-background-color: #007acc;
        }
        QLineEdit:focus {
            border: 1px solid #007acc;
        }
        QComboBox {
            background-color: #3c3c3c;
            border: 1px solid #555;
            border-radius: 4px;
            padding: 5px;
            color: #fff;
        }
        QTabWidget::pane {
            border: 1px solid #444;
            border-radius: 4px;
        }
        QTabBar::tab {
            background: #3c3c3c;
            padding: 8px 20px;
            margin-right: 2px;
            border-top-left-radius: 4px;
            border-top-right-radius: 4px;
        }
        QTabBar::tab:selected {
            background: #505050;
            color: #fff;
            border-bottom: 2px solid #007acc;
        }
        QLabel {
            color: #ccc;
        }
        QSlider::groove:horizontal {
            border: 1px solid #333;
            height: 6px;
            background: #1a1a1a;
            margin: 2px 0;
            border-radius: 3px;
        }
        QSlider::handle:horizontal {
            background: #007acc;
            border: 1px solid #007acc;
            width: 16px;
            height: 16px;
            margin: -6px 0;
            border-radius: 8px;
        }
        QSlider::handle:horizontal:hover {
            background: #0099ff;
        }
        QPushButton {
            background-color: #444;
            border: none;
            border-radius: 5px;
            padding: 8px 15px;
            color: white;
            font-weight: bold;
            font-size: 13px;
        }
        QPushButton:hover {
            background-color: #555;
        }
        QPushButton:pressed {
            background-color: #333;
        }
        QPushButton:disabled {
            background-color: #333;
            color: #777;
        }
        QPlainTextEdit {
            background-color: #1e1e1e;
            color: #a9b7c6;
            border: 1px solid #444;
            font-family: 'Consolas', 'Monospace';
        }
        """
        self.setStyleSheet(dark_style)

    def init_gui(self):
        """Builds the entire UI."""
        self.setWindowTitle("PAROL6 3D Control Center by Jakub Grzebień 2025")
        self.resize(1400, 850)
        
        # === MAIN LAYOUT ===
        self.main_layout = QtWidgets.QHBoxLayout(self)
        self.main_layout.setContentsMargins(10, 10, 10, 10)
        self.main_layout.setSpacing(15)
        
        # --- 1. LEFT PANEL (MANUAL/AUTO) ---
        self.control_widget = self._create_left_control_panel()
        self.main_layout.addWidget(self.control_widget)

        # --- 2. MIDDLE PANEL (3D VIS) ---
        self.view = self._create_3d_visualization_panel()
        container_3d = QtWidgets.QWidget()
        container_3d.setStyleSheet("background-color: #000; border: 1px solid #555; border-radius: 4px;")
        l3d = QtWidgets.QVBoxLayout(container_3d)
        l3d.setContentsMargins(1, 1, 1, 1)
        l3d.addWidget(self.view)
        
        self.main_layout.addWidget(container_3d)

        # --- 3. RIGHT PANEL (UART + POS + BUTTONS) ---
        right_panel = self._create_right_panel()
        self.main_layout.addWidget(right_panel)

        # === STRETCHING ===
        self.main_layout.setStretch(0, 0) # Left
        self.main_layout.setStretch(1, 1) # 3D (Stretchable)
        self.main_layout.setStretch(2, 0) # Right

        # === LOAD MESHES ===
        self._load_3d_meshes()

        # === SIGNALS ===
        self._connect_gui_signals()

    def _create_estop_overlay(self):
        self.estop_overlay = QtWidgets.QFrame(self)
        self.estop_overlay.setStyleSheet("background-color: rgba(200, 20, 20, 220);")
        self.estop_overlay.hide()

        overlay_layout = QtWidgets.QVBoxLayout(self.estop_overlay)
        overlay_layout.setAlignment(Qt.AlignCenter)
        overlay_layout.setSpacing(20)

        lbl_image = QLabel()
        lbl_image.setAlignment(Qt.AlignCenter)
        lbl_image.setStyleSheet("background: transparent; border: none;")

        image_path = 'assets/ESTOP.png' 
        pixmap = QPixmap(image_path)
        if not pixmap.isNull():
            scaled_pixmap = pixmap.scaled(200, 200, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            lbl_image.setPixmap(scaled_pixmap)
        else:
            lbl_image.setText("🛑")
            lbl_image.setStyleSheet("background: transparent; color: white; font-size: 100px;")

        lbl_text = QLabel("EMERGENCY STOP")
        lbl_text.setStyleSheet("background: transparent; color: white; font-weight: 900; font-size: 48px; border: none;")
        lbl_text.setAlignment(Qt.AlignCenter)

        lbl_desc = QLabel("E-Stop button pressed.\nRelease E-Stop to resume operation.")
        lbl_desc.setStyleSheet("background: transparent; color: #eee; font-size: 18px; border: none;")
        lbl_desc.setAlignment(Qt.AlignCenter)

        overlay_layout.addStretch()
        overlay_layout.addWidget(lbl_image)
        overlay_layout.addWidget(lbl_text)
        overlay_layout.addWidget(lbl_desc)
        overlay_layout.addStretch()

    def resizeEvent(self, event):
        if hasattr(self, 'estop_overlay'):
            self.estop_overlay.resize(self.width(), self.height())
        super().resizeEvent(event)

    # -----------------------------------------------------------------
    # LEFT PANEL (MANUAL / AUTO)
    # -----------------------------------------------------------------
    def _create_left_control_panel(self):
        control_widget = QtWidgets.QWidget()
        control_widget.setFixedWidth(280)
        control_layout = QtWidgets.QVBoxLayout(control_widget)
        control_layout.setSpacing(10)
        control_layout.setContentsMargins(0, 0, 0, 0)

        # --- TABS ---
        self.tab_widget = QtWidgets.QTabWidget()
        control_layout.addWidget(self.tab_widget)

        # === TAB 1: Manual Control ===
        manual_tab = QtWidgets.QWidget()
        
        # Define layout primarily to avoid UnboundLocalError
        manual_layout = QtWidgets.QVBoxLayout(manual_tab) 
        manual_layout.setSpacing(15) 
        manual_layout.setContentsMargins(10, 15, 10, 10)
        
        # 1. SPEED SLIDER
        speed_group = QtWidgets.QGroupBox("Speed")
        speed_layout = QtWidgets.QHBoxLayout()
        
        self.speed_label = QtWidgets.QLabel("50%")
        self.speed_label.setFixedWidth(40)
        self.speed_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.speed_label.setStyleSheet("font-weight: bold; color: #00e5ff;")

        self.speed_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.speed_slider.setRange(10, 100) # 10% to 100%
        self.speed_slider.setValue(50) 
        self.speed_slider.valueChanged.connect(self.on_speed_changed)
        
        speed_layout.addWidget(self.speed_slider)
        speed_layout.addWidget(self.speed_label)
        speed_group.setLayout(speed_layout)
        
        manual_layout.addWidget(speed_group)

        # 2. XYZ
        xyz_group = QtWidgets.QGroupBox("Target (mm)")
        xyz_layout = QtWidgets.QGridLayout()
        xyz_layout.setVerticalSpacing(10)
        self.xyz_inputs = {}
        for i, label in enumerate(['X', 'Y', 'Z']):
            lbl = QtWidgets.QLabel(f"{label}")
            lbl.setStyleSheet("font-weight: bold; color: #bbb;")
            inp = QtWidgets.QLineEdit()
            inp.setAlignment(Qt.AlignRight)
            validator = QtGui.QDoubleValidator(config.RANGES_XYZ[i][0], config.RANGES_XYZ[i][1], 3)
            inp.setValidator(validator)
            inp.setText(f"{config.DEFAULT_XYZ_GUI[i]:.1f}")
            xyz_layout.addWidget(lbl, i, 0)
            xyz_layout.addWidget(inp, i, 1)
            self.xyz_inputs[label] = inp
        
        self.set_xyz_btn = QtWidgets.QPushButton("Move to XYZ")
        self.set_xyz_btn.setStyleSheet("background-color: #0277bd;")
        xyz_layout.addWidget(self.set_xyz_btn, 3, 0, 1, 2)
        xyz_group.setLayout(xyz_layout)
        manual_layout.addWidget(xyz_group)

        # 3. RPY
        rpy_group = QtWidgets.QGroupBox("Orientation (°)")
        rpy_layout = QtWidgets.QGridLayout()
        rpy_layout.setVerticalSpacing(10)
        self.rpy_inputs = {}
        for i, label in enumerate(['Roll', 'Pitch', 'Yaw']):
            lbl = QtWidgets.QLabel(label[0]) 
            lbl.setStyleSheet("font-weight: bold; color: #bbb;")
            inp = QtWidgets.QLineEdit()
            inp.setAlignment(Qt.AlignRight)
            inp.setValidator(QtGui.QDoubleValidator(-180, 180, 2))
            inp.setText("0.0")
            rpy_layout.addWidget(lbl, i, 0)
            rpy_layout.addWidget(inp, i, 1)
            self.rpy_inputs[label] = inp
        
        self.set_rpy_btn = QtWidgets.QPushButton("Set RPY")
        self.set_rpy_btn.setStyleSheet("background-color: #00695c;")
        rpy_layout.addWidget(self.set_rpy_btn, 3, 0, 1, 2)
        rpy_group.setLayout(rpy_layout)
        manual_layout.addWidget(rpy_group)

        # 4. JOINT SLIDERS
        sliders_group = QtWidgets.QGroupBox("Joints")
        sliders_layout = QtWidgets.QVBoxLayout()
        sliders_layout.setSpacing(15) 
        self.joint_sliders = []
        self.joint_labels = []
        
        for i in range(6):
            row = QtWidgets.QHBoxLayout()
            lbl_name = QtWidgets.QLabel(f"J{i+1}")
            lbl_name.setFixedWidth(20)
            
            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            slider.setRange(config.SLIDER_RANGE_MIN, config.SLIDER_RANGE_MAX)
            slider.setValue(self.map_rad_to_slider(0.0, i))
            
            lbl_val = QtWidgets.QLabel("0.0°")
            lbl_val.setFixedWidth(45)
            lbl_val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            
            row.addWidget(lbl_name)
            row.addWidget(slider)
            row.addWidget(lbl_val)
            
            sliders_layout.addLayout(row)
            self.joint_sliders.append(slider)
            self.joint_labels.append(lbl_val)
            
        sliders_group.setLayout(sliders_layout)
        manual_layout.addWidget(sliders_group)

        # Spacer to push banner to the bottom
        manual_layout.addStretch()

        # 5. STATUS BANNER
        self.status_banner = QtWidgets.QLabel("SYSTEM READY")
        self.status_banner.setAlignment(Qt.AlignCenter)
        self.status_banner.setFixedHeight(50) 
        self.status_banner.setWordWrap(True)
        self.status_banner.setStyleSheet("""
            QLabel {
                background-color: #333;
                color: #888;
                border: 2px solid #444;
                border-radius: 8px;
                font-weight: bold;
                font-size: 13px;
                padding: 5px;
            }
        """)
        manual_layout.addWidget(self.status_banner)

        # Add manual tab
        self.tab_widget.addTab(manual_tab, "Manual")

        # === TAB 2: Auto / Program ===
        program_tab = QtWidgets.QWidget()
        program_layout = QtWidgets.QVBoxLayout(program_tab)
        program_layout.setContentsMargins(10, 15, 10, 10)
        
        self.program_editor = QtWidgets.QPlainTextEdit()
        self.program_editor.setPlaceholderText("Enter program commands here...\nMOVE(200, 0, 150)\nDELAY(500)")
        program_layout.addWidget(self.program_editor)
        
        prog_btns = QtWidgets.QHBoxLayout()
        self.run_program_btn = QtWidgets.QPushButton("▶ RUN")
        self.run_program_btn.setMinimumHeight(40)
        self.run_program_btn.setStyleSheet("background-color: #2e7d32; border-radius: 5px;") 
        
        self.stop_program_btn = QtWidgets.QPushButton("⏹ STOP")
        self.stop_program_btn.setMinimumHeight(40)
        self.stop_program_btn.setEnabled(False)
        self.stop_program_btn.setStyleSheet("""
            QPushButton { background-color: #c62828; border-radius: 5px; }
            QPushButton:disabled { background-color: #444; color: #666; }
        """)
        
        prog_btns.addWidget(self.run_program_btn)
        prog_btns.addWidget(self.stop_program_btn)
        program_layout.addLayout(prog_btns)
        
        self.tab_widget.addTab(program_tab, "Auto")

        control_layout.addStretch()
        return control_widget

    def on_speed_changed(self, value):
        """Speed slider handler."""
        self.speed_label.setText(f"{value}%")
        # Convert 10-100 to 0.1-1.0
        multiplier = value / 100.0
        self.set_speed_signal.emit(multiplier)

    def _create_3d_visualization_panel(self):
        """3D Visualization with dark background."""
        view = gl.GLViewWidget()
        view.opts['center'] = QVector3D(0, 0, 0.2)
        view.opts['msaa'] = True
        g = gl.GLGridItem()
        g.setSize(1, 1, 1)
        g.setSpacing(0.1, 0.1, 0.1)
        view.addItem(g)
        
        ax = gl.GLAxisItem()
        ax.setSize(0.2, 0.2, 0.2)
        view.addItem(ax)
        
        view.setCameraPosition(distance=0.9, elevation=30, azimuth=45)
        view.setMinimumSize(800, 600)
        view.setBackgroundColor((30, 30, 30))
        return view

    # ---------------------------------------------------------
    # RIGHT PANEL - UART, POS, BUTTONS
    # ---------------------------------------------------------
    def _create_right_panel(self):
        container = QtWidgets.QWidget()
        container.setFixedWidth(180) 
        
        layout = QtWidgets.QVBoxLayout(container)
        layout.setSpacing(10)
        layout.setContentsMargins(0, 0, 0, 0)
        
        # === 1. UART ===
        uart_group = QtWidgets.QGroupBox("UART")
        uart_layout = QtWidgets.QVBoxLayout()
        self.port_combo = QtWidgets.QComboBox()
        try:
            import serial.tools.list_ports
            available_ports = [port.device for port in serial.tools.list_ports.comports()]
        except ImportError:
            available_ports = []
        self.port_combo.addItems(available_ports if available_ports else ["No ports"])
        
        self.uart_status_label = QtWidgets.QLabel("Disconnected")
        self.uart_status_label.setStyleSheet("color: #777; font-size: 11px;")
        self.uart_status_label.setAlignment(Qt.AlignCenter)
        
        uart_layout.addWidget(self.port_combo)
        uart_layout.addWidget(self.uart_status_label)
        uart_group.setLayout(uart_layout)
        layout.addWidget(uart_group)

        # === 2. POSITION (DEG) ===
        pos_group = QtWidgets.QGroupBox("Position")
        pos_layout = QtWidgets.QVBoxLayout()
        pos_layout.setSpacing(5)
        
        self.joint_value_labels = [] 
        for i in range(6):
            row = QtWidgets.QHBoxLayout()
            lbl_title = QLabel(f"J{i+1}:")
            lbl_title.setStyleSheet("color: #aaa; font-size: 11px;")
            
            lbl_val = QLabel("0.0°")
            lbl_val.setFont(QFont("Monospace", 10, QFont.Bold))
            lbl_val.setStyleSheet("color: #00e5ff;")
            lbl_val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            
            self.joint_value_labels.append(lbl_val)
            row.addWidget(lbl_title)
            row.addWidget(lbl_val)
            pos_layout.addLayout(row)

        # Separator
        line = QtWidgets.QFrame()
        line.setFrameShape(QtWidgets.QFrame.HLine)
        line.setStyleSheet("color: #555; margin: 5px 0;")
        pos_layout.addWidget(line)

        # TCP Position (XYZABC)
        lbl_header_tcp = QLabel("TCP (XYZABC)")
        lbl_header_tcp.setStyleSheet("color: #fff; font-weight: bold; font-size: 11px;")
        lbl_header_tcp.setAlignment(Qt.AlignCenter)
        pos_layout.addWidget(lbl_header_tcp)

        self.tcp_value_labels = {}
        labels_map = [
            ("X", "mm"), ("Y", "mm"), ("Z", "mm"),
            ("A", "deg"), ("B", "deg"), ("C", "deg")
        ]

        for axis, unit in labels_map:
            row = QtWidgets.QHBoxLayout()
            lbl_name = QLabel(f"{axis}:")
            lbl_name.setStyleSheet("color: #aaa; font-size: 11px;")
            
            lbl_val = QLabel("0.00")
            lbl_val.setFont(QFont("Monospace", 10))
            if unit == "mm":
                lbl_val.setStyleSheet("color: #76ff03;") # Green
            else:
                lbl_val.setStyleSheet("color: #ff9100;") # Orange
            
            lbl_val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            
            self.tcp_value_labels[axis] = lbl_val
            row.addWidget(lbl_name)
            row.addWidget(lbl_val)
            pos_layout.addLayout(row)
        
        pos_group.setLayout(pos_layout)
        layout.addWidget(pos_group)

        # === 3. BUTTONS ===
        btns_layout = QtWidgets.QVBoxLayout()
        btns_layout.setSpacing(10)
        
        def create_btn(text, icon_name, color_hex=None, fallback_char=""):
            btn = QtWidgets.QPushButton(text)
            
            if HAS_QTA and icon_name:
                btn.setIcon(qta.icon(icon_name, color='white'))
                btn.setIconSize(QtCore.QSize(20, 20))
            elif fallback_char:
                btn.setText(f"{fallback_char}  {text}")

            base_style = """
                QPushButton {
                    border-radius: 5px; 
                    color: white; 
                    text-align: center; 
                    padding: 8px;
                    font-weight: bold;
                    border: 1px solid rgba(255,255,255,0.1);
                }
            """
            if color_hex:
                color_style = f"""
                    QPushButton {{ background-color: {color_hex}; }}
                    QPushButton:hover {{ background-color: {color_hex}dd; }}
                    QPushButton:pressed {{ background-color: {color_hex}aa; }}
                """
            else:
                color_style = """
                    QPushButton { background-color: #424242; }
                    QPushButton:hover { background-color: #555; }
                """
            
            btn.setStyleSheet(base_style + color_style)
            btn.setMinimumHeight(45)
            return btn

        # --- Buttons ---
        self.home_button = create_btn("HOME", "fa5s.home", "#1565c0", "⌂")          
        self.standby_btn = create_btn("STANDBY", "fa5s.pause-circle", "#6a1b9a", "⏸") 
        self.stop_btn    = create_btn("STOP", "fa5s.stop-circle", "#c62828", "🛑")    
        self.safety_btn  = create_btn("SAFETY", "fa5s.exclamation-triangle", "#ef6c00", "⚠️") 
        self.tool_btn    = create_btn("TOOL", "fa5s.tools", "#00838f", "🔧")          
        self.camera_btn  = create_btn("CAMERA", "fa5s.video", "#455a64", "📷")        

        btns_layout.addWidget(self.home_button)
        btns_layout.addWidget(self.standby_btn)
        btns_layout.addWidget(self.stop_btn)
        btns_layout.addWidget(self.safety_btn)
        btns_layout.addWidget(self.tool_btn)
        btns_layout.addWidget(self.camera_btn)
        
        layout.addLayout(btns_layout)
        layout.addStretch() 
        
        return container

    # ------------------------
    # Mapping
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
    # Logic
    # ------------------------

    def _load_3d_meshes(self):
        """Loads 3D meshes."""
        self.links_3d = [] 
        for i, stl_file in enumerate(config.STL_FILES):
            try:
                print(f"[DEBUG] Loading file: 'assets/{stl_file}'...")
                mesh_data = trimesh.load(f'assets/{stl_file}')
                
                if not isinstance(mesh_data, trimesh.Trimesh):
                    print(f"ERROR: {stl_file} is not a mesh.")
                    raise ValueError(f"{stl_file} loaded as PointCloud.")

                mesh_data.process()
                mesh_data.fix_normals()
                mesh_data.fill_holes()

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
                print(f"Error loading mesh '{stl_file}': {e}. Using placeholder.")
                mesh_item = gl.GLMeshItem(vertexes=np.array([[0,0,0]]))
                vc = np.array([[0.8, 0.8, 0.8]]) 
            
            self.default_link_colors[i] = vc 
            self.view.addItem(mesh_item)
            self.links_3d.append(mesh_item)
            
        print(f"Loaded {len(self.links_3d)} meshes.")


    def swap_tool_mesh(self, tool_name):
        """Swaps the L6/Tool mesh in runtime."""
        
        filename = config.TOOL_STL_MAP.get(tool_name)
        if not filename:
            print(f"[GUI] No mesh defined for {tool_name}")
            return

        print(f"[GUI] Loading tool mesh: {filename}")
        
        TOOL_INDEX = 6 
        
        if len(self.links_3d) > TOOL_INDEX:
            old_mesh = self.links_3d[TOOL_INDEX]
            self.view.removeItem(old_mesh)
        
        try:
            mesh_data = trimesh.load(f'assets/{filename}')
            if not isinstance(mesh_data, trimesh.Trimesh):
                 if hasattr(mesh_data, 'geometry') and len(mesh_data.geometry) > 0:
                     mesh_data = list(mesh_data.geometry.values())[0]
                 else:
                     raise ValueError("Invalid 3D file format")

            mesh_data.process()
            mesh_data.fix_normals()
            
            if hasattr(mesh_data.visual, "vertex_colors") and mesh_data.visual.vertex_colors is not None:
                vc = (mesh_data.visual.vertex_colors.astype(np.float32) / 255.0)[:, :3]
            else:
                vc = np.ones((len(mesh_data.vertices), 3), dtype=np.float32) * 0.6

            new_mesh_item = gl.GLMeshItem(
                vertexes=np.array(mesh_data.vertices),
                faces=np.array(mesh_data.faces),
                vertexColors=vc,
                shader='shaded',
                smooth=True,
                drawFaces=True
            )
            
            self.view.addItem(new_mesh_item)
            self.links_3d[TOOL_INDEX] = new_mesh_item
            
            self.default_link_colors[TOOL_INDEX] = vc

        except Exception as e:
            print(f"[GUI ERROR] Failed to swap tool mesh: {e}")


    def _connect_gui_signals(self):
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

        # Right Panel Buttons
        self.home_button.clicked.connect(self.on_home_click) 
        
        self.standby_btn.clicked.connect(self.go_standby)
        self.stop_btn.clicked.connect(self.on_stop_click) 
        self.camera_btn.clicked.connect(self.reset_camera)
        
        self.safety_btn.clicked.connect(self.on_safety_click)
        self.tool_btn.clicked.connect(self.on_tool_change_click)
        
        self.run_program_btn.clicked.connect(self.on_run_program)
        self.stop_program_btn.clicked.connect(self.on_stop_program)

    def init_worker(self):
        self.worker_thread = QThread()
        self.worker = RobotWorker(self.standby_angles, self.kinematics)
        self.worker.moveToThread(self.worker_thread)
        
        self.change_tool_signal.connect(self.worker.change_active_tool)
        self.worker.tool_changed.connect(self.on_tool_changed_feedback)

        self.worker.angles_updated.connect(self.apply_transforms_from_angles)
        self.worker.angles_updated.connect(self.update_joint_value_labels) 
        self.worker.angles_updated.connect(self.update_sliders_from_angles)
        
        self.worker.status_updated.connect(self.update_status_label)
        self.worker.limit_switch_status.connect(self.on_limit_switch_hit)
        
        self.worker.program_finished.connect(self.on_program_finished)
        self.worker.program_line_highlight.connect(self.on_program_line_highlight)
        
        if hasattr(self.worker, 'estop_status_signal'):
            self.worker.estop_status_signal.connect(self.on_estop_signal)

        self.start_move_signal.connect(self.worker.start_move)
        self.start_linear_move_signal.connect(self.worker.start_linear_move)
        self.set_port_signal.connect(self.worker.set_com_port)
        self.request_homing_signal.connect(self.worker.start_homing)
        self.request_gripper_signal.connect(self.worker.set_gripper_state)
        
        # Connect Speed Signal
        if hasattr(self.worker, 'set_speed_multiplier'):
            self.set_speed_signal.connect(self.worker.set_speed_multiplier)
        else:
            print("WARNING: Worker has no set_speed_multiplier method!")
        
        self.start_program_signal.connect(self.worker.start_program)
        self.stop_program_signal.connect(self.worker.stop_program)
        
        self.port_combo.currentTextChanged.connect(self.set_port_signal.emit)
        
        self.worker_thread.start()
        self.set_port_signal.emit(self.port_combo.currentText())

    def closeEvent(self, event):
        print("Closing application...")
        self.stop_program_signal.emit()
        self.worker_thread.quit()
        self.worker_thread.wait(2000)
        comm.close_serial_port()
        event.accept()


    def _check_homed_safe(self):
        """Zwraca True jeśli robot jest zbazowany, w przeciwnym razie wyświetla błąd."""
        if not self.is_homed:
            self.update_status_label("BLOCKED: Robot not homed! Press HOME first.", "red")
            print("[GUI] Movement blocked - Robot not homed.")
            return False
        return True

    # ------------------------
    # Slots
    # ------------------------

    @pyqtSlot(bool)
    def on_estop_signal(self, is_active):
        if is_active:
            self.estop_overlay.raise_()
            self.estop_overlay.show()
            self.control_widget.setEnabled(False)
        else:
            self.estop_overlay.hide()
            self.control_widget.setEnabled(True)

    @pyqtSlot(np.ndarray)
    def apply_transforms_from_angles(self, angles):
        if not self.links_3d or len(self.links_3d) != len(config.LINK_NAMES):
            return

        transforms = self.kinematics.forward_kinematics_full(angles)
        MANUAL_INDICES = [0, 1, 2, 3, 4, 5, 6]
        
        for i, config_name in enumerate(config.LINK_NAMES):
            mesh = self.links_3d[i]
            if i < len(MANUAL_INDICES):
                target_idx = MANUAL_INDICES[i]
            else:
                continue

            if target_idx >= len(transforms): continue

            T = transforms[target_idx]
            
            try:
                urdf_link_name = self.kinematics.chain.links[target_idx].name
                if urdf_link_name in self.visual_origins:
                    xyz_offset, rpy_offset = self.visual_origins[urdf_link_name]
                    T_origin = np.eye(4)
                    T_origin[:3, :3] = R.from_euler('xyz', rpy_offset).as_matrix()
                    T_origin[:3, 3] = xyz_offset
                    T = T @ T_origin
            except Exception:
                pass

            T_scene = config.S_MATRIX @ T
            mesh.setTransform(matrix_to_qtransform(T_scene))
        
        self.view.update()

    def read_target_pose_inputs(self):
        X_user_mm = parse_float_from_input(self.xyz_inputs['X'], default_val=0.0)
        Y_user_mm = parse_float_from_input(self.xyz_inputs['Y'], default_val=0.0)
        Z_user_mm = parse_float_from_input(self.xyz_inputs['Z'], default_val=250.0) 

        X_robot_m =  Y_user_mm / 1000.0
        Y_robot_m = -X_user_mm / 1000.0
        Z_robot_m =  Z_user_mm / 1000.0
            
        return np.array([X_robot_m, Y_robot_m, Z_robot_m])

    @pyqtSlot(str, str)
    def update_status_label(self, text, color):
        """Updates status banner and parses homing codes."""
        print(f"!!! RAW UART DATA: '{text}' (Color: {color})")
        # --- PARSOWANIE Hx / Rx ---
        # Sprawdzamy czy dialog istnieje i jest widoczny
        if self.homing_dialog and self.homing_dialog.isVisible():
            clean_text = text.strip().upper()
            
            # Debugowanie w konsoli (czy widzisz to po wysłaniu H1?)
            # print(f"[DEBUG PARSE] Text: '{text}' -> Clean: '{clean_text}'")

            # Logika dla H1, H2... R1, R2...
            if len(clean_text) >= 2 and clean_text[1].isdigit():
                code = clean_text[0]      # 'H' lub 'R'
                
                # Zabezpieczenie: bierzemy pierwszą cyfrę (np. z H1\n)
                try:
                    axis_idx = int(clean_text[1]) - 1 # 0-5
                except ValueError:
                    axis_idx = -1

                if 0 <= axis_idx <= 5:
                    if code == 'H':
                        # print(f"[DEBUG GUI] Axis {axis_idx+1} HIT detected!")
                        self.homing_dialog.set_axis_state(axis_idx, 'HIT')
                        return # Nie pokazuj tego w głównym banerze

                    elif code == 'R':
                        # print(f"[DEBUG GUI] Axis {axis_idx+1} DONE detected!")
                        self.homing_dialog.set_axis_state(axis_idx, 'DONE')
                        return
        # ---------------------------

        # Standardowa obsługa bannera (dla innych komunikatów)
        self.uart_status_label.setText(text)
        
        bg_color = "#333333"; text_color = "#ffffff"; border_color = "#555555"

        if "red" in color:
            bg_color = "#c62828"; border_color = "#ff8a80"
            text = f"⚠️ {text.upper()} ⚠️"
        elif "orange" in color:
            bg_color = "#ef6c00"; border_color = "#ffe0b2"
        elif "green" in color:
            bg_color = "#2e7d32"; border_color = "#a5d6a7"
        elif "blue" in color or "cyan" in color:
            bg_color = "#0277bd"; border_color = "#81d4fa"

        try:
            self.status_banner.setText(text)
            self.status_banner.setStyleSheet(f"""
                QLabel {{
                    background-color: {bg_color};
                    color: {text_color};
                    border: 2px solid {border_color};
                    border-radius: 8px;
                    font-weight: bold;
                    font-size: 13px;
                    padding: 5px;
                }}
            """)
            self.uart_status_label.setStyleSheet(f"color: {border_color};")
        except AttributeError:
            pass

        if "HOMING_COMPLETE_OK" in text:
            self.is_homed = True 
            if self.homing_dialog:
                # Zamknij z małym opóźnieniem, żeby zobaczyć efekt
                QtCore.QTimer.singleShot(1000, lambda: self.homing_dialog.accept() if self.homing_dialog else None)

    def update_sliders_from_angles(self, angles_rad):
        if len(angles_rad) < 7: offset = 0
        else: offset = 1 
        
        try:
            fk_matrix = self.kinematics.forward_kinematics(angles_rad)
            self.update_gui_feedback(fk_matrix)
        except Exception as e:
            pass

        for i in range(6):
            if self.joint_sliders[i].isSliderDown():
                continue

            if i + offset < len(angles_rad):
                rad_val = angles_rad[i+offset]
                slider_val = self.map_rad_to_slider(rad_val, i)
                
                self.joint_sliders[i].blockSignals(True)
                self.joint_sliders[i].setValue(slider_val)
                self.joint_sliders[i].blockSignals(False)
                self.joint_labels[i].setText(f"{np.degrees(rad_val):.1f}°")

    @pyqtSlot(np.ndarray)
    def update_joint_value_labels(self, angles_rad):
        if not self.joint_value_labels: return
        for i in range(6):
            deg = np.degrees(angles_rad[i])
            self.joint_value_labels[i].setText(f"{deg:.1f}°")

    def update_rpy_inputs_from_matrix(self, matrix):
        try:
            r = R.from_matrix(matrix[:3, :3])
            
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                roll, pitch, yaw = r.as_euler('xyz', degrees=True) 
            
            for inp in self.rpy_inputs.values(): inp.blockSignals(True)
            self.rpy_inputs['Roll'].setText(f"{roll:.2f}")
            self.rpy_inputs['Pitch'].setText(f"{pitch:.2f}")
            self.rpy_inputs['Yaw'].setText(f"{yaw:.2f}")
            
            if self.tcp_value_labels:
                self.tcp_value_labels["A"].setText(f"{roll:.1f}")
                self.tcp_value_labels["B"].setText(f"{pitch:.1f}")
                self.tcp_value_labels["C"].setText(f"{yaw:.1f}")
                
        except Exception: pass
        finally:
            for inp in self.rpy_inputs.values(): inp.blockSignals(False)

    def update_gui_feedback(self, fk_matrix):
        """Updates text fields and TCP display."""
        self.update_rpy_inputs_from_matrix(fk_matrix)
        pos_robota_m = fk_matrix[:3, 3]
        
        X_user_m = -pos_robota_m[1]
        Y_user_m =  pos_robota_m[0]
        Z_user_m =  pos_robota_m[2]
            
        for inp in self.xyz_inputs.values(): inp.blockSignals(True)
        self.xyz_inputs['X'].setText(f"{X_user_m * 1000.0:.1f}")
        self.xyz_inputs['Y'].setText(f"{Y_user_m * 1000.0:.1f}")
        self.xyz_inputs['Z'].setText(f"{Z_user_m * 1000.0:.1f}")
        for inp in self.xyz_inputs.values(): inp.blockSignals(False)
        
        if self.tcp_value_labels:
            self.tcp_value_labels["X"].setText(f"{X_user_m * 1000.0:.1f}")
            self.tcp_value_labels["Y"].setText(f"{Y_user_m * 1000.0:.1f}")
            self.tcp_value_labels["Z"].setText(f"{Z_user_m * 1000.0:.1f}")

    def perform_update_robot(self):
        if not self._check_homed_safe():
            return
        target_pos_m = self.read_target_pose_inputs()
        roll_deg = parse_float_from_input(self.rpy_inputs['Roll'])
        pitch_deg = parse_float_from_input(self.rpy_inputs['Pitch'])
        yaw_deg = parse_float_from_input(self.rpy_inputs['Yaw'])
        
        try:
            rot = R.from_euler('xyz', [np.radians(roll_deg), np.radians(pitch_deg), np.radians(yaw_deg)])
            R_mat = rot.as_matrix()
            target_transform = np.eye(4)
            target_transform[:3, :3] = R_mat
            target_transform[:3, 3] = target_pos_m
        except Exception: return
        
        try:
            start_angles = self.worker.get_previous_angles()
            start_transform = self.kinematics.forward_kinematics(start_angles)
        except Exception: return

        self.start_linear_move_signal.emit(start_transform, target_transform)

    def perform_update_from_sliders(self):
        if not self._check_homed_safe():
            return
        active_angles = []
        for i in range(6):
            slider_val = self.joint_sliders[i].value()
            rad_val = self.map_slider_to_rad(slider_val, i)
            active_angles.append(rad_val)
        
        active_angles_np = np.array(active_angles)
        
        try:
            fk_matrix = self.kinematics.forward_kinematics(active_angles_np)
            
            if fk_matrix is None or fk_matrix.shape != (4, 4):
                print(f"[GUI ERROR] forward_kinematics returned wrong shape: {fk_matrix}")
                return

            self.current_orientation = fk_matrix.copy()
            self.update_gui_feedback(self.current_orientation)
            
        except Exception as e:
            print(f"!!! ERROR IN PERFORM_UPDATE !!!: {e}")
            import traceback
            traceback.print_exc()
        
        angles_for_worker = np.concatenate(([0.0], active_angles_np))
        self.start_move_signal.emit(angles_for_worker)

    def update_slider_label(self, value, joint_index):
        rad_val = self.map_slider_to_rad(value, joint_index)
        self.joint_labels[joint_index].setText(f"{np.degrees(rad_val):.1f}°")

    def schedule_update(self):
        if self.update_pending: return
        self.update_pending = True
        QtCore.QTimer.singleShot(config.DEBOUNCE_MS, self.perform_update_and_clear_pending)

    def perform_update_and_clear_pending(self):
        self.perform_update_robot()
        self.update_pending = False

    def go_standby(self):
        if not self._check_homed_safe():
            return
        target_angles = self.standby_angles.copy()
        fk = self.kinematics.forward_kinematics(target_angles)
        self.current_orientation = fk.copy()
        self.update_gui_feedback(self.current_orientation) 
        self.update_sliders_from_angles(target_angles) 
        self.start_move_signal.emit(target_angles)
        self.update_status_label("Status: Standby", "blue")

    # --- BUTTONS ---

    def on_home_click(self):
        self.homing_dialog = HomingDialog(self)
        self.homing_dialog.show()

    def on_safety_click(self):
        if not self._check_homed_safe():
            return
        target_deg = [0, -50, 70, 90, 0, 0]
        target_rad = np.array([0] + [np.radians(a) for a in target_deg])
        
        fk = self.kinematics.forward_kinematics(target_rad)
        self.update_gui_feedback(fk)
        self.update_sliders_from_angles(target_rad)
        
        self.start_move_signal.emit(target_rad)
        self.update_status_label("Status: SAFETY MOVE", "orange")

    def on_tool_change_click(self):
        """Displays tool selection menu."""
        menu = QtWidgets.QMenu(self)
        
        menu.setStyleSheet("""
            QMenu {
                background-color: #333;
                color: white;
                border: 1px solid #555;
            }
            QMenu::item {
                padding: 8px 20px;
            }
            QMenu::item:selected {
                background-color: #00838f;
            }
        """)

        action_small = menu.addAction("Small Gripper (Offset: 10cm)")
        action_small.triggered.connect(lambda: self.change_tool_signal.emit("CHWYTAK_MALY"))
        
        action_big = menu.addAction("Big Gripper (Offset: 25cm)")
        action_big.triggered.connect(lambda: self.change_tool_signal.emit("CHWYTAK_DUZY"))

        menu.exec_(self.tool_btn.mapToGlobal(QtCore.QPoint(0, self.tool_btn.height())))

    def on_stop_click(self):
        self.request_gripper_signal.emit("STOP")
        self.on_stop_program()
        self.update_status_label("Status: STOP SENT", "red")

    def reset_camera(self):
        self.view.setCameraPosition(distance=0.9, elevation=30, azimuth=45)

    def on_tool_changed(self, tool_name):
        # self.ui.lblCurrentTool.setText(f"Active: {tool_name}")
        print(f"GUI: Tool change confirmed: {tool_name}")

    @pyqtSlot(str)
    def on_tool_changed_feedback(self, tool_name):
        print(f"[GUI] Tool changed to: {tool_name}")
        
        self.tool_btn.setText(f"TOOL: {tool_name}")
        self.update_status_label(f"Changed to: {tool_name}", "cyan")
        
        self.swap_tool_mesh(tool_name)
        self.perform_update_from_sliders()

    @pyqtSlot()
    def on_gripper_click(self):
        self.gripper_state = not self.gripper_state
        if self.gripper_state:
            command = "VAC_ON"
        else:
            command = "VAC_OFF"
        print(f"Sending gripper command: {command}")
        self.request_gripper_signal.emit(command)

    def set_link_color(self, link_index_3d, color_rgb):
        try:
            target_mesh = self.links_3d[link_index_3d]
        except IndexError: return

        try:
            current_vertexes = target_mesh.vertexes
            current_faces = target_mesh.faces
            if current_vertexes is None: return
            num_verts = current_vertexes.shape[0]
        except Exception: return
            
        if isinstance(color_rgb, list):
            new_color_array = np.tile(color_rgb, (num_verts, 1)).astype(np.float32)
        elif isinstance(color_rgb, np.ndarray):
            new_color_array = np.tile(color_rgb, (num_verts, 1)).astype(np.float32)
        else: return

        target_mesh.setMeshData(
            vertexes=current_vertexes,
            faces=current_faces,
            vertexColors=new_color_array
        )

    @pyqtSlot(int, bool)
    def on_limit_switch_hit(self, joint_index, is_hit):
        link_index_3d = joint_index + 1
        if link_index_3d >= len(self.links_3d): return

        if is_hit:
            self.set_link_color(link_index_3d, [0.1, 0.9, 0.1])
        else:
            default_color_data = self.default_link_colors.get(link_index_3d)
            if default_color_data is not None:
                self.set_link_color(link_index_3d, default_color_data[0])
            else:
                self.set_link_color(link_index_3d, [0.8, 0.8, 0.8])

    @pyqtSlot()
    def on_run_program(self):
        if not self._check_homed_safe():
            return
        program_text = self.program_editor.toPlainText()
        if not program_text.strip(): return
        self.run_program_btn.setEnabled(False)
        self.stop_program_btn.setEnabled(True)
        self.tab_widget.setCurrentIndex(0) 
        self.update_status_label("Status: Running program...", "blue")
        self.start_program_signal.emit(program_text)

    @pyqtSlot()
    def on_stop_program(self):
        self.stop_program_signal.emit()
        self.update_status_label("Status: Stopping...", "orange")

    @pyqtSlot(str, bool)
    def on_program_finished(self, status_msg, was_stopped):
        self.run_program_btn.setEnabled(True)
        self.stop_program_btn.setEnabled(False)
        self.tab_widget.setCurrentIndex(1)
        self.update_status_label(f"Status: {status_msg}", "orange" if was_stopped else "green")
        self.on_program_line_highlight(-1)

    @pyqtSlot(int)
    def on_program_line_highlight(self, line_number):
        cursor = self.program_editor.textCursor()
        selection = QtWidgets.QTextEdit.ExtraSelection()
        selection.format.setBackground(QtCore.Qt.transparent)
        selection.cursor = cursor
        selections = [selection]

        if line_number >= 0:
            cursor.movePosition(QtGui.QTextCursor.Start)
            cursor.movePosition(QtGui.QTextCursor.Down, QtGui.QTextCursor.MoveAnchor, line_number)
            selection = QtWidgets.QTextEdit.ExtraSelection()
            selection.format.setBackground(QColor(60, 60, 40))
            selection.format.setProperty(QTextFormat.FullWidthSelection, True)
            selection.cursor = cursor
            selections.append(selection)

        self.program_editor.setExtraSelections(selections)