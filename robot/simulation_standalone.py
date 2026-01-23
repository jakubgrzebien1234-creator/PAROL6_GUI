
import sys
import os
import numpy as np
import trimesh
from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QMatrix4x4, QVector4D, QVector3D, QColor
import pyqtgraph.opengl as gl
from scipy.spatial.transform import Rotation as R

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

import robot.config as config
from robot.kinematics import RobotKinematics

def matrix_to_qtransform(matrix):
    """Converts a numpy 4x4 matrix to QMatrix4x4."""
    m = QMatrix4x4()
    for i in range(4):
        m.setRow(i, QVector4D(float(matrix[i, 0]), float(matrix[i, 1]), float(matrix[i, 2]), float(matrix[i, 3])))
    return m

class SimulationWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PAROL6 - Standalone Simulation")
        self.resize(1200, 800)
        
        # 1. Initialize Kinematics
        try:

            self.kinematics = RobotKinematics(config.URDF_PATH, None)
            self.joint_limits = self.kinematics.get_joint_limits()
            self.visual_origins = self.kinematics.get_visual_origins()
            print("[SIM] Kinematics initialized.")
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Init Error", f"Failed to initialize kinematics:\n{e}")
            self.joint_limits = [(-np.pi, np.pi)] * 6
            self.visual_origins = {}

        # 2. Main Layout
        self.main_layout = QtWidgets.QHBoxLayout(self)
        self.main_layout.setContentsMargins(0, 0, 0, 0)
        self.main_layout.setSpacing(0)

        # 3. Left Panel (Sliders)
        self.left_panel = QtWidgets.QWidget()
        self.left_panel.setFixedWidth(300)
        self.left_panel.setStyleSheet("background-color: #2b2b2b; color: #eee; border-right: 1px solid #444;")
        
        self.sliders_layout = QtWidgets.QVBoxLayout(self.left_panel)
        self.sliders_layout.setContentsMargins(20, 20, 20, 20)
        self.sliders_layout.setSpacing(20)
        
        title = QtWidgets.QLabel("JOINT CONTROLS")
        title.setStyleSheet("font-weight: bold; font-size: 16px; color: #00e5ff; margin-bottom: 10px;")
        title.setAlignment(Qt.AlignCenter)
        self.sliders_layout.addWidget(title)
        
        self.sliders = []
        self.labels = []
        
        for i in range(6):
            group = QtWidgets.QWidget()
            gh = QtWidgets.QVBoxLayout(group)
            gh.setContentsMargins(0,0,0,0)
            gh.setSpacing(5)
            
            lbl_info = QtWidgets.QHBoxLayout()
            name_lbl = QtWidgets.QLabel(f"Joint {i+1}")
            name_lbl.setStyleSheet("font-weight: bold; color: #aaa;")
            val_lbl = QtWidgets.QLabel("0.0°")
            val_lbl.setStyleSheet("color: #fff; font-weight: bold;")
            val_lbl.setAlignment(Qt.AlignRight)
            self.labels.append(val_lbl)
            
            lbl_info.addWidget(name_lbl)
            lbl_info.addWidget(val_lbl)
            gh.addLayout(lbl_info)
            
            slider = QtWidgets.QSlider(Qt.Horizontal)
            slider.setRange(config.SLIDER_RANGE_MIN, config.SLIDER_RANGE_MAX)
            mid = (config.SLIDER_RANGE_MAX + config.SLIDER_RANGE_MIN) // 2
            slider.setValue(mid)
            slider.valueChanged.connect(self.on_slider_changed)
            
            self.sliders.append(slider)
            gh.addWidget(slider)
            
            self.sliders_layout.addWidget(group)
            
        self.sliders_layout.addStretch()
        

        
        self.main_layout.addWidget(self.left_panel)
        
        # 4. Right Panel (3D View)
        self.view = gl.GLViewWidget()
        self.view.opts['center'] = QVector3D(0, 0, 0.4)
        self.view.opts['msaa'] = True
        self.view.setCameraPosition(distance=1.5, elevation=25, azimuth=-45)
        self.view.setBackgroundColor((30, 30, 30))
        
        # Floor Grid
        g = gl.GLGridItem()
        g.setSize(2, 2, 1)
        g.setSpacing(0.1, 0.1, 0.1)
        self.view.addItem(g)
        
        # Coordinates
        ax = gl.GLAxisItem()
        ax.setSize(0.15, 0.15, 0.15)
        self.view.addItem(ax)
        
        self.main_layout.addWidget(self.view, 1)

        # 5. Load Meshes
        self.links_3d = []
        self.load_meshes()
        
        # Load Extra Base (if any)
        self.load_extra_base()
        
        self.reset_sliders() # Initial update

    def _create_mesh_item_from_file(self, filename):
        """Helper to load mesh from file."""
        # Try local assets first, then relative to script
        paths_to_try = [
            f"assets/{filename}",
            f"../assets/{filename}",
            os.path.join(parent_dir, f"assets/{filename}")
        ]
        
        mesh_data = None
        loaded_path = ""
        for p in paths_to_try:
            if os.path.exists(p):
                loaded_path = p
                break
        
        if not loaded_path:
            print(f"[SIM] Warning: Could not find mesh {filename}")
            return gl.GLMeshItem()

        try:
            mesh_data = trimesh.load(loaded_path)
            if isinstance(mesh_data, trimesh.Scene):
                if len(mesh_data.geometry) == 0: raise ValueError("Empty Scene")
                mesh_data = mesh_data.dump(concatenate=True)

            mesh_data.process()
            # Suppress normalization warning for flat/zero faces
            try:
                mesh_data.fix_normals()
            except Exception: pass
            mesh_data.fill_holes()
            
            # Colors
            if hasattr(mesh_data.visual, 'to_color'):
                try: mesh_data.visual = mesh_data.visual.to_color()
                except: pass
            
            vc = None
            if hasattr(mesh_data.visual, "vertex_colors") and mesh_data.visual.vertex_colors is not None:
                vc = (mesh_data.visual.vertex_colors.astype(np.float32) / 255.0)[:, :3]
                if len(vc) != len(mesh_data.vertices): vc = None
            
            if vc is None:
                # Default gray
                vc = np.ones((len(mesh_data.vertices), 3), dtype=np.float32) * 0.75

            mesh_item = gl.GLMeshItem(
                vertexes=np.array(mesh_data.vertices),
                faces=np.array(mesh_data.faces),
                vertexColors=vc,
                shader='shaded',
                smooth=True,
                drawFaces=True
            )
            return mesh_item
        except Exception as e:
            print(f"[SIM] Error loading {filename}: {e}")
            return gl.GLMeshItem()

    def load_meshes(self):
        print("[SIM] Loading kinetic meshes...")
        for i, stl_file in enumerate(config.STL_FILES):
             mesh = self._create_mesh_item_from_file(stl_file)
             self.view.addItem(mesh)
             self.links_3d.append(mesh)

    def load_extra_base(self):
        if hasattr(config, 'BASE_EXTRA_FILES') and config.BASE_EXTRA_FILES:
             print(f"[SIM] Loading {len(config.BASE_EXTRA_FILES)} extra static base files...")
             
             T_base = np.eye(4)
             try:
                # Base is usually Link 0
                urdf_link_name = self.kinematics.chain.links[0].name
                if urdf_link_name in self.visual_origins:
                    xyz_offset, rpy_offset = self.visual_origins[urdf_link_name]
                    T_origin = np.eye(4)
                    T_origin[:3, :3] = R.from_euler('xyz', rpy_offset).as_matrix()
                    T_origin[:3, 3] = xyz_offset
                    T_base = T_base @ T_origin
             except: pass

             T_final = config.S_MATRIX @ T_base
             
             for extra_file in config.BASE_EXTRA_FILES:
                  mesh = self._create_mesh_item_from_file(extra_file)
                  current_T = T_final.copy()
                  if hasattr(config, 'BASE_EXTRA_OFFSETS') and extra_file in config.BASE_EXTRA_OFFSETS:
                       off = config.BASE_EXTRA_OFFSETS[extra_file]
                       T_off = np.eye(4); T_off[:3,3] = off
                       current_T = current_T @ T_off
                  
                  mesh.setTransform(matrix_to_qtransform(current_T))
                  self.view.addItem(mesh)

    def map_slider_to_rad(self, slider_val, joint_index):
        if joint_index >= len(self.joint_limits): return 0.0
        min_rad, max_rad = self.joint_limits[joint_index]
        percentage = (slider_val - config.SLIDER_RANGE_MIN) / (config.SLIDER_RANGE_MAX - config.SLIDER_RANGE_MIN)
        return min_rad + percentage * (max_rad - min_rad)

    def map_rad_to_slider(self, rad_val, joint_index):
        if joint_index >= len(self.joint_limits): return config.SLIDER_RANGE_MIN
        min_rad, max_rad = self.joint_limits[joint_index]
        rad_range = max_rad - min_rad
        if rad_range == 0: return (config.SLIDER_RANGE_MAX + config.SLIDER_RANGE_MIN) // 2
        percentage = (rad_val - min_rad) / rad_range
        val = config.SLIDER_RANGE_MIN + percentage * (config.SLIDER_RANGE_MAX - config.SLIDER_RANGE_MIN)
        return int(np.clip(val, config.SLIDER_RANGE_MIN, config.SLIDER_RANGE_MAX))

    def reset_sliders(self):
        # Reset to 0 degrees for all joints (or standby if desired)
        # Using 0s for simplicity as Home
        for i, slider in enumerate(self.sliders):
            slider.blockSignals(True)
            val = self.map_rad_to_slider(0.0, i)
            slider.setValue(val)
            slider.blockSignals(False)
        self.on_slider_changed()

    def on_slider_changed(self):
        angles = []
        for i, slider in enumerate(self.sliders):
            rad = self.map_slider_to_rad(slider.value(), i)
            angles.append(rad)
            self.labels[i].setText(f"{np.degrees(rad):.1f}°")
        
        # gui.py passes [0, J1, J2, J3, J4, J5, J6] to kinematics
        full_angles = np.concatenate(([0.0], np.array(angles)))
        self.update_robot_visuals(full_angles)

    def update_robot_visuals(self, angles):
        if not self.links_3d: return
        try:
            transforms = self.kinematics.forward_kinematics_full(angles)
        except Exception: return

        # Apply transforms
        # config.LINK_NAMES matches links_3d
        for i, config_name in enumerate(config.LINK_NAMES):
            if i >= len(self.links_3d) or i >= len(transforms): break
            
            mesh = self.links_3d[i]
            T = transforms[i]

            # Visual origin adjustment
            try:
                urdf_link_name = self.kinematics.chain.links[i].name
                if urdf_link_name in self.visual_origins:
                    xyz_offset, rpy_offset = self.visual_origins[urdf_link_name]
                    T_origin = np.eye(4)
                    T_origin[:3, :3] = R.from_euler('xyz', rpy_offset).as_matrix()
                    T_origin[:3, 3] = xyz_offset
                    T = T @ T_origin
            except: pass

            T_scene = config.S_MATRIX @ T
            mesh.setTransform(matrix_to_qtransform(T_scene))
            
        self.view.update()

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    
    # Modern Dark Style
    app.setStyle("Fusion")
    dark_palette = QtGui.QPalette()
    dark_palette.setColor(QtGui.QPalette.Window, QColor(53, 53, 53))
    dark_palette.setColor(QtGui.QPalette.WindowText, Qt.white)
    dark_palette.setColor(QtGui.QPalette.Base, QColor(35, 35, 35))
    dark_palette.setColor(QtGui.QPalette.AlternateBase, QColor(53, 53, 53))
    dark_palette.setColor(QtGui.QPalette.ToolTipBase, Qt.white)
    dark_palette.setColor(QtGui.QPalette.ToolTipText, Qt.white)
    dark_palette.setColor(QtGui.QPalette.Text, Qt.white)
    dark_palette.setColor(QtGui.QPalette.Button, QColor(53, 53, 53))
    dark_palette.setColor(QtGui.QPalette.ButtonText, Qt.white)
    dark_palette.setColor(QtGui.QPalette.BrightText, Qt.red)
    dark_palette.setColor(QtGui.QPalette.Link, QColor(42, 130, 218))
    dark_palette.setColor(QtGui.QPalette.Highlight, QColor(42, 130, 218))
    dark_palette.setColor(QtGui.QPalette.HighlightedText, Qt.black)
    app.setPalette(dark_palette)

    win = SimulationWindow()
    win.show()
    sys.exit(app.exec_())
