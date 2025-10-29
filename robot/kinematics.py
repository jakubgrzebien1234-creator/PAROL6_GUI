import numpy as np
from ikpy.chain import Chain
import xml.etree.ElementTree as ET
from scipy.spatial.transform import Rotation as R

class RobotKinematics:
    """
    Obsługuje wszystkie obliczenia kinematyczne (FK, IK)
    na podstawie pliku URDF.
    """
    def __init__(self, urdf_path, active_links_mask):
        try:
            self.chain = Chain.from_urdf_file(
                urdf_path,
                active_links_mask=active_links_mask
            )
            self.chain.max_iterations = 200
            self.chain.convergence_limit = 1e-6
            print("URDF wczytany pomyślnie.")
            
            self.joint_limits_rad = self._load_joint_limits()
            self.visual_origins = self._load_visual_origins(urdf_path)
            
        except Exception as e:
            print(f"Krytyczny błąd wczytywania URDF: {e}. Używam atrapy.")
            # Stworzymy atrapy
            class MockLink:
                name = "mock"
                has_rotation = True
                bounds = (-np.pi, np.pi)
            class MockChain:
                links = [MockLink() for i in range(7)]
                def forward_kinematics(self, angles, full_kinematics=False):
                    identity = np.eye(4)
                    if full_kinematics: return [identity] * len(self.links)
                    return identity
                def inverse_kinematics(self, *args, **kwargs):
                    return np.zeros(7)
            self.chain = MockChain()
            self.joint_limits_rad = [(-1.7, 1.7), (-0.98, 1.0), (-2.0, 1.3), (-2.0, 2.0), (-2.1, 2.1), (-3.1, 3.1)]
            self.visual_origins = {}

    def _load_joint_limits(self):
        """Pobiera limity stawów z załadowanego łańcucha."""
        limits = []
        for link in self.chain.links[1:]: # Pomiń bazę
            if link.has_rotation:
                limits.append(link.bounds)
            else:
                limits.append((-np.pi, np.pi))
        print(f"Wczytano limity stawów (rad): {limits}")
        return limits

    def _load_visual_origins(self, urdf_path):
        """Parsuje URDF w poszukiwaniu tagów 'visual' > 'origin'."""
        origins = {}
        try:
            tree = ET.parse(urdf_path)
            root = tree.getroot()
            for link in root.findall('link'):
                name = link.attrib['name']
                visual = link.find('visual')
                if visual is not None:
                    origin = visual.find('origin')
                    if origin is not None:
                        xyz = [float(x) for x in origin.attrib.get('xyz', '0 0 0').split()]
                        rpy = [float(r) for r in origin.attrib.get('rpy', '0 0 0').split()]
                        origins[name] = (xyz, rpy)
        except Exception as e:
            print(f"Błąd parsowania XML URDF dla 'visual': {e}")
        return origins

    def get_joint_limits(self):
        return self.joint_limits_rad

    def get_visual_origins(self):
        return self.visual_origins

    def forward_kinematics(self, angles):
        """Zwraca macierz 4x4 tylko dla efektora końcowego."""
        return self.chain.forward_kinematics(angles)

    def forward_kinematics_full(self, angles):
        """Zwraca listę macierzy 4x4 dla wszystkich linków (do rysowania)."""
        return self.chain.forward_kinematics(angles, full_kinematics=True)

    def inverse_kinematics(self, target_pos, target_orient_3x3, initial_pos):
        """Oblicza kinematykę odwrotną."""
        return self.chain.inverse_kinematics(
            target_position=target_pos,
            target_orientation=target_orient_3x3,
            orientation_mode='all',
            initial_position=initial_pos
        )
