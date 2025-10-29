from PyQt5.QtCore import QObject, QTimer, pyqtSlot, pyqtSignal, Qt
import numpy as np
import robot.config as config

# === Atrapa modułu komunikacyjnego ===
# Ta atrapa będzie używana, jeśli import `robot.communication` się nie powiedzie
class MockComm:
    def send_angles(self, port, angles):
        # Nie drukuj nic, jeśli port to "Brak portów"
        if port and port != "Brak portów":
            print(f"[MOCK UART -> {port}] Wysyłanie kątów: {np.round(angles, 2)}")
    
    def close_serial_port(self):
        print("[MOCK UART] Zamykanie portu.")

try:
    import robot.communication as comm
    print("Worker: Zaimportowano prawdziwy moduł `robot.communication`.")
except ImportError:
    print("Worker: Nie można załadować `robot.communication`, używam atrapy.")
    comm = MockComm()
# ======================================


class RobotWorker(QObject):
    # Sygnał emitowany w każdym kroku animacji, wysyła aktualne kąty
    angles_updated = pyqtSignal(np.ndarray)
    # Sygnał emitowany do zmiany tekstu statusu w GUI
    status_updated = pyqtSignal(str, str) # (text, color)

    def __init__(self, initial_angles):
        super().__init__()
        
        # === POPRAWKA TIMERA ===
        # Timer MUSI być stworzony w wątku, w którym będzie działał.
        # Nie tworzymy go tutaj (w __init__), ale w `start_move` (leniwie).
        self.timer = None
        # ======================
        
        self.previous_angles = initial_angles.copy()
        self.target_angles = None
        self.anim_step = 0
        self.com_port = None
        print("Worker gotowy.")

    @pyqtSlot(str)
    def set_com_port(self, port):
        """Slot: Ustawia port COM do użycia."""
        self.com_port = port
        print(f"Worker używa teraz portu: {self.com_port}")

    @pyqtSlot(np.ndarray)
    def start_move(self, target_angles):
        """Slot: Rozpoczyna nową sekwencję ruchu."""
        print("Worker otrzymał nowe zadanie ruchu.")
        self.target_angles = target_angles.copy()
        self.anim_step = 0

        # === POPRAWKA TIMERA ===
        # Stwórz timer przy pierwszym wywołaniu, GDY JESTEŚMY JUŻ W WĄTKU
        if self.timer is None:
            self.timer = QTimer()
            # Użyj precyzyjnego timera dla płynniejszej animacji
            self.timer.setTimerType(Qt.PreciseTimer) 
            self.timer.timeout.connect(self.step_animation)
            print("Worker: Timer utworzony w wątku roboczym.")
        # ======================

        if not self.timer.isActive():
            self.timer.start(config.ANIM_INTERVAL_MS)
    
    def get_previous_angles(self):
        """Zwraca ostatnią znaną pozycję kątów."""
        return self.previous_angles

    def step_animation(self):
        """Główna pętla timera. Wykonuje jeden krok animacji."""
        if self.target_angles is None:
            if self.timer: # Dodano sprawdzenie, czy timer istnieje
                self.timer.stop()
            self.status_updated.emit("Status: Błąd (brak celu)", "red")
            return

        self.anim_step += 1
        t = min(1.0, self.anim_step / float(config.ANIM_STEPS_TOTAL))
        
        interp_angles = self.previous_angles + t * (self.target_angles - self.previous_angles)
        
        # 1. Wyślij zaktualizowane kąty do GUI (aby przerysować model 3D)
        self.angles_updated.emit(interp_angles)
        
        # 2. Wyślij kąty do prawdziwego robota (jeśli port jest ustawiony)
        if self.com_port and self.com_port != "Brak portów":
            angles_to_send_deg = np.degrees(interp_angles[1:]) # Pomiń bazę
            try:
                comm.send_angles(self.com_port, angles_to_send_deg)
                self.status_updated.emit(f"Status: W ruchu ({self.anim_step}/{config.ANIM_STEPS_TOTAL})", "blue")
            except Exception as e:
                print(f"Błąd UART: {e}")
                self.status_updated.emit("Status: Błąd UART", "red")
                if self.timer: self.timer.stop()
                return
        else:
             # Pomiń błąd, jeśli używamy atrapy
             if "MockComm" not in str(comm):
                self.status_updated.emit("Status: Brak portu COM", "orange")


        # 3. Sprawdź, czy animacja się zakończyła
        if self.anim_step >= config.ANIM_STEPS_TOTAL:
            self.previous_angles = self.target_angles.copy()
            self.target_angles = None
            if self.timer: self.timer.stop()
            self.status_updated.emit("Status: Osiągnięto cel", "green")

