from PyQt5.QtCore import QObject, QTimer, pyqtSlot, pyqtSignal
import numpy as np
import time

# Użyj "atrapy" comm, jeśli prawdziwy moduł zawiedzie
try:
    import robot.communication as comm
    # UPEWNIJ SIĘ, ŻE PRAWDZIWY MODUŁ TEŻ BĘDZIE MIAŁ 'send_command'
except ImportError:
    print("OSTRZEŻENIE: Nie można zaimportować 'robot.communication'. Używam atrapy.")
    
    # ATTRAPA MUSI ZAWIERAĆ WSZYSTKIE FUNKCJE, KTÓRYCH UŻYWA WORKER
    class CommMock:
        def open_serial_port(self, port_name, baud_rate=115200, timeout=0.1):
            print(f"[ATRAPA] Otwieram port {port_name}")
            return True # Zwróć symulowany uchwyt (True)
            
        def close_serial_port(self):
            print("[ATRAPA] Zamykam port.")
            
        def read_line(self, port_name):
            # Zwróć None, aby symulować brak danych
            return None
            
        def send_angles(self, port_name, angles):
            print(f"[ATRAPA] Wysyłam {len(angles)} kątów na {port_name}")
            time.sleep(0.05)

        # --- NOWA FUNKCJA ---
        def send_command(self, port_name, command_string):
            """Wysyła pojedynczą komendę tekstową, np. '$HOME'."""
            print(f"[ATRAPA] Wysyłam komendę '{command_string}' na {port_name}")
            time.sleep(0.05)
            
    comm = CommMock()


class RobotWorker(QObject):
    angles_updated = pyqtSignal(np.ndarray)
    status_updated = pyqtSignal(str, str)
    limit_switch_status = pyqtSignal(int, bool)

    def __init__(self, initial_angles):
        super().__init__()
        self.current_port = None
        self.previous_angles = initial_angles
        
        # Timer do regularnego sprawdzania portu UART
        self.poll_timer = QTimer(self)
        self.poll_timer.timeout.connect(self._update_step)
        # Ustaw jak często ma sprawdzać (np. co 20ms = 50Hz)
        self.poll_timer.start(20) 
        print("[WORKER] Uruchomiono timer odczytu UART.")

    @pyqtSlot()
    def _update_step(self):
        """Główna pętla timera, wywoływana co 20ms."""
        self._check_serial_feedback()

    def _check_serial_feedback(self):
        """Sprawdza port szeregowy w poszukiwaniu danych."""
        
        # Nie próbuj czytać, jeśli port nie jest wybrany
        if self.current_port is None or self.current_port == "Brak portów":
            return
            
        try:
            # Użyj nowej funkcji comm.read_line()
            # Musimy przekazać port_name, aby pasowało do logiki comm
            line = comm.read_line(self.current_port) 
            
            if not line:
                return # Nic nie przyszło

            # Mamy linię, parsujemy
            print(f"[WORKER] Otrzymano z UART: {line}") # Debug
            
            if line.startswith("$H"):
                # Krańcówka WCIŚNIĘTA
                try:
                    joint_num_str = line[2] # Pobierz cyfrę (np. "3")
                    joint_index = int(joint_num_str) - 1 # Konwertuj na indeks (0-5)
                    if 0 <= joint_index <= 5:
                        self.limit_switch_status.emit(joint_index, True)
                except Exception as e:
                    print(f"Błąd parsowania ramki $H: {e}")
                    
            elif line.startswith("$R"):
                # Krańcówka ZWOLNIONA
                try:
                    joint_num_str = line[2]
                    joint_index = int(joint_num_str) - 1
                    if 0 <= joint_index <= 5:
                        self.limit_switch_status.emit(joint_index, False)
                except Exception as e:
                    print(f"Błąd parsowania ramki $R: {e}")

        except Exception as e:
            # Ogólny błąd odczytu
            print(f"[WORKER] Błąd pętli odczytu: {e}")
            pass 

    def get_previous_angles(self):
        return self.previous_angles

    @pyqtSlot(str)
    def set_com_port(self, port_name):
        """Slot: Ustawia port COM do użycia."""
        if self.current_port == port_name:
            return # Bez zmian
            
        print(f"[WORKER] Zmieniam port na {port_name}")
        
        comm.close_serial_port()
        self.current_port = port_name # Ustaw nowy port
        
        if port_name != "Brak portów":
            try:
                # Spróbuj otworzyć port od razu,
                # aby czytanie mogło się rozpocząć.
                comm.open_serial_port(port_name)
                # Wyślij kąty, aby go zainicjować
                self.start_move(self.previous_angles)
            except Exception as e:
                self.status_updated.emit(f"Błąd: Port {port_name} nie działa", "red")
                self.current_port = None
        else:
             self.current_port = None # Ustaw na None, jeśli wybrano "Brak portów"

    @pyqtSlot(np.ndarray)
    def start_move(self, target_angles_rad):
        """
        Slot: Odbiera polecenie ruchu z GUI i wysyła je do robota.
        """
        if self.current_port is None or self.current_port == "Brak portów":
            self.status_updated.emit("Błąd: Nie wybrano portu", "red")
            # Mimo błędu, aktualizuj GUI
            self.previous_angles = target_angles_rad
            self.angles_updated.emit(target_angles_rad)
            return

        try:
            self.status_updated.emit("Wysyłanie...", "orange")
            
            # Wywołaj funkcję wysyłającą (używa logiki z comm)
            comm.send_angles(self.current_port, target_angles_rad)
            
            self.status_updated.emit("Wysłano (J1-J6)", "green")
            
            # Zaktualizuj stan wewnętrzny i GUI
            self.previous_angles = target_angles_rad
            self.angles_updated.emit(target_angles_rad)

        except Exception as e:
            # Złap błędy z communication.py (np. port odłączony)
            print(f"Worker: Błąd podczas wysyłania: {e}")
            self.status_updated.emit(f"Błąd UART: {e}", "red")
            self.previous_angles = target_angles_rad
            self.angles_updated.emit(target_angles_rad)

    # --- NOWY SLOT DLA PRZYCISKU HOME ---
    @pyqtSlot()
    def start_homing(self):
        """Slot: Wysyła specjalną komendę 'HOME' do robota."""
        
        # Sprawdź, czy port jest wybrany
        if self.current_port is None or self.current_port == "Brak portów":
            self.status_updated.emit("Błąd: Nie wybrano portu", "red")
            return

        try:
            self.status_updated.emit("Wysyłanie $HOME...", "orange")
            
            # Użyj nowej funkcji z 'comm' do wysłania komendy tekstowej
            comm.send_command(self.current_port, "HOME") 
            
            self.status_updated.emit("Wysłano HOME", "green")
            
        except Exception as e:
            print(f"Worker: Błąd podczas wysyłania HOME: {e}")
            self.status_updated.emit(f"Błąd UART: {e}", "red")


    @pyqtSlot(str)
    def set_gripper_state(self, command):
        """
        Odbiera polecenie dla chwytaka (np. "VAC_ON" lub "VAC_OFF")
        i wysyła je przez UART.
        """
        print(f"Worker: Otrzymano polecenie chwytaka: {command}")
        
        # 1. Poprawione sprawdzenie: Użyj 'self.current_port' (tak jak w start_homing)
        if self.current_port is None or self.current_port == "Brak portów":
            print("Worker: Brak portu COM. Polecenie chwytaka nie wysłane.")
            self.status_updated.emit("Status: Brak portu COM", "orange")
            return # Zakończ funkcję
            
        # 2. Poprawione wywołanie: Użyj globalnego 'comm', a nie 'self.comm'
        try:
            self.status_updated.emit(f"Wysyłanie {command}...", "orange")
            
            # Użyj globalnej funkcji 'comm' i przekaż 'self.current_port'
            comm.send_command(self.current_port, command) 
            
            self.status_updated.emit(f"Wysłano {command}", "blue") # Używam 'blue' dla spójności
            
        except Exception as e:
            print(f"Błąd wysyłania polecenia chwytaka: {e}")
            self.status_updated.emit(f"Status: Błąd {command}", "red")