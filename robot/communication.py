import serial
import numpy as np
import time

DEFAULT_BAUD_RATE = 115200 
DEFAULT_TIMEOUT = 0.01 # 10ms

_serial_instance = None 

def open_serial_port(port_name: str, baud_rate: int = DEFAULT_BAUD_RATE, timeout: float = DEFAULT_TIMEOUT):
    """
    Otwiera port szeregowy i zwraca jego uchwyt.
    """
    global _serial_instance
    
    if _serial_instance is not None and _serial_instance.is_open:
        if _serial_instance.port == port_name:
            return _serial_instance
        else:
            print(f"Zamykam istniejący port {_serial_instance.port} przed otwarciem {port_name}.")
            _serial_instance.close()
            _serial_instance = None 

    try:
        _serial_instance = serial.Serial(
            port=port_name,
            baudrate=baud_rate,
            timeout=timeout
        )
        time.sleep(0.1) 
        print(f"Port szeregowy {port_name} otwarty.")
        return _serial_instance
    except serial.SerialException as e:
        _serial_instance = None 
        print(f"Błąd otwierania portu {port_name}: {e}")
        raise
    except Exception as e:
        _serial_instance = None 
        print(f"Nieznany błąd podczas otwierania portu {port_name}: {e}")
        raise

def close_serial_port():
    """
    Zamyka globalną instancję portu szeregowego, jeśli jest otwarta.
    """
    global _serial_instance
    if _serial_instance is not None and _serial_instance.is_open:
        _serial_instance.close()
        _serial_instance = None
        print("Port szeregowy zamknięty.")

def send_single_angle(port_name: str, joint_number: int, angle_deg: float):
    """
    Wysyła pojedynczy kąt dla wskazanego przegubu (np. J1) przez UART.
    (Zaktualizowano format na J<num> <kąt>)
    """
    global _serial_instance

    if port_name == "Brak portów" or port_name is None:
        raise serial.SerialException("Nie wybrano portu COM.")

    try:
        ser = open_serial_port(port_name)
        if ser and ser.is_open:
            # <--- POPRAWKA FORMATU (dodano spację) --->
            command = f"J{joint_number} {int(round(angle_deg))}\r\n"
            print(f"→ {command.strip()}")
            ser.write(command.encode('ascii'))
        else:
            raise serial.SerialException("Nie udało się otworzyć portu szeregowego.")
    except Exception as e:
        print(f"Błąd wysyłania komendy: {e}")
        close_serial_port()
        raise

def send_angles_on_open_port(ser_handle: serial.Serial, angles_rad: np.ndarray):
    """
    Wysyła zestaw 6 kątów przez otwarty port.
    Konwertuje RADIANY (z GUI) na STOPNIE (dla mikrokontrolera).
    """
    try:
        # 1. Konwersja RADIANY -> STOPNIE
        angles_deg = np.degrees(angles_rad)

        # 2. Wybierz kąty do wysłania (pomiń bazę o indeksie 0)
        if len(angles_deg) == 7:
            angles_to_send = angles_deg[1:] # Bierzemy kąty J1-J6
        elif len(angles_deg) == 6:
            angles_to_send = angles_deg
        else:
            print(f"Błąd formatu: Otrzymano {len(angles_deg)} kątów.")
            return

        # 3. Pętla wysyłająca 6 komend
        for i in range(6):
            joint_num = i + 1  # Stawy J1 do J6
            angle = angles_to_send[i]
            
            # <--- POPRAWKA FORMATU (dodano spację) --->
            command = f"J{joint_num} {int(round(angle))}\r\n"
            
            print(f"→ {command.strip()}") 
            ser_handle.write(command.encode('ascii'))
            
            # <--- POPRAWKA TIMINGU --->
            # Ponownie zwiększono opóźnienie. 20ms to wciąż za mało.
            time.sleep(0.050) # 50ms przerwy
            # <--- KONIEC POPRAWKI --->

    except Exception as e:
        print(f"Błąd podczas wysyłania zestawu kątów: {e}")
        raise # Przekaż błąd dalej do send_angles


def send_angles(port_name: str, angles_rad: np.ndarray):
    """
    Zapewnia, że port jest otwarty i wysyła pełen zestaw kątów (J1-J6).
    """
    global _serial_instance

    if port_name == "Brak portów" or port_name is None:
        raise serial.SerialException("Nie wybrano portu COM.")

    try:
        current_serial_handle = open_serial_port(port_name) 
        
        if current_serial_handle and current_serial_handle.is_open:
            send_angles_on_open_port(current_serial_handle, angles_rad)
        else:
            raise serial.SerialException(f"Port szeregowy {port_name} nie mógł zostać otwarty.")

    except serial.SerialException as e:
        print(f"Błąd komunikacji UART z {port_name}: {e}")
        close_serial_port() 
        raise 
    except Exception as e:
        print(f"Nieznany błąd podczas wysyłania kątów przez UART: {e}")
        close_serial_port()
        raise

def read_line(port_name: str):
    """
    Sprawdza, czy na porcie są dane i odczytuje jedną linię.
    Używa globalnego uchwytu, aby nie walczyć o port z funkcjami wysyłającymi.
    """
    global _serial_instance
    
    if port_name == "Brak portów" or port_name is None:
        return None # Port nie jest ustawiony
    
    try:
        ser = open_serial_port(port_name) 
        
        if ser and ser.is_open and ser.in_waiting > 0:
            line_bytes = ser.readline()
            if not line_bytes:
                return None
            
            line_str = line_bytes.decode('ascii', errors='ignore').strip()
            
            if line_str:
                return line_str
            else:
                return None
        
        return None
        
    except Exception as e:
        return None

# --- NOWA FUNKCJA DLA KOMENDY HOME ---
def send_command(port_name: str, command: str):
    """
    Wysyła pojedynczą komendę tekstową (np. "$HOME") przez UART.
    """
    global _serial_instance

    if port_name == "Brak portów" or port_name is None:
        raise serial.SerialException("Nie wybrano portu COM.")

    try:
        ser = open_serial_port(port_name)
        if ser and ser.is_open:
            # Formatuj komendę - dodaj znak końca linii
            command_to_send = f"{command}\r\n"
            print(f"→ {command_to_send.strip()}")
            ser.write(command_to_send.encode('ascii'))
        else:
            raise serial.SerialException("Nie udało się otworzyć portu szeregowego.")
    except Exception as e:
        print(f"Błąd wysyłania komendy '{command}': {e}")
        close_serial_port()
        raise