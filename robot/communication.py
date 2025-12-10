import serial
import numpy as np
import time
import serial.tools.list_ports
DEFAULT_BAUD_RATE = 115200 
DEFAULT_TIMEOUT = 0.01 
# === NOWOŚĆ: Mała pauza między wysłaniem J1, J2, J3... ===
# To daje STM32 czas na przetworzenie każdej komendy.
JOINT_SEND_DELAY = 0.005 # 5 milisekund

_serial_instance = None 

def open_serial_port(port_name: str, baud_rate: int = DEFAULT_BAUD_RATE, timeout: float = DEFAULT_TIMEOUT):
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
    global _serial_instance
    if _serial_instance is not None and _serial_instance.is_open:
        _serial_instance.close()
        _serial_instance = None
        print("Port szeregowy zamknięty.")


# === NOWA FUNKCJA POMOCNICZA ===
def send_single_angle_on_open_port(ser_handle: serial.Serial, joint_number: int, angle_deg: float):
    """
    Wysyła pojedynczą komendę kąta w formacie:
    "J1_90.21\r\n"
    """
    try:
        # Format: J1_90.21 (z podkreślnikiem)
        command = f"J{joint_number}_{angle_deg:.2f}\r\n"
        # print(f"→ {command.strip()}") # Opcjonalny debug
        ser_handle.write(command.encode('ascii'))

    except Exception as e:
        print(f"Błąd podczas wysyłania kąta J{joint_number}: {e}")
        raise

# === ZMIENIONA FUNKCJA GŁÓWNA ===
def send_angles(port_name: str, angles_rad: np.ndarray):
    """
    Zapewnia, że port jest otwarty i wysyła 6 KĄTÓW 
    jako SZEŚĆ OSOBNYCH WIADOMOŚCI.
    """
    global _serial_instance

    if port_name == "Brak portów" or port_name is None:
        raise serial.SerialException("Nie wybrano portu COM.")

    try:
        ser = open_serial_port(port_name) 

        if not (ser and ser.is_open):
             raise serial.SerialException(f"Port szeregowy {port_name} nie mógł zostać otwarty.")
        
        # 1. Konwersja na stopnie
        angles_deg = np.degrees(angles_rad)
        
        # 2. Wybór kątów (J1-J6)
        if len(angles_deg) == 7:
            angles_to_send = angles_deg[1:]  # J1–J6
        elif len(angles_deg) == 6:
            angles_to_send = angles_deg
        else:
            print(f"Błąd formatu: Otrzymano {len(angles_deg)} kątów.")
            return

        # 3. Pętla wysyłająca 6 osobnych komend
        for i in range(6):
            joint_num = i + 1
            angle_val = angles_to_send[i]
            
            send_single_angle_on_open_port(ser, joint_num, angle_val)
            #print(f'Wysłano kąt J{joint_num}: {angle_val:.2f}°') # Debug
            # 4. Krytyczna pauza, aby STM32 nadążył
            time.sleep(JOINT_SEND_DELAY) 

        # print(f'Wysłano 6 kątów (osobno)') # Debug

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
    (Bez zmian)
    """
    global _serial_instance
    
    if port_name == "Brak portów" or port_name is None:
        return None 
    
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

def send_command(port_name: str, command: str):
    """
    Wysyła pojedynczą komendę tekstową (np. "HOME", "VAC_ON") przez UART.
    (Bez zmian)
    """
    global _serial_instance

    if port_name == "Brak portów" or port_name is None:
        raise serial.SerialException("Nie wybrano portu COM.")

    try:
        ser = open_serial_port(port_name)
        if ser and ser.is_open:
            command_to_send = f"{command}\r\n"
            #print(f"→ {command_to_send.strip()}") # Debug
            ser.write(command_to_send.encode('ascii'))
        else:
            raise serial.SerialException("Nie udało się otworzyć portu szeregowego.")
    except Exception as e:
        print(f"Błąd wysyłania komendy '{command}': {e}")
        close_serial_port()
        raise