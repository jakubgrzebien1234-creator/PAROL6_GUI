import serial
import numpy as np
import time
import serial.tools.list_ports

DEFAULT_BAUD_RATE = 115200 
DEFAULT_TIMEOUT = 0.01 

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
     
        _serial_instance.reset_input_buffer()
        _serial_instance.reset_output_buffer()
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

def close_serial_port(port_name=None):
    global _serial_instance
    if _serial_instance is not None and _serial_instance.is_open:
        _serial_instance.close()
        _serial_instance = None
        print("Port szeregowy zamknięty.")

def send_angles(port_name: str, angles_rad: np.ndarray):
    """
    Konwertuje radiany na stopnie i wysyła je w JEDNEJ paczce:
    Format: J_v1,v2,v3,v4,v5,v6\n
    """
    global _serial_instance

    if port_name == "Brak portów" or port_name is None:
        raise serial.SerialException("Nie wybrano portu COM.")

    try:
        if _serial_instance and _serial_instance.is_open and _serial_instance.port == port_name:
            ser = _serial_instance
        else:
            ser = open_serial_port(port_name)

        if not (ser and ser.is_open):
             raise serial.SerialException(f"Port szeregowy {port_name} nie jest otwarty.")
        
        angles_deg = np.degrees(angles_rad)
        
        if len(angles_deg) == 7:
            j = angles_deg[1:] 
        elif len(angles_deg) == 6:
            j = angles_deg
        else:
            print(f"Błąd formatu: Otrzymano {len(angles_deg)} kątów.")
            return

        command = "J_{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}\n".format(
            j[0], j[1], j[2], j[3], j[4], j[5]
        )

        ser.write(command.encode('ascii'))
        

    except serial.SerialException as e:
        print(f"Błąd komunikacji UART z {port_name}: {e}")
        close_serial_port() 
        raise
    except Exception as e:
        print(f"Nieznany błąd podczas wysyłania kątów: {e}")
        close_serial_port() 
        raise

def read_line(port_name: str):
    """
    Odczytuje jedną linię z portu.
    """
    global _serial_instance
    
    if port_name == "Brak portów" or port_name is None:
        return None 
    
    try:
        if _serial_instance and _serial_instance.is_open and _serial_instance.port == port_name:
            ser = _serial_instance
        else:
            return None 

        if ser.in_waiting > 0:
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
    Wysyła komendy tekstowe np. HOME, VAC_ON
    """
    global _serial_instance

    if port_name == "Brak portów" or port_name is None:
        return

    try:
        if _serial_instance and _serial_instance.is_open and _serial_instance.port == port_name:
            ser = _serial_instance
        else:
            ser = open_serial_port(port_name)

        if ser and ser.is_open:
            if not command.endswith('\n'):
                command += '\n'
            ser.write(command.encode('ascii'))
        else:
            print("Port zamknięty, nie można wysłać komendy.")
    except Exception as e:
        print(f"Błąd wysyłania komendy '{command}': {e}")
        close_serial_port()