# robot/communication.py

import serial
import numpy as np
import time

DEFAULT_BAUD_RATE = 115200 # Sprawdź, czy to jest zgodne z Twoim mikrokontrolerem
DEFAULT_TIMEOUT = 0.1 

_serial_instance = None # Prywatna zmienna do przechowywania instancji Serial

def open_serial_port(port_name: str, baud_rate: int = DEFAULT_BAUD_RATE, timeout: float = DEFAULT_TIMEOUT):
    """
    Otwiera port szeregowy i zwraca jego uchwyt.
    Jeśli port jest już otwarty, ale jest to inny port, zamyka go i otwiera nowy.
    Jeśli jest otwarty i to ten sam port, zwraca istniejący uchwyt.
    """
    global _serial_instance
    
    # Jeśli port jest już otwarty
    if _serial_instance is not None and _serial_instance.is_open:
        # Jeśli to ten sam port, po prostu go zwróć
        if _serial_instance.port == port_name:
            return _serial_instance
        else:
            # Jeśli to inny port, zamknij obecny
            print(f"Zamykam istniejący port {_serial_instance.port} przed otwarciem {port_name}.")
            _serial_instance.close()
            _serial_instance = None # Resetuj globalną instancję

    # Spróbuj otworzyć nowy port
    try:
        _serial_instance = serial.Serial(
            port=port_name,
            baudrate=baud_rate,
            timeout=timeout
        )
        time.sleep(0.1) # Daj portowi czas na inicjalizację
        print(f"Port szeregowy {port_name} otwarty.")
        return _serial_instance
    except serial.SerialException as e:
        _serial_instance = None # Upewnij się, że instancja jest None w przypadku błędu
        print(f"Błąd otwierania portu {port_name}: {e}")
        raise
    except Exception as e:
        _serial_instance = None # Upewnij się, że instancja jest None w przypadku błędu
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

def send_angles_on_open_port(ser: serial.Serial, angles_deg: np.ndarray):
    """
    Wysyła komendy JXYY dla każdego jointa na JUŻ OTWARTYM porcie.
    """
    if not ser.is_open:
        raise serial.SerialException("Port szeregowy nie jest otwarty!")

    for i, angle in enumerate(angles_deg):
        joint_number = i + 1
        angle_int = int(round(angle)) 
        command = f"J{joint_number}{angle_int}\n" 
        
        try:
            ser.write(command.encode('ascii'))
            # ser.flush() # Odkomentuj, jeśli masz problemy z buforowaniem na mikrokontrolerze
        except serial.SerialException as e:
            raise serial.SerialException(f"Błąd zapisu na port {ser.port}: {e}")
        except Exception as e:
            raise Exception(f"Nieznany błąd podczas zapisu na port {ser.port}: {e}")

def send_angles(port_name: str, angles_deg: np.ndarray):
    """
    Zapewnia, że port jest otwarty, wysyła kąty, a następnie utrzymuje port otwarty
    lub zamyka go w przypadku błędu.
    Ta funkcja jest przeznaczona do użycia w iteracyjnej pętli (np. animacji),
    gdzie port powinien być otwarty przez cały czas trwania ruchu.
    """
    global _serial_instance

    if port_name == "Brak portów" or port_name is None:
        raise serial.SerialException("Nie wybrano portu COM.")

    try:
        # Upewnij się, że port jest otwarty i to ten właściwy
        current_serial_handle = open_serial_port(port_name) 
        
        # Jeśli port jest otwarty, wyślij kąty
        if current_serial_handle and current_serial_handle.is_open:
            send_angles_on_open_port(current_serial_handle, angles_deg)
        else:
            # Powinno być obsłużone przez open_serial_port, ale to dodatkowa weryfikacja
            raise serial.SerialException(f"Port szeregowy {port_name} nie mógł zostać otwarty.")

    except serial.SerialException as e:
        # Złap wyjątki związane z portem szeregowym
        print(f"Błąd komunikacji UART z {port_name}: {e}")
        # W przypadku błędu zawsze próbujemy zamknąć port
        close_serial_port() 
        raise # Ponownie zgłoś błąd do głównego programu
    except Exception as e:
        # Złap inne, nieprzewidziane wyjątki
        print(f"Nieznany błąd podczas wysyłania kątów przez UART: {e}")
        close_serial_port()
        raise # Ponownie zgłoś błąd do głównego programu