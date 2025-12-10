
import sys
from PyQt5 import QtWidgets
from robot.gui import MainWindow

# ------------------------
# Uruchomienie Aplikacji
# ------------------------
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    # Ustaw styl, aby poprawić wygląd na różnych systemach




    try:
        app.setStyle("Fusion")
    except Exception as e:
        print(f"Nie można ustawić stylu Fusion: {e}")

    # Utwórz instancję głównego okna z modułu gui
    window = MainWindow()
    
    # Pokaż okno
    window.show()
    
    # Uruchom główną pętlę aplikacji
    sys.exit(app.exec_())