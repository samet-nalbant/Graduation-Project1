import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox
from gcs import Ui_MainWindow
from PyQt5 import QtWidgets
class MainApp(QMainWindow):
    def __init__(self):
        super(MainApp, self).__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.ui.connectProcesses()
        if self.ui.observerPID == None or self.ui.targetPID == None:
            dlg = QMessageBox(self)
            dlg.setWindowTitle("ERROR")
            dlg.setText("Please Connect The Vehicles")
            dlg.exec()
            exit()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainApp()
    window.show()
    sys.exit(app.exec_())
