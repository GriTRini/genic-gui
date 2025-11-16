# main.py
import sys
from PySide6.QtWidgets import QApplication
from src.gui import MainWindow

if __name__ == "__main__":
    # 🚨 GUI 환경 오류 방지 (WSL2 등)
    # import os
    # os.environ['QT_QPA_PLATFORM'] = 'xcb' 
    
    # 1. 파일 구조 확인: 이 main.py 파일과 ros2_node.py, gui.py, data_logger.py가 같은 폴더에 있어야 합니다.
    
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())