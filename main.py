import tkinter as tk
from ui.main_window import DeltaRobotGUI
import os

def main():
    # Ensure directories exist
    os.makedirs("models", exist_ok=True)
    os.makedirs("logs", exist_ok=True)
    
    root = tk.Tk()
    app = DeltaRobotGUI(root)
    
    print("Starting Delta Robot Pro...")
    root.mainloop()

if __name__ == "__main__":
    main()
