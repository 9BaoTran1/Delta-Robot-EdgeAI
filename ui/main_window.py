import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import cv2
from PIL import Image, ImageTk
import threading
import time
import queue

# Import custom modules
from vision.detector import YOLODetector
from vision.tracker import StableTracker
from core.mapper import RobotMapper
from core.robot_comm import RobotController

class DeltaRobotGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Delta Robot Pro Control Center v3.0")
        self.root.geometry("1100x700")
        
        # Initialize Core Systems
        self.robot = RobotController()
        self.robot.on_message_received = self.on_robot_msg
        
        self.detector = YOLODetector()
        self.tracker = StableTracker()
        self.mapper = RobotMapper()
        
        # UI State
        self.camera_running = False
        self.frame_queue = queue.Queue(maxsize=2)
        self.detect_enabled = False
        self.fps = 0
        self.cap = None
        
        self.create_widgets()
        self.update_display()
        self.log("System Initialized with Modular Architecture")

    def on_robot_msg(self, msg):
        self.log(f"ROBOT: {msg}")
        # Add logic to update UI labels if needed

    def log(self, msg):
        timestamp = time.strftime('%H:%M:%S')
        if hasattr(self, 'txt_log'):
            self.txt_log.config(state="normal")
            self.txt_log.insert(tk.END, f"[{timestamp}] {msg}\n")
            self.txt_log.see(tk.END)
            self.txt_log.config(state="disabled")

    def create_widgets(self):
        # (I will simplify this for the demo, you can copy your widget creation logic here)
        # Main structure
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill="both", expand=True, padx=5, pady=5)
        
        # Tab 1: Camera
        self.tab_cam = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_cam, text="Vision & AI")
        
        self.lbl_cam = tk.Label(self.tab_cam, text="Camera Offline", bg="black", fg="white")
        self.lbl_cam.pack(fill="both", expand=True, side="left")
        
        # Control Panel (Right)
        ctrl_panel = ttk.Frame(self.tab_cam, width=300)
        ctrl_panel.pack(fill="y", side="right")
        
        tk.Button(ctrl_panel, text="START CAMERA", command=self.start_camera, bg="green", fg="white").pack(fill="x", pady=5)
        tk.Button(ctrl_panel, text="STOP CAMERA", command=self.stop_camera, bg="red", fg="white").pack(fill="x", pady=5)
        
        self.var_detect = tk.BooleanVar()
        tk.Checkbutton(ctrl_panel, text="Enable AI Detection", variable=self.var_detect, command=self.toggle_detect).pack(pady=10)
        
        # Tab 2: Logs
        self.tab_log = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_log, text="System Logs")
        self.txt_log = scrolledtext.ScrolledText(self.tab_log, bg="#1e1e1e", fg="#00ff00")
        self.txt_log.pack(fill="both", expand=True)

    def toggle_detect(self):
        self.detect_enabled = self.var_detect.get()
        self.log(f"Detection: {'ON' if self.detect_enabled else 'OFF'}")

    def start_camera(self):
        if not self.camera_running:
            self.cap = cv2.VideoCapture(0) # Default camera
            self.camera_running = True
            threading.Thread(target=self.capture_loop, daemon=True).start()
            self.log("Camera Thread Started")

    def stop_camera(self):
        self.camera_running = False
        if self.cap:
            self.cap.release()
        self.log("Camera Thread Stopped")

    def capture_loop(self):
        while self.camera_running:
            ret, frame = self.cap.read()
            if not ret: break
            
            # AI Inference
            if self.detect_enabled:
                dets = self.detector.detect(frame)
                tracked = self.tracker.update(dets)
                
                for cx, cy, tid, cid, score in tracked:
                    cv2.circle(frame, (cx, cy), 10, (0, 255, 0), -1)
                    rx, ry = self.mapper.pixel_to_robot(cx, cy)
                    cv2.putText(frame, f"ID:{tid} R:({rx:.1f},{ry:.1f})", (cx, cy-20), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            if not self.frame_queue.full():
                self.frame_queue.put(frame)
            time.sleep(0.01)

    def update_display(self):
        if not self.frame_queue.empty():
            frame = self.frame_queue.get()
            img = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            photo = ImageTk.PhotoImage(image=img)
            self.lbl_cam.configure(image=photo)
            self.lbl_cam.image = photo
        self.root.after(16, self.update_display)
