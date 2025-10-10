
import cv2
import threading
import os
import customtkinter as ctk

VIDEO_DIR = os.path.join(os.path.dirname(__file__), 'videos')
VIDEO_PATH = os.path.join(VIDEO_DIR, 'captured_video.mp4')
if not os.path.exists(VIDEO_DIR):
    os.makedirs(VIDEO_DIR)
ctk.set_appearance_mode("System")
ctk.set_default_color_theme("blue")


class VideoCaptureApp(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("Video Capture")
        self.geometry("600x520")

        self.label = ctk.CTkLabel(self, text="Video Capture Control", font=("Arial", 18))
        self.label.pack(pady=10)

        self.source_var = ctk.StringVar(value="Webcam")
        self.webcam_radio = ctk.CTkRadioButton(self, text="Webcam", variable=self.source_var, value="Webcam")
        self.webcam_radio.pack()
        self.esp_radio = ctk.CTkRadioButton(self, text="ESP32-CAM", variable=self.source_var, value="ESP32-CAM")
        self.esp_radio.pack()

        self.ip_label = ctk.CTkLabel(self, text="ESP32-CAM IP:")
        self.ip_label.pack(pady=(10,0))
        self.ip_entry = ctk.CTkEntry(self)
        self.ip_entry.pack()
        self.ip_entry.insert(0, "192.168.1.100") # Default IP, change as needed

        self.start_btn = ctk.CTkButton(self, text="Start Capture", command=self.start_capture)
        self.start_btn.pack(pady=10)

        self.pause_btn = ctk.CTkButton(self, text="Pause Capture", command=self.pause_capture, state='disabled')
        self.pause_btn.pack()

        self.stop_btn = ctk.CTkButton(self, text="Stop & Save", command=self.stop_capture, state='disabled')
        self.stop_btn.pack(pady=(10,0))

        # Canvas for video stream
        self.canvas = ctk.CTkCanvas(self, width=480, height=360, bg="black")
        self.canvas.pack(pady=10)

        self.capturing = False
        self.paused = False
        self.thread = None
        self.out = None
        self.cap = None
        self.frame_image = None

    def start_capture(self):
        if not self.capturing:
            self.capturing = True
            self.paused = False
            self.start_btn.configure(state='disabled')
            self.pause_btn.configure(state='normal')
            self.stop_btn.configure(state='normal')
            self.thread = threading.Thread(target=self.capture_video)
            self.thread.start()

    def pause_capture(self):
        self.paused = not self.paused
        self.pause_btn.configure(text="Resume Capture" if self.paused else "Pause Capture")

    def stop_capture(self):
        self.capturing = False
        # Wait for thread to finish but with timeout to prevent hanging
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2.0)
        if self.cap:
            self.cap.release()
            self.cap = None
        if self.out:
            self.out.release()
            self.out = None
        self.canvas.delete("all")
        self.start_btn.configure(state='normal')
        self.pause_btn.configure(state='disabled', text="Pause Capture")
        self.stop_btn.configure(state='disabled')
        
        # Show confirmation message
        self.show_message(f"Video saved to: {VIDEO_PATH}")

    def show_message(self, msg):
        msg_win = ctk.CTkToplevel(self)
        msg_win.title("Info")
        msg_win.geometry("400x100")
        label = ctk.CTkLabel(msg_win, text=msg, font=("Arial", 12))
        label.pack(pady=20)
        btn = ctk.CTkButton(msg_win, text="OK", command=msg_win.destroy)
        btn.pack()

    def get_video_source(self):
        if self.source_var.get() == "Webcam":
            return 0
        else:
            ip = self.ip_entry.get().strip()
            return f"http://{ip}/stream"

    def capture_video(self):
        import PIL.Image, PIL.ImageTk
        source = self.get_video_source()
        if source == 0:
            self.cap = cv2.VideoCapture(0)
        else:
            self.cap = cv2.VideoCapture(source)

        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        fps = 20.0
        width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if width == 0 or height == 0:
            self.show_error("Failed to get video frame size. Is the camera/stream available?")
            if self.cap:
                self.cap.release()
            return
        self.out = cv2.VideoWriter(VIDEO_PATH, fourcc, fps, (width, height))
        if not self.out.isOpened():
            self.show_error(f"Failed to open video file for writing: {VIDEO_PATH}")
            if self.cap:
                self.cap.release()
            return

        saved_any = False
        while self.capturing:
            if not self.paused:
                ret, frame = self.cap.read()
                if ret:
                    self.out.write(frame)
                    saved_any = True
                    # Convert frame to RGB and display in canvas
                    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    img = PIL.Image.fromarray(rgb)
                    img = img.resize((480, 360))
                    self.frame_image = PIL.ImageTk.PhotoImage(img)
                    self.canvas.create_image(0, 0, anchor='nw', image=self.frame_image)
                else:
                    break
            else:
                # Sleep when paused to prevent high CPU usage
                threading.Event().wait(0.1)
            
            # Use after() instead of update() to prevent blocking
            try:
                self.after_idle(lambda: None)
            except:
                break
                
        if not saved_any:
            self.show_error("No frames were saved. Check your camera or stream link.")

    def show_error(self, msg):
        err_win = ctk.CTkToplevel(self)
        err_win.title("Error")
        err_win.geometry("350x100")
        label = ctk.CTkLabel(err_win, text=msg, text_color="red", font=("Arial", 14))
        label.pack(pady=20)
        btn = ctk.CTkButton(err_win, text="OK", command=err_win.destroy)
        btn.pack()

if __name__ == "__main__":
    app = VideoCaptureApp()
    app.mainloop()
