#!/usr/bin/env bash

set -Eeuo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PROJECT_ROOT

python3 - <<'PY'
import atexit
import os
import queue
import signal
import subprocess
import sys
import threading
import time

PROJECT_ROOT = os.environ["PROJECT_ROOT"]

COLORS = {
    "bg": "#141619",
    "panel": "#1d2024",
    "panel2": "#25292f",
    "panel3": "#303640",
    "text": "#f4f7fb",
    "muted": "#a7b4c3",
    "muted2": "#c2ccd6",
    "accent": "#2f9bff",
    "accent_dark": "#0878d8",
    "danger": "#e5484d",
    "border": "#3a4656",
    "field": "#111419",
}

FONT_FAMILY = "DejaVu Sans"
MONO_FONT_FAMILY = "DejaVu Sans Mono"
BASE_FONT_SPECS = {
    "kicker": (FONT_FAMILY, 9, "bold"),
    "header": (FONT_FAMILY, 20, "bold"),
    "sub": (FONT_FAMILY, 10, "normal"),
    "section": (FONT_FAMILY, 11, "bold"),
    "card_title": (FONT_FAMILY, 11, "bold"),
    "body": (FONT_FAMILY, 10, "normal"),
    "body_bold": (FONT_FAMILY, 10, "bold"),
    "small": (FONT_FAMILY, 9, "normal"),
    "small_bold": (FONT_FAMILY, 9, "bold"),
    "status": (FONT_FAMILY, 10, "bold"),
    "button": (FONT_FAMILY, 10, "bold"),
    "entry": (FONT_FAMILY, 14, "normal"),
    "identity": (FONT_FAMILY, 12, "bold"),
    "dialog_title": (FONT_FAMILY, 14, "bold"),
    "console": (MONO_FONT_FAMILY, 11, "normal"),
}
FONTS = {}


def fallback_exec():
    os.chdir(PROJECT_ROOT)
    os.execv("./run_play_ctrl_ros2.sh", ["./run_play_ctrl_ros2.sh", "--headless"])


try:
    import tkinter as tk
    from tkinter import filedialog, messagebox
    from tkinter import font as tkfont
    from tkinter import ttk
except Exception as exc:
    print(f"[launch.sh] tkinter is not available ({exc}); falling back to direct launch.")
    fallback_exec()


def default_onnx_path():
    return os.path.join(PROJECT_ROOT, "weights", "example_policy.onnx")


class FlamingoLauncher:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Flamingo ROS 2 Control Center")
        self.root.geometry("1220x780+140+80")
        self.root.minsize(1080, 700)

        self.process = None
        self.reader_thread = None
        self.log_queue = queue.Queue()
        self.started_at = None
        self.status_var = tk.StringVar(value="Idle")
        self.toggle_tiles = []

        self._build_fonts()
        self._build_vars()
        self._setup_style()
        self._build_layout()
        self._bind_lifecycle()
        self._poll_log_queue()
        self._tick_status()

    def _build_fonts(self):
        FONTS.clear()
        for name, (family, size, weight) in BASE_FONT_SPECS.items():
            FONTS[name] = tkfont.Font(family=family, size=size, weight=weight)

    def _build_vars(self):
        self.bool_vars = {
            "headless": tk.BooleanVar(value=True),
            "rviz2": tk.BooleanVar(value=True),
            "front_camera": tk.BooleanVar(value=True),
            "adas_camera": tk.BooleanVar(value=True),
            "imu": tk.BooleanVar(value=True),
            "lidar": tk.BooleanVar(value=True),
            "lidar_imu": tk.BooleanVar(value=True),
            "height_map": tk.BooleanVar(value=False),
            # Keep the material cleanup enabled by default without exposing it as a mode toggle.
            "sanitize_materials": tk.BooleanVar(value=True),
        }
        self.text_vars = {
            "camera_rate": tk.StringVar(value="30"),
            "camera_width": tk.StringVar(value="320"),
            "camera_height": tk.StringVar(value="240"),
            "imu_rate": tk.StringVar(value="100"),
            "lidar_rate": tk.StringVar(value="5"),
            "perf_report_interval": tk.StringVar(value="2"),
            "path_gt_topic": tk.StringVar(value="/path_gt"),
            "policy_onnx_path": tk.StringVar(value=default_onnx_path()),
            "hw_shoulder_kp": tk.StringVar(value="35.0"),
            "hw_shoulder_kd": tk.StringVar(value="0.45"),
            "hw_wheel_kp": tk.StringVar(value="0.0"),
            "hw_wheel_kd": tk.StringVar(value="0.3"),
            "action_shoulder_scale": tk.StringVar(value="0.25"),
            "action_wheel_scale": tk.StringVar(value="40.0"),
            "obs_joint_pos_scale": tk.StringVar(value="1.0"),
            "obs_joint_vel_scale": tk.StringVar(value="0.15"),
            "obs_base_ang_vel_scale": tk.StringVar(value="0.25"),
            "obs_projected_gravity_scale": tk.StringVar(value="1.0"),
            "obs_cmd_vel_x_scale": tk.StringVar(value="2.0"),
            "obs_cmd_vel_y_scale": tk.StringVar(value="0.0"),
            "obs_cmd_vel_yaw_scale": tk.StringVar(value="0.25"),
        }

    def _setup_style(self):
        self.root.configure(bg=COLORS["bg"])
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass

        colors = COLORS

        style.configure(".", background=colors["bg"], foreground=colors["text"], fieldbackground=colors["field"], font=FONTS["body"])
        style.configure("Root.TFrame", background=colors["bg"])
        style.configure("Panel.TFrame", background=colors["panel"], relief="flat")
        style.configure("Panel2.TFrame", background=colors["panel2"], relief="flat")
        style.configure("Panel3.TFrame", background=colors["panel3"], relief="flat")
        style.configure("Header.TLabel", background=colors["bg"], foreground=colors["text"], font=FONTS["header"])
        style.configure("Kicker.TLabel", background=colors["bg"], foreground=colors["accent"], font=FONTS["kicker"])
        style.configure("Sub.TLabel", background=colors["bg"], foreground=colors["muted"], font=FONTS["sub"])
        style.configure("Section.TLabel", background=colors["panel"], foreground=colors["text"], font=FONTS["section"])
        style.configure("CardTitle.TLabel", background=colors["panel2"], foreground=colors["text"], font=FONTS["card_title"])
        style.configure("CardText.TLabel", background=colors["panel2"], foreground=colors["muted"], font=FONTS["body"])
        style.configure("Muted.TLabel", background=colors["panel"], foreground=colors["muted"], font=FONTS["body"])
        style.configure("Muted2.TLabel", background=colors["panel2"], foreground=colors["muted2"], font=FONTS["body"])
        style.configure("Status.TLabel", background=colors["panel2"], foreground=colors["text"], font=FONTS["status"])
        style.configure(
            "TEntry",
            fieldbackground=colors["field"],
            foreground=colors["text"],
            bordercolor=colors["border"],
            lightcolor=colors["border"],
            darkcolor=colors["border"],
            padding=(8, 5),
            font=FONTS["entry"],
        )
        style.map("TEntry", bordercolor=[("focus", colors["accent"])])
        style.configure("Accent.TButton", background=colors["accent_dark"], foreground="white", font=FONTS["button"], padding=(14, 7))
        style.map("Accent.TButton", background=[("active", colors["accent"]), ("disabled", "#243447")])
        style.configure("Danger.TButton", background=colors["danger"], foreground="white", font=FONTS["button"], padding=(14, 7))
        style.map("Danger.TButton", background=[("active", "#da3633"), ("disabled", "#3a2a2a")])
        style.configure("Secondary.TButton", background=colors["accent_dark"], foreground="white", font=FONTS["button"], padding=(12, 6))
        style.map("Secondary.TButton", background=[("active", colors["accent"])])

    def _build_layout(self):
        root = ttk.Frame(self.root, style="Root.TFrame", padding=18)
        root.pack(fill="both", expand=True)
        root.columnconfigure(0, weight=1, minsize=470)
        root.columnconfigure(1, weight=2, minsize=620)
        root.rowconfigure(2, weight=1)

        header = ttk.Frame(root, style="Root.TFrame")
        header.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 12))
        header.columnconfigure(0, weight=1)
        ttk.Label(header, text="ROBOT OPERATIONS PANEL", style="Kicker.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Label(header, text="Flamingo ROS 2 Control Center", style="Header.TLabel").grid(row=1, column=0, sticky="w", pady=(2, 0))
        ttk.Label(
            header,
            text="Simulation launch, ROS telemetry, sensor loadout, and runtime diagnostics.",
            style="Sub.TLabel",
        ).grid(row=2, column=0, sticky="w", pady=(4, 0))

        identity = tk.Frame(header, bg=COLORS["panel"], highlightbackground=COLORS["border"], highlightthickness=1)
        identity.grid(row=0, column=1, rowspan=3, sticky="e", padx=(16, 0))
        tk.Label(identity, text="SYSTEM", bg=COLORS["panel"], fg=COLORS["muted"], font=FONTS["small_bold"]).grid(row=0, column=0, sticky="w", padx=14, pady=(9, 0))
        tk.Label(identity, text="F4 / ISAAC-ROS", bg=COLORS["panel"], fg=COLORS["text"], font=FONTS["identity"]).grid(row=1, column=0, sticky="w", padx=14, pady=(1, 9))

        topbar = tk.Frame(root, bg="#2a2d31", highlightbackground="#3d4148", highlightthickness=1)
        topbar.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(0, 12))
        topbar.columnconfigure(0, weight=1)
        tk.Label(topbar, textvariable=self.status_var, bg="#2a2d31", fg=COLORS["muted2"], font=FONTS["status"]).grid(
            row=0, column=0, sticky="w", padx=14, pady=9
        )
        control_cluster = tk.Frame(topbar, bg="#2a2d31")
        control_cluster.grid(row=0, column=1, sticky="e", padx=10)
        self.start_button = ttk.Button(control_cluster, text="RUN", style="Accent.TButton", command=self.start)
        self.start_button.pack(side="left", padx=(0, 8), ipadx=18)
        self.stop_button = ttk.Button(control_cluster, text="STOP", style="Danger.TButton", command=self.stop, state="disabled")
        self.stop_button.pack(side="left", ipadx=16)

        config_shell = ttk.Frame(root, style="Panel.TFrame", padding=0)
        config_shell.grid(row=2, column=0, sticky="nsew", padx=(0, 12))
        config_shell.rowconfigure(0, weight=1)
        config_shell.columnconfigure(0, weight=1)
        config_canvas = tk.Canvas(config_shell, bg=COLORS["panel"], highlightthickness=0, borderwidth=0)
        config_scroll = ttk.Scrollbar(config_shell, orient="vertical", command=config_canvas.yview)
        config_canvas.configure(yscrollcommand=config_scroll.set)
        config_canvas.grid(row=0, column=0, sticky="nsew")
        config_scroll.grid(row=0, column=1, sticky="ns")
        config = ttk.Frame(config_canvas, style="Panel.TFrame", padding=14)
        config_window = config_canvas.create_window((0, 0), window=config, anchor="nw")
        config.columnconfigure(0, weight=1)

        def _sync_config_scroll(_event=None):
            config_canvas.configure(scrollregion=config_canvas.bbox("all"))
            config_canvas.itemconfigure(config_window, width=config_canvas.winfo_width())

        config.bind("<Configure>", _sync_config_scroll)
        config_canvas.bind("<Configure>", _sync_config_scroll)
        config_canvas.bind_all("<MouseWheel>", lambda event: config_canvas.yview_scroll(int(-1 * (event.delta / 120)), "units"))

        log_panel = ttk.Frame(root, style="Panel.TFrame", padding=14)
        log_panel.grid(row=2, column=1, sticky="nsew")
        log_panel.rowconfigure(1, weight=1)
        log_panel.columnconfigure(0, weight=1)

        mode_outer, mode_card = self._card(config, "Operation Mode", "Runtime envelope and visualization surfaces")
        mode_outer.pack(fill="x", pady=(0, 12))
        mode_card.columnconfigure(0, weight=1)
        mode_card.columnconfigure(1, weight=1)
        self._toggle_tile(mode_card, "HEADLESS", "Isaac viewport off", "headless").grid(row=0, column=0, sticky="ew", padx=(0, 6), pady=4)
        self._toggle_tile(mode_card, "RVIZ2", "Visualization client", "rviz2").grid(row=0, column=1, sticky="ew", padx=(6, 0), pady=4)

        sensor_outer, sensor_card = self._card(config, "Sensor Loadout", "Enable only the telemetry required for this run")
        sensor_outer.pack(fill="x", pady=(0, 12))
        sensor_card.columnconfigure(0, weight=1)
        sensor_card.columnconfigure(1, weight=1)
        for idx, (label, subtitle, key) in enumerate(
            (
                ("FRONT RGBD", "F_camera_link", "front_camera"),
                ("ADAS RGBD", "A_camera_link", "adas_camera"),
                ("BASE IMU", "base_link", "imu"),
                ("LIDAR CLOUD", "PointCloud2", "lidar"),
                ("LIDAR IMU", "lidar_link", "lidar_imu"),
                ("HEIGHT MAP", "Ray grid", "height_map"),
            )
        ):
            padx = (0, 6) if idx % 2 == 0 else (6, 0)
            self._toggle_tile(sensor_card, label, subtitle, key).grid(
                row=idx // 2, column=idx % 2, sticky="ew", padx=padx, pady=5
            )

        params_outer, params_card = self._card(config, "Telemetry Parameters", "Rates and image dimensions applied before scene creation")
        params_outer.pack(fill="x", pady=(0, 12))
        for idx, (label, key, unit) in enumerate(
            (
                ("Camera rate", "camera_rate", "Hz"),
                ("Image width", "camera_width", "px"),
                ("Image height", "camera_height", "px"),
                ("IMU rate", "imu_rate", "Hz"),
                ("Lidar rate", "lidar_rate", "Hz"),
                ("Perf report", "perf_report_interval", "sec"),
                ("Path topic", "path_gt_topic", ""),
            )
        ):
            self._entry_row(params_card, label, key, unit).grid(row=idx, column=0, sticky="ew", pady=3)
        self._file_row(params_card, "Policy ONNX", "policy_onnx_path").grid(row=7, column=0, sticky="ew", pady=(8, 3))
        params_card.columnconfigure(0, weight=1)

        runtime_outer, runtime_card = self._card(config, "Policy Runtime", "Hardware gains and policy scaling profiles")
        runtime_outer.pack(fill="x", pady=(0, 12))
        runtime_card.columnconfigure(0, weight=1)
        for idx, (title, subtitle, fields) in enumerate(
            (
                (
                    "Hardware Settings",
                    "Shoulder and wheel Kp/Kd",
                    (
                        ("Shoulder Kp", "hw_shoulder_kp", ""),
                        ("Shoulder Kd", "hw_shoulder_kd", ""),
                        ("Wheel Kp", "hw_wheel_kp", ""),
                        ("Wheel Kd", "hw_wheel_kd", ""),
                    ),
                ),
                (
                    "Observation Settings",
                    "Actual policy observations in velocity env",
                    (
                        ("Joint pos scale", "obs_joint_pos_scale", ""),
                        ("Joint vel scale", "obs_joint_vel_scale", ""),
                        ("Base ang scale", "obs_base_ang_vel_scale", ""),
                        ("Projected gravity", "obs_projected_gravity_scale", ""),
                        ("Cmd vel X scale", "obs_cmd_vel_x_scale", ""),
                        ("Cmd vel Y scale", "obs_cmd_vel_y_scale", ""),
                        ("Cmd vel Yaw scale", "obs_cmd_vel_yaw_scale", ""),
                    ),
                ),
                (
                    "Action Settings",
                    "Shoulder position and wheel velocity scale",
                    (
                        ("Shoulder action", "action_shoulder_scale", ""),
                        ("Wheel action", "action_wheel_scale", ""),
                    ),
                ),
            )
        ):
            self._settings_button(runtime_card, title, subtitle, fields).grid(row=idx, column=0, sticky="ew", pady=4)

        command_outer, command_card = self._card(config, "Operator Input", "Global keyboard capture is used when launched from this panel")
        command_outer.pack(fill="x")
        ttk.Label(command_card, text="Drive: W/S or Up/Down    Turn: A/D or Left/Right    Stop: Space", style="Muted2.TLabel").pack(anchor="w", pady=(0, 6))
        ttk.Label(command_card, text="Closing the launcher or parent terminal stops the process group.", style="Muted2.TLabel").pack(anchor="w")

        log_head = ttk.Frame(log_panel, style="Panel.TFrame")
        log_head.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        ttk.Label(log_head, text="Runtime Console", style="Section.TLabel").pack(side="left")
        ttk.Label(log_head, text="stdout / stderr", style="Muted.TLabel").pack(side="left", padx=(10, 0))
        ttk.Button(log_head, text="Clear", style="Secondary.TButton", command=self.clear_log).pack(side="right")

        console_frame = ttk.Frame(log_panel, style="Panel.TFrame")
        console_frame.grid(row=1, column=0, sticky="nsew")
        console_frame.rowconfigure(0, weight=1)
        console_frame.columnconfigure(0, weight=1)
        self.console = tk.Text(
            console_frame,
            bg="#03070b",
            fg="#d8e4ef",
            insertbackground="#d6e2f0",
            selectbackground="#1f6feb",
            relief="flat",
            wrap="word",
            font=FONTS["console"],
            padx=16,
            pady=14,
        )
        self.console.grid(row=0, column=0, sticky="nsew")
        scroll = ttk.Scrollbar(console_frame, orient="vertical", command=self.console.yview)
        scroll.grid(row=0, column=1, sticky="ns")
        self.console.configure(yscrollcommand=scroll.set)

        footer = ttk.Frame(root, style="Root.TFrame")
        footer.grid(row=3, column=0, columnspan=2, sticky="ew", pady=(10, 0))
        ttk.Label(footer, text="Process supervision: enabled   |   TF root: world -> f4/base_link   |   Path: /path_gt", style="Sub.TLabel").pack(side="left")

    def _card(self, parent, title, subtitle):
        outer = tk.Frame(parent, bg=COLORS["panel2"], highlightbackground=COLORS["border"], highlightthickness=1)
        head = tk.Frame(outer, bg=COLORS["panel2"])
        head.pack(fill="x", padx=14, pady=(12, 7))
        tk.Label(head, text=title, bg=COLORS["panel2"], fg=COLORS["text"], font=FONTS["card_title"]).pack(anchor="w")
        tk.Label(head, text=subtitle, bg=COLORS["panel2"], fg=COLORS["muted2"], font=FONTS["body"]).pack(anchor="w", pady=(2, 0))
        body = ttk.Frame(outer, style="Panel2.TFrame", padding=(14, 0, 14, 14))
        body.pack(fill="both", expand=True)
        return outer, body

    def _toggle_tile(self, parent, title, subtitle, key):
        tile = tk.Frame(parent, bd=0, highlightthickness=1, cursor="hand2")
        title_label = tk.Label(tile, text=title, font=FONTS["body_bold"], anchor="w")
        state_label = tk.Label(tile, text="", font=FONTS["small_bold"], anchor="e")
        subtitle_label = tk.Label(tile, text=subtitle, font=FONTS["small"], anchor="w")

        title_label.grid(row=0, column=0, sticky="ew", padx=(12, 6), pady=(10, 0))
        state_label.grid(row=0, column=1, sticky="e", padx=(6, 12), pady=(10, 0))
        subtitle_label.grid(row=1, column=0, columnspan=2, sticky="ew", padx=12, pady=(2, 10))
        tile.columnconfigure(0, weight=1)

        def toggle(_event=None):
            self.bool_vars[key].set(not self.bool_vars[key].get())
            self._refresh_toggle_tiles()

        for widget in (tile, title_label, state_label, subtitle_label):
            widget.bind("<Button-1>", toggle)

        self.toggle_tiles.append((key, tile, title_label, subtitle_label, state_label))
        self._paint_toggle_tile(key, tile, title_label, subtitle_label, state_label)
        return tile

    def _paint_toggle_tile(self, key, tile, title_label, subtitle_label, state_label):
        active = bool(self.bool_vars[key].get())
        bg = "#0f2b43" if active else "#0f161e"
        fg = "#ffffff" if active else COLORS["text"]
        sub = "#b9dcff" if active else COLORS["muted"]
        border = COLORS["accent"] if active else COLORS["border"]
        state_text = "ON" if active else "OFF"
        state_fg = "#9bd0ff" if active else "#8995a3"
        for widget in (tile, title_label, subtitle_label, state_label):
            widget.configure(bg=bg)
        tile.configure(highlightbackground=border, highlightcolor=border)
        title_label.configure(fg=fg)
        subtitle_label.configure(fg=sub)
        state_label.configure(text=state_text, fg=state_fg)

    def _refresh_toggle_tiles(self):
        for key, tile, title_label, subtitle_label, state_label in self.toggle_tiles:
            self._paint_toggle_tile(key, tile, title_label, subtitle_label, state_label)

    def _settings_button(self, parent, title, subtitle, fields):
        row = tk.Frame(parent, bg="#0f161e", highlightbackground=COLORS["border"], highlightthickness=1)
        row.columnconfigure(0, weight=1)
        tk.Label(row, text=title, bg="#0f161e", fg=COLORS["text"], font=FONTS["body_bold"]).grid(
            row=0, column=0, sticky="w", padx=(14, 8), pady=(11, 0)
        )
        tk.Label(row, text=subtitle, bg="#0f161e", fg=COLORS["muted2"], font=FONTS["body"]).grid(
            row=1, column=0, sticky="w", padx=(14, 8), pady=(2, 11)
        )
        ttk.Button(
            row,
            text="Configure",
            style="Secondary.TButton",
            command=lambda: self._open_settings_dialog(title, fields),
        ).grid(row=0, column=1, rowspan=2, sticky="e", padx=12, pady=10)
        return row

    def _open_settings_dialog(self, title, fields):
        dialog = tk.Toplevel(self.root)
        dialog.title(title)
        dialog.configure(bg=COLORS["bg"])
        dialog.transient(self.root)
        dialog.grab_set()

        shell = tk.Frame(dialog, bg=COLORS["panel"], highlightbackground=COLORS["border"], highlightthickness=1)
        shell.pack(fill="both", expand=True, padx=18, pady=18)
        tk.Label(shell, text=title, bg=COLORS["panel"], fg=COLORS["text"], font=FONTS["dialog_title"]).pack(
            anchor="w", padx=16, pady=(14, 2)
        )
        tk.Label(
            shell,
            text="Values are applied the next time RUN starts the simulation.",
            bg=COLORS["panel"],
            fg=COLORS["muted"],
            font=FONTS["body"],
        ).pack(anchor="w", padx=16, pady=(0, 12))

        body = ttk.Frame(shell, style="Panel2.TFrame", padding=(14, 12, 14, 12))
        body.pack(fill="x", padx=16)
        body.columnconfigure(0, weight=1)
        for idx, (label, key, unit) in enumerate(fields):
            self._entry_row(body, label, key, unit).grid(row=idx, column=0, sticky="ew", pady=4)

        footer = ttk.Frame(shell, style="Panel.TFrame", padding=(16, 14, 16, 16))
        footer.pack(fill="x")
        ttk.Button(footer, text="Close", style="Accent.TButton", command=dialog.destroy).pack(side="right", ipadx=16)

        dialog.bind("<Escape>", lambda _event: dialog.destroy())
        dialog.update_idletasks()
        width = max(440, dialog.winfo_reqwidth())
        height = dialog.winfo_reqheight()
        x = self.root.winfo_rootx() + max(24, (self.root.winfo_width() - width) // 2)
        y = self.root.winfo_rooty() + 90
        dialog.geometry(f"{width}x{height}+{x}+{y}")

    def _entry_row(self, parent, label, key, unit):
        row = ttk.Frame(parent, style="Panel2.TFrame")
        row.columnconfigure(1, weight=1)
        ttk.Label(row, text=label, style="Muted2.TLabel", width=16).grid(row=0, column=0, sticky="w")
        ttk.Entry(row, textvariable=self.text_vars[key], width=14).grid(row=0, column=1, sticky="ew", padx=10)
        ttk.Label(row, text=unit, style="Muted2.TLabel", width=4).grid(row=0, column=2, sticky="e")
        return row

    def _file_row(self, parent, label, key):
        row = ttk.Frame(parent, style="Panel2.TFrame")
        row.columnconfigure(1, weight=1)
        ttk.Label(row, text=label, style="Muted2.TLabel", width=16).grid(row=0, column=0, sticky="w")
        ttk.Entry(row, textvariable=self.text_vars[key]).grid(row=0, column=1, sticky="ew", padx=10)
        ttk.Button(row, text="Browse", style="Secondary.TButton", command=lambda: self._browse_onnx(key)).grid(
            row=0, column=2, sticky="e"
        )
        return row

    def _browse_onnx(self, key):
        initial = self.text_vars[key].get().strip() or PROJECT_ROOT
        initial_dir = initial if os.path.isdir(initial) else os.path.dirname(initial) or PROJECT_ROOT
        selected = filedialog.askopenfilename(
            title="Select ONNX policy",
            initialdir=initial_dir,
            filetypes=(("ONNX policy", "*.onnx"), ("All files", "*")),
        )
        if selected:
            self.text_vars[key].set(selected)

    def _bind_lifecycle(self):
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        atexit.register(self._terminate_process)
        for sig in (signal.SIGHUP, signal.SIGINT, signal.SIGTERM):
            try:
                signal.signal(sig, self._signal_handler)
            except Exception:
                pass

    def _signal_handler(self, signum, _frame):
        self._log(f"[launcher] received signal {signum}; stopping simulation\n")
        self._terminate_process()
        try:
            self.root.destroy()
        except Exception:
            pass
        sys.exit(128 + int(signum))

    def _config(self):
        def f(key, fallback):
            try:
                return float(self.text_vars[key].get())
            except Exception:
                return fallback

        def i(key, fallback):
            try:
                return max(int(self.text_vars[key].get()), 1)
            except Exception:
                return fallback

        return {
            "ENABLE_ROS2_FRONT_CAMERA": self._b("front_camera"),
            "ENABLE_ROS2_ADAS_CAMERA": self._b("adas_camera"),
            "ENABLE_ROS2_IMU": self._b("imu"),
            "ENABLE_ROS2_LIDAR": self._b("lidar"),
            "ENABLE_ROS2_LIDAR_IMU": self._b("lidar_imu"),
            "ENABLE_ROS2_HEIGHT_MAP": self._b("height_map"),
            "RUN_RVIZ2": self._b("rviz2"),
            "TELEOP_USE_STDIN": "0",
            "SANITIZE_SCENE_MATERIALS": self._b("sanitize_materials"),
            "ROS2_CAMERA_RATE": str(max(f("camera_rate", 30.0), 0.1)),
            "ROS2_CAMERA_WIDTH": str(i("camera_width", 320)),
            "ROS2_CAMERA_HEIGHT": str(i("camera_height", 240)),
            "ROS2_IMU_RATE": str(max(f("imu_rate", 100.0), 0.1)),
            "ROS2_LIDAR_RATE": str(max(f("lidar_rate", 5.0), 0.1)),
            "ROBOT_PATH_GT_TOPIC": self.text_vars["path_gt_topic"].get().strip() or "/path_gt",
            "POLICY_ONNX_PATH": self.text_vars["policy_onnx_path"].get().strip(),
            "HW_SHOULDER_KP": str(f("hw_shoulder_kp", 35.0)),
            "HW_SHOULDER_KD": str(f("hw_shoulder_kd", 0.45)),
            "HW_WHEEL_KP": str(f("hw_wheel_kp", 0.0)),
            "HW_WHEEL_KD": str(f("hw_wheel_kd", 0.3)),
            "ACTION_SHOULDER_SCALE": str(f("action_shoulder_scale", 0.25)),
            "ACTION_WHEEL_SCALE": str(f("action_wheel_scale", 40.0)),
            "OBS_JOINT_POS_SCALE": str(f("obs_joint_pos_scale", 1.0)),
            "OBS_JOINT_VEL_SCALE": str(f("obs_joint_vel_scale", 0.15)),
            "OBS_BASE_ANG_VEL_SCALE": str(f("obs_base_ang_vel_scale", 0.25)),
            "OBS_PROJECTED_GRAVITY_SCALE": str(f("obs_projected_gravity_scale", 1.0)),
            "OBS_CMD_VEL_X_SCALE": str(f("obs_cmd_vel_x_scale", 2.0)),
            "OBS_CMD_VEL_Y_SCALE": str(f("obs_cmd_vel_y_scale", 0.0)),
            "OBS_CMD_VEL_YAW_SCALE": str(f("obs_cmd_vel_yaw_scale", 0.25)),
            "_PERF_REPORT_INTERVAL": str(max(f("perf_report_interval", 2.0), 0.0)),
        }

    def _b(self, key):
        return "1" if bool(self.bool_vars[key].get()) else "0"

    def start(self):
        if self.process and self.process.poll() is None:
            messagebox.showinfo("Already running", "Simulation is already running.")
            return

        cfg = self._config()
        env = os.environ.copy()
        perf_interval = cfg.pop("_PERF_REPORT_INTERVAL")
        env.update(cfg)

        args = ["./run_play_ctrl_ros2.sh"]
        if self.bool_vars["headless"].get():
            args.append("--headless")
        if float(perf_interval) > 0:
            args.extend(["--perf_report_interval", perf_interval])

        self.clear_log()
        self._log("[launcher] starting simulation\n")
        self._log("[launcher] command: " + " ".join(args) + "\n")
        for key in sorted(cfg):
            self._log(f"[launcher] {key}={cfg[key]}\n")

        try:
            self.process = subprocess.Popen(
                args,
                cwd=PROJECT_ROOT,
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL,
                text=True,
                bufsize=1,
                preexec_fn=os.setsid,
            )
        except Exception as exc:
            self._log(f"[launcher] failed to start: {exc}\n")
            messagebox.showerror("Launch failed", str(exc))
            return

        self.started_at = time.time()
        self.start_button.configure(state="disabled")
        self.stop_button.configure(state="normal")
        self.status_var.set(f"Running PID {self.process.pid}")
        self.reader_thread = threading.Thread(target=self._read_process_output, daemon=True)
        self.reader_thread.start()

    def stop(self):
        self._log("[launcher] stopping simulation\n")
        self._terminate_process()
        self.start_button.configure(state="normal")
        self.stop_button.configure(state="disabled")
        self.status_var.set("Stopped")

    def close(self):
        if self.process and self.process.poll() is None:
            if not messagebox.askyesno("Stop simulation?", "Stop the running simulation and close the launcher?"):
                return
        self._terminate_process()
        self.root.destroy()

    def _terminate_process(self):
        proc = self.process
        if proc is None or proc.poll() is not None:
            return
        try:
            os.killpg(proc.pid, signal.SIGINT)
            proc.wait(timeout=8)
        except Exception:
            try:
                os.killpg(proc.pid, signal.SIGTERM)
                proc.wait(timeout=5)
            except Exception:
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except Exception:
                    pass

    def _read_process_output(self):
        assert self.process is not None
        for line in self.process.stdout:
            self.log_queue.put(line)
        code = self.process.wait()
        self.log_queue.put(f"\n[launcher] process exited with status {code}\n")
        self.log_queue.put(("__EXIT__", code))

    def _poll_log_queue(self):
        try:
            while True:
                item = self.log_queue.get_nowait()
                if isinstance(item, tuple) and item[0] == "__EXIT__":
                    self.start_button.configure(state="normal")
                    self.stop_button.configure(state="disabled")
                    self.status_var.set(f"Exited with status {item[1]}")
                else:
                    self._log(item)
        except queue.Empty:
            pass
        self.root.after(80, self._poll_log_queue)

    def _tick_status(self):
        if self.process and self.process.poll() is None and self.started_at is not None:
            elapsed = int(time.time() - self.started_at)
            self.status_var.set(f"Running PID {self.process.pid} | elapsed {elapsed // 60:02d}:{elapsed % 60:02d}")
        self.root.after(1000, self._tick_status)

    def _log(self, text):
        self.console.insert("end", text)
        self.console.see("end")

    def clear_log(self):
        self.console.delete("1.0", "end")

    def run(self):
        self.root.mainloop()


try:
    app = FlamingoLauncher()
except Exception as exc:
    print(f"[launch.sh] Could not open launcher GUI ({exc}); falling back to direct launch.")
    fallback_exec()

app.run()
PY
