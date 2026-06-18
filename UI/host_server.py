#!/usr/bin/env python3
"""
上位机 ── 接收 RK3588 yolo 发来的异常帧并实时显示。
协议: [4字节json长度][JSON][4字节jpg1长度][jpg1][4字节jpg2长度][jpg2]
UI 极致美化版 · 智能儿童安全监护系统
"""

import socket
import struct
import json
import threading
import time
import os
import tkinter as tk
from datetime import datetime
from tkinter import ttk, messagebox
from io import BytesIO
from PIL import Image, ImageTk

# ===================== 高级美化配色方案 =====================
BG_DARK   = "#121212"       # 主背景
BG_PANEL  = "#1A1A2A"       # 面板背景
BG_CARD   = "#2B2B3D"       # 卡片背景
BG_ALERT  = "#D32F2F"       # 告警红色
BG_NORMAL = "#2E7D32"       # 正常绿色

FG_WHITE  = "#F5F5F5"       # 主文字
FG_SECOND = "#A0A0A0"       # 次要文字
FG_GREEN  = "#4ADE80"       # 成功色
FG_RED    = "#F87171"       # 告警色
FG_ORANGE = "#FDBA74"       # 提醒色
FG_BLUE   = "#38BDF8"       # 主色调
FG_PURPLE = "#A78BFA"       # 装饰色

BORDER     = "#3A3A4D"      # 边框色
ACCENT     = "#60A5FA"      # 强调色

# 全局参数
HOST = "0.0.0.0"
PORT = 9527
MAX_HISTORY = 200
ACCOUNTS_FILE = "accounts.json"

def _load_accounts():
    if os.path.exists(ACCOUNTS_FILE):
        with open(ACCOUNTS_FILE, "r") as f:
            return json.load(f)
    return {"admin": "123456"}

def _save_accounts(acc):
    with open(ACCOUNTS_FILE, "w") as f:
        json.dump(acc, f, indent=2)


class HostServer:
    def __init__(self, root):
        self.root = root
        self.root.title("🔴 智能儿童安全监护系统 V1.0")
        self.root.geometry("1400x800")
        self.root.minsize(1200, 700)
        self.root.configure(bg=BG_DARK)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        # 状态变量
        self.running = True
        self.history = []
        self.breath_state = True
        self._frame_count = 0
        self._startup_time = time.time()

        # 登录
        if not self._show_login():
            self.root.destroy()
            return
        self.root.geometry("1400x800")
        self.root.minsize(1200, 700)
        self.root.resizable(True, True)

        # 初始化
        self._init_style()
        self._build_ui()
        self._start_server()
        self._start_breath_effect()

    def _show_login(self):
        """登录 / 注册"""
        result = [False]
        accounts = _load_accounts()

        def do_login(event=None):
            u = user_var.get().strip()
            p = pass_var.get()
            if not u or not p:
                msg.config(text="请输入用户名和密码", fg=FG_ORANGE); return
            if u in accounts and accounts[u] == p:
                result[0] = True
                login_frame.destroy()
            else:
                msg.config(text="用户名或密码错误", fg=FG_RED)

        def do_register():
            u = user_var.get().strip()
            p = pass_var.get()
            p2 = confirm_var.get()
            if not u or not p:      msg.config(text="用户名和密码不能为空", fg=FG_ORANGE); return
            if len(u) < 3:           msg.config(text="用户名至少 3 位", fg=FG_ORANGE); return
            if len(p) < 4:           msg.config(text="密码至少 4 位", fg=FG_ORANGE); return
            if p != p2:              msg.config(text="两次密码不一致", fg=FG_ORANGE); return
            if u in accounts:        msg.config(text="用户名已存在", fg=FG_ORANGE); return
            accounts[u] = p
            _save_accounts(accounts)
            msg.config(text=f"账号 {u} 创建成功，请登录", fg=FG_GREEN)
            self.root.after(800, show_login)

        def show_register():
            title_lbl.config(text="注册新账号")
            sub_lbl.config(text="创建您的账号")
            confirm_lbl.pack(anchor=tk.W, pady=(0, 2))
            confirm_entry.pack(fill=tk.X, ipady=4, pady=(0, 8))
            login_btn.config(text="注  册", command=do_register)
            switch_btn.config(text="已有账号？去登录", command=show_login)
            msg.config(text="")
            user_var.set(""); pass_var.set(""); confirm_var.set("")

        def show_login():
            title_lbl.config(text="智能安全监护系统")
            sub_lbl.config(text="请登录以继续")
            confirm_lbl.pack_forget()
            confirm_entry.pack_forget()
            login_btn.config(text="登  录", command=do_login)
            switch_btn.config(text="注册新账号", command=show_register)
            msg.config(text="")
            user_var.set(""); pass_var.set(""); confirm_var.set("")

        # ── window ──
        self.root.geometry("440x420")
        self.root.resizable(False, False)
        self.root.update_idletasks()
        rx = (self.root.winfo_screenwidth() - 440) // 2
        ry = (self.root.winfo_screenheight() - 420) // 2
        self.root.geometry(f"+{rx}+{ry}")
        self.root.configure(bg=BG_DARK)

        login_frame = tk.Frame(self.root, bg=BG_DARK)
        login_frame.pack(fill=tk.BOTH, expand=True)

        tk.Frame(login_frame, bg=FG_BLUE, height=3).pack(fill=tk.X)

        body = tk.Frame(login_frame, bg=BG_DARK)
        body.pack(fill=tk.BOTH, expand=True, padx=50, pady=40)

        title_lbl = tk.Label(body, text="智能安全监护系统",
                             font=("Microsoft YaHei UI", 18, "bold"),
                             fg=FG_WHITE, bg=BG_DARK)
        title_lbl.pack(pady=(0, 4))
        sub_lbl = tk.Label(body, text="请登录以继续",
                           font=("Microsoft YaHei UI", 10),
                           fg=FG_SECOND, bg=BG_DARK)
        sub_lbl.pack(pady=(0, 18))

        def _input(label_text, show=None):
            tk.Label(body, text=label_text, font=("Microsoft YaHei UI", 10),
                     fg=FG_SECOND, bg=BG_DARK).pack(anchor=tk.W, pady=(6, 2))
            var = tk.StringVar()
            kw = {"font": ("Microsoft YaHei UI", 12), "relief": tk.FLAT,
                  "bg": BG_PANEL, "fg": FG_WHITE, "insertbackground": FG_WHITE,
                  "bd": 8, "highlightthickness": 1,
                  "highlightbackground": BORDER, "highlightcolor": FG_BLUE}
            if show:
                kw["show"] = show
            ent = tk.Entry(body, textvariable=var, **kw)
            ent.pack(fill=tk.X, ipady=5, pady=(0, 8))
            return var, ent

        user_var, user_entry = _input("用户名")
        user_entry.focus_set()

        pass_var, pass_entry = _input("密码", show="●")
        pass_entry.bind("<Return>", do_login)

        confirm_var, confirm_entry = _input("确认密码", show="●")
        confirm_lbl = confirm_entry.master.winfo_children()[0]
        confirm_lbl.pack_forget()
        confirm_entry.pack_forget()

        msg = tk.Label(body, text="", font=("Microsoft YaHei UI", 9),
                       bg=BG_DARK, anchor=tk.CENTER)
        msg.pack(pady=(8, 8))

        login_btn = tk.Button(body, text="登  录", command=do_login,
                              font=("Microsoft YaHei UI", 12, "bold"),
                              bg=FG_BLUE, fg="#ffffff",
                              activebackground=ACCENT, activeforeground="#ffffff",
                              relief=tk.FLAT, bd=0, padx=20, pady=8,
                              cursor="hand2")
        login_btn.pack(fill=tk.X, ipady=5)

        switch_btn = tk.Button(body, text="注册新账号", command=show_register,
                               font=("Microsoft YaHei UI", 9),
                               bg=BG_DARK, fg=FG_BLUE, activebackground=BG_DARK,
                               relief=tk.FLAT, bd=0, cursor="hand2")
        switch_btn.pack(pady=(10, 0))

        exit_btn = tk.Button(body, text="退出程序", command=self.root.destroy,
                             font=("Microsoft YaHei UI", 9),
                             bg=BG_DARK, fg=FG_RED, activebackground=BG_DARK,
                             relief=tk.FLAT, bd=0, cursor="hand2")
        exit_btn.pack(pady=(6, 0))

        while not result[0] and self.root.winfo_exists():
            try:
                self.root.update()
            except tk.TclError:
                return False
        return result[0]

    def _init_style(self):
        """全局高级样式"""
        style = ttk.Style()
        style.theme_use('clam')
        style.configure(".", background=BG_DARK, foreground=FG_WHITE)

    def _build_ui(self):
        """构建极致美化UI"""
        # ========== 顶部标题栏 ==========
        top_frame = tk.Frame(self.root, bg=BG_PANEL, height=64)
        top_frame.pack(fill=tk.X, side=tk.TOP)
        top_frame.pack_propagate(False)

        title_label = tk.Label(
            top_frame, text="👶 智能儿童安全监护系统", 
            font=("Microsoft YaHei UI", 19, "bold"),
            fg=FG_WHITE, bg=BG_PANEL
        )
        title_label.place(relx=0.5, rely=0.5, anchor=tk.CENTER)


        # 连接状态灯
        self.conn_dot = tk.Canvas(top_frame, width=18, height=18, bg=BG_PANEL, highlightthickness=0)
        self.conn_dot.pack(side=tk.RIGHT, padx=(0,12), pady=22)
        self._draw_dot("gray")

        self.conn_label = tk.Label(
            top_frame, text=f"等待设备连接 · 端口 {PORT}",
            font=("Microsoft YaHei UI", 12),
            fg=FG_SECOND, bg=BG_PANEL
        )
        self.conn_label.pack(side=tk.RIGHT, padx=(0,24), pady=18)

        # 实时时钟
        self.clock_lbl = tk.Label(top_frame, text="00:00:00",
                                   font=("Consolas", 13, "bold"),
                                   fg=FG_SECOND, bg=BG_PANEL)
        self.clock_lbl.pack(side=tk.RIGHT, padx=(0, 8), pady=20)

        # ========== 数据卡片组 ==========
        cards_container = tk.Frame(self.root, bg=BG_DARK)
        cards_container.pack(fill=tk.X, padx=20, pady=16)

        self.card_ts      = self._make_card(cards_container, "📅 接收时间", "—\n—", FG_BLUE)
        self.card_cpu     = self._make_card(cards_container, "⚙️ CPU 占用", "--%", FG_BLUE)
        self.card_npu     = self._make_card(cards_container, "🚀 NPU 占用", "--%", FG_PURPLE)
        self.card_balance = self._make_card(cards_container, "⚠️ 失衡告警", "正常", FG_GREEN)
        self.card_climb   = self._make_card(cards_container, "🧗 攀爬告警", "正常", FG_GREEN)
        self.card_lean   = self._make_card(cards_container, "🪟 探头告警", "正常", FG_GREEN)
        self.card_prox    = self._make_card(cards_container, "📏 距离监测", "安全", FG_GREEN)

        # ========== 主内容 ==========
        main_content = tk.Frame(self.root, bg=BG_DARK)
        main_content.pack(fill=tk.BOTH, expand=True, padx=20, pady=8)

        # 左侧视频区
        video_panel = tk.Frame(main_content, bg=BG_DARK)
        video_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self._create_video_panel(video_panel, "🎨 目标标注画面", FG_BLUE, 0)
        self._create_video_panel(video_panel, "📊 深度感知图", FG_ORANGE, 1)

        # 右侧面板
        right_panel = tk.Frame(main_content, bg=BG_DARK, width=360)
        right_panel.pack(side=tk.RIGHT, fill=tk.Y, padx=(16,0))
        right_panel.pack_propagate(False)

        # 告警横幅
        self.alert_frame = tk.Frame(right_panel, bg=BG_CARD, bd=0)
        self.alert_frame.pack(fill=tk.X, pady=(0,12))
        self.alert_banner = tk.Label(
            self.alert_frame, text="● 系统正常运行中",
            font=("Microsoft YaHei UI", 13, "bold"), 
            fg=FG_GREEN, bg=BG_CARD,
            padx=18, pady=14, anchor=tk.W
        )
        self.alert_banner.pack(fill=tk.X)

        # 实时数据
        detail_container = tk.Frame(right_panel, bg=BG_PANEL)
        detail_container.pack(fill=tk.X, pady=(0,12))
        tk.Label(
            detail_container, text="📋 实时数据详情", 
            font=("Microsoft YaHei UI", 11, "bold"),
            fg=FG_WHITE, bg=BG_PANEL
        ).pack(anchor=tk.W, padx=16, pady=8)
        
        self.detail_text = tk.Text(
            detail_container, height=10, font=("Consolas", 10),
            bg=BG_PANEL, fg=FG_WHITE, wrap=tk.WORD,
            bd=0, padx=16, pady=10,
            insertbackground=FG_WHITE,
            selectbackground=BORDER
        )
        self.detail_text.pack(fill=tk.X, padx=10, pady=(0,10))
        self.detail_text.config(state=tk.DISABLED)

        # 历史记录
        history_container = tk.Frame(right_panel, bg=BG_PANEL)
        history_container.pack(fill=tk.BOTH, expand=True)
        hist_header = tk.Frame(history_container, bg=BG_PANEL)
        hist_header.pack(fill=tk.X)

        tk.Label(
            history_container, text="📜 警告历史记录",
            font=("Microsoft YaHei UI", 11, "bold"),
            fg=FG_WHITE, bg=BG_PANEL
        ).pack(anchor=tk.W, padx=16, pady=(8,0))

        self.hist_count_lbl = tk.Label(history_container, text="0 条",
                                        font=("Consolas", 9),
                                        fg=FG_SECOND, bg=BG_PANEL)
        self.hist_count_lbl.pack(anchor=tk.W, padx=16, pady=(0,4))
        
        list_frame = tk.Frame(history_container, bg=BG_DARK)
        list_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=(0,10))
        
        scrollbar = tk.Scrollbar(list_frame, bg=BG_DARK, troughcolor=BG_DARK, bd=0)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        self.history_list = tk.Listbox(
            list_frame, font=("Consolas", 10), 
            bg=BG_DARK, fg=FG_WHITE,
            selectbackground=BORDER, 
            selectforeground=ACCENT,
            activestyle="none", bd=0, 
            highlightthickness=0, 
            yscrollcommand=scrollbar.set
        )
        self.history_list.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.config(command=self.history_list.yview)
        self.history_list.bind("<<ListboxSelect>>", self._on_history_select)

        # ========== 底部状态栏 ==========
        bottom = tk.Frame(self.root, bg=BG_PANEL, height=26)
        bottom.pack(fill=tk.X, side=tk.BOTTOM)
        bottom.pack_propagate(False)

        self.frame_lbl = tk.Label(bottom, text="📊 帧数: 0", font=("Consolas", 9),
                                   fg=FG_SECOND, bg=BG_PANEL)
        self.frame_lbl.pack(side=tk.LEFT, padx=12)

        self.uptime_lbl = tk.Label(bottom, text="⏳ 运行: 00:00:00",
                                    font=("Consolas", 9),
                                    fg=FG_SECOND, bg=BG_PANEL)
        self.uptime_lbl.pack(side=tk.RIGHT, padx=12)

        self.conn_status_lbl = tk.Label(bottom, text="等待连接",
                                         font=("Consolas", 9),
                                         fg=FG_SECOND, bg=BG_PANEL)
        self.conn_status_lbl.pack(side=tk.RIGHT, padx=8)

    def _create_video_panel(self, parent, title, color, index):
        """精致视频面板"""
        frame = tk.Frame(parent, bg=BG_PANEL, padx=2, pady=2)
        frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=8)
        
        tk.Label(
            frame, text=title, 
            font=("Microsoft YaHei UI", 12, "bold"),
            fg=color, bg=BG_PANEL
        ).pack(anchor=tk.W, padx=16, pady=10)
        
        label = tk.Label(frame, bg=BG_DARK, bd=2, relief=tk.FLAT)
        label.pack(fill=tk.BOTH, expand=True, padx=12, pady=(0,12))
        
        if index == 0:
            self.label_img1 = label
        else:
            self.label_img2 = label

    def _make_card(self, parent, title, value, color):
        """高级卡片（带阴影微动）"""
        card = tk.Frame(parent, bg=BG_CARD, padx=20, pady=16,
                        highlightbackground=BORDER, highlightthickness=1)
        card.pack(side=tk.LEFT, padx=8, expand=True, fill=tk.X)
        
        # 悬浮微动效果
        def on_enter(e): card.configure(highlightbackground=ACCENT)
        def on_leave(e): card.configure(highlightbackground=BORDER)
        card.bind("<Enter>", on_enter)
        card.bind("<Leave>", on_leave)
        
        tk.Label(
            card, text=title, 
            font=("Microsoft YaHei UI", 11),
            fg=FG_SECOND, bg=BG_CARD
        ).pack(anchor=tk.W)
        
        value_label = tk.Label(
            card, text=value, 
            font=("Microsoft YaHei UI", 18, "bold"),
            fg=color, bg=BG_CARD, justify=tk.LEFT
        )
        value_label.pack(anchor=tk.W, pady=(6, 0))
        card._value_label = value_label
        return card

    def _draw_dot(self, color):
        """状态指示灯（精致版）"""
        self.conn_dot.delete("all")
        color_map = {
            "green": FG_GREEN, 
            "orange": FG_ORANGE, 
            "red": FG_RED,
            "gray": "#666666"
        }
        fill = color_map.get(color, color)
        self.conn_dot.create_oval(2,2,16,16, fill=fill, outline=fill)

    def _start_breath_effect(self):
        """呼吸灯动画 + 时钟 + 运行时长"""
        if not self.running: return
        if self.conn_label.cget("fg") == FG_GREEN:
            self.breath_state = not self.breath_state
            c = "green" if self.breath_state else BG_NORMAL
            self._draw_dot(c)
        # 时钟
        self.clock_lbl.config(text=datetime.now().strftime("%H:%M:%S"))
        # 运行时长
        secs = int(time.time() - self._startup_time)
        self.uptime_lbl.config(
            text=f"⏳ 运行: {secs//3600:02d}:{(secs%3600)//60:02d}:{secs%60:02d}")
        self.root.after(1000, self._start_breath_effect)

    # ===================== 网络通信 =====================
    def _start_server(self):
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_sock.bind((HOST, PORT))
        self.server_sock.listen(1)
        self.server_sock.settimeout(1.0)
        threading.Thread(target=self._accept_loop, daemon=True).start()

    def _accept_loop(self):
        while self.running:
            try:
                conn, addr = self.server_sock.accept()
                self.root.after(0, self._on_connect, addr)
                self._recv_loop(conn)
            except socket.timeout:
                continue
            except Exception:
                if self.running:
                    self.root.after(0, self._on_disconnect)

    def _on_connect(self, addr):
        self.conn_label.config(text=f"✅ 已连接 · {addr[0]}:{addr[1]}", fg=FG_GREEN)
        self.conn_status_lbl.config(text="已连接", fg=FG_GREEN)

    def _on_disconnect(self):
        self.conn_label.config(text="❌ 连接断开 · 等待重连", fg=FG_ORANGE)
        self.conn_status_lbl.config(text="断开", fg=FG_ORANGE)

    def _recv_exact(self, conn, n):
        buf = b""
        while len(buf) < n:
            chunk = conn.recv(n - len(buf))
            if not chunk: raise ConnectionError()
            buf += chunk
        return buf

    def _recv_frame(self, conn):
        json_len = struct.unpack("!I", self._recv_exact(conn,4))[0]
        json_bytes = self._recv_exact(conn, json_len)
        meta = json.loads(json_bytes.decode("utf-8"))
        
        len1 = struct.unpack("!I", self._recv_exact(conn,4))[0]
        jpg1 = self._recv_exact(conn, len1) if len1>0 else b""
        
        len2 = struct.unpack("!I", self._recv_exact(conn,4))[0]
        jpg2 = self._recv_exact(conn, len2) if len2>0 else b""
        
        return meta, jpg1, jpg2

    def _recv_loop(self, conn):
        try:
            while self.running:
                meta, jpg1, jpg2 = self._recv_frame(conn)
                self.root.after(0, self._update_ui, meta, jpg1, jpg2)
        except Exception:
            self.root.after(0, self._on_disconnect)

    # ===================== UI更新 =====================
    def _update_ui(self, meta, jpg1, jpg2, recv_ts=None, add_history=True):
        if recv_ts is None:
            recv_ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")

        if add_history:
            self._frame_count += 1
            self.frame_lbl.config(text=f"📊 告警帧数: {self._frame_count}")

        self._update_image(self.label_img1, jpg1)
        self._update_image(self.label_img2, jpg2)

        date_str = f"{recv_ts[:4]}-{recv_ts[4:6]}-{recv_ts[6:8]}"
        time_str = f"{recv_ts[9:11]}:{recv_ts[11:13]}:{recv_ts[13:15]}"
        self.card_ts._value_label.config(text=f"{date_str}\n{time_str}")

        cpu = meta.get("cpu_usage", -1)
        npu = meta.get("npu_usage", -1)
        self._update_usage_card(self.card_cpu, cpu)
        self._update_usage_card(self.card_npu, npu)

        bal_alert = meta.get("balance_alert", False)
        climb_alert = meta.get("climbing_alert", False)
        lean_alert = meta.get("leaning_alert", False)
        prox_alert = meta.get("proximity_alert", False)
        min_dist = meta.get("min_distance_m", 999)

        self._update_alert_card(self.card_balance, bal_alert, "失衡!", "正常")
        self._update_alert_card(self.card_climb, climb_alert, "攀爬!", "正常")
        self._update_alert_card(self.card_lean, lean_alert, "探头!", "正常")
        self._update_alert_card(self.card_prox, prox_alert, f"{min_dist:.2f}m", "安全")

        alerts = self._get_alert_list(meta)
        if alerts:
            self.alert_banner.config(
                text=f"🚨 告警：{' | '.join(alerts)}",
                fg=FG_WHITE, bg=BG_ALERT
            )
            self.alert_frame.config(bg=BG_ALERT)
        else:
            self.alert_banner.config(
                text="● 系统正常运行中", 
                fg=FG_GREEN, bg=BG_CARD
            )
            self.alert_frame.config(bg=BG_CARD)

        self._update_detail_text(meta, recv_ts)
        if add_history:
            self._update_history(alerts, recv_ts, meta, jpg1, jpg2)

    def _update_image(self, label, img_data):
        if not img_data: return
        try:
            # use parent frame size (fixed), not label size (can grow)
            parent = label.master
            max_w = max(parent.winfo_width() - 16, 200)
            max_h = max(parent.winfo_height() - 16, 150)
            img = Image.open(BytesIO(img_data))
            ratio = min(max_w / img.width, max_h / img.height, 1.0)
            nw, nh = int(img.width * ratio), int(img.height * ratio)
            img = img.resize((nw, nh), Image.LANCZOS)
            photo = ImageTk.PhotoImage(img)
            label.config(image=photo)
            label.image = photo
        except:
            pass

    def _update_usage_card(self, card, value):
        if value < 0:
            card._value_label.config(text="--%", fg=FG_SECOND)
        else:
            color = FG_RED if value > 80 else FG_WHITE
            card._value_label.config(text=f"{value:.0f}%", fg=color)

    def _update_alert_card(self, card, is_alert, alert_text, normal_text):
        if is_alert:
            card._value_label.config(text=alert_text, fg=FG_RED)
        else:
            card._value_label.config(text=normal_text, fg=FG_GREEN)

    def _get_alert_list(self, meta):
        alerts = []
        if meta.get("balance_alert"): alerts.append("姿态失衡")
        if meta.get("climbing_alert"): alerts.append("危险攀爬")
        if meta.get("leaning_alert"): alerts.append("探头危险")
        if meta.get("proximity_alert"):
            dist = meta.get("min_distance_m",0)
            alerts.append(f"距离过近({dist:.2f}m)")
        return alerts

    def _update_detail_text(self, meta, recv_ts):
        self.detail_text.config(state=tk.NORMAL)
        self.detail_text.delete("1.0", tk.END)
        
        cpu = meta.get("cpu_usage", -1)
        npu = meta.get("npu_usage", -1)
        min_dist = meta.get("min_distance_m", 0)
        
        lines = [
            f"⏱ 时间：{recv_ts}",
            f"💻 CPU：{cpu:.0f}%    NPU：{npu:.0f}%" if cpu>=0 else "硬件状态：获取中",
            f"📏 最近距离：{min_dist:.3f} m",
            "─" * 34
        ]

        objects = meta.get("objects", [])
        if objects:
            lines.append(f"🎯 识别目标：{', '.join(objects)}")

        for p in meta.get("persons", []):
            idx = p.get("idx","?")
            s = p.get("status","未知")
            t = p.get("theta",0)
            d = p.get("min_dist_m",0)
            lines.append(f"👤 儿童{idx}：{s} | 倾角{t:.1f}° | 距离{d:.2f}m")

            el = p.get("elbow", -1)
            sh = p.get("shoulder", -1)
            kn = p.get("knee", -1)
            hi = p.get("hip", -1)
            ht = p.get("head_torso", -1)
            nw = p.get("nose_to_win", -1)
            if el >= 0 or sh >= 0 or kn >= 0 or hi >= 0:
                parts = []
                if el >= 0: parts.append(f"肘{el:.0f}°")
                if sh >= 0: parts.append(f"肩{sh:.0f}°")
                if kn >= 0: parts.append(f"膝{kn:.0f}°")
                if hi >= 0: parts.append(f"髋{hi:.0f}°")
                lines.append(f"    关节角度：{' | '.join(parts)}")
            if ht >= 0:
                lines.append(f"    头-躯干夹角：{ht:.1f}°")
            if nw >= 0:
                lines.append(f"    鼻-窗户距离：{nw:.3f}m")

        self.detail_text.insert("1.0", "\n".join(lines))
        self.detail_text.config(state=tk.DISABLED)

    def _update_history(self, alerts, recv_ts, meta, jpg1, jpg2):
        alert_str = " | ".join(alerts) if alerts else "正常运行"
        time_short = recv_ts[9:15] + recv_ts[16:19]
        prefix = "⚠" if alerts else "✔"

        self.history_list.insert(0, f" {prefix} 【{time_short}】 {alert_str}")
        if alerts:
            self.history_list.itemconfig(0, fg=FG_RED)
        self.history.insert(0, (recv_ts, meta, jpg1, jpg2))

        while len(self.history) > MAX_HISTORY:
            self.history.pop()
        while self.history_list.size() > MAX_HISTORY:
            self.history_list.delete(tk.END)
        self.hist_count_lbl.config(text=f"{self.history_list.size()} 条记录")

    def _on_history_select(self, event):
        sel = self.history_list.curselection()
        if not sel: return
        idx = sel[0]
        if idx < len(self.history):
            ts, meta, img1, img2 = self.history[idx]
            self._update_ui(meta, img1, img2, ts, add_history=False)

    def on_close(self):
        self.running = False
        try: self.server_sock.close()
        except: pass
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = HostServer(root)
    root.mainloop()