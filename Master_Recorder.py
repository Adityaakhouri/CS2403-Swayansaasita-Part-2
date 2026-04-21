import subprocess
import re
import socket
import threading
import struct
import time
import os
import json
import numpy as np
import pandas as pd
from datetime import datetime

CMD_PORT   = 9000
DEPTH_PORT = 5000
IMU_PORT   = 5001
GPS_PORT   = 5002
LIDAR_H    = 192
LIDAR_W    = 256
SAVE_BASE  = os.path.expanduser(
    "~/recordings_data")

CAMERA_MAP = {
    "21200000": ("front-left",  "Front-left"),
    "11300000": ("side-left",   "Side-left"),
    "21100000": ("front-right", "Front-right"),
    "21300000": ("side-right",  "Side-right"),
    "01000000": ("back-right",  "Back-right"),
    "11100000": ("back-left",   "Back-left"),
}


class MasterRecorder:

    def __init__(self):
        self.running       = False
        self.t0            = None
        self.depth_count   = 0
        self.imu_count     = 0
        self.gps_count     = 0
        self.ffmpeg_procs  = []
        self.cam_indexes   = []
        self.cam_names     = []
        self.base          = None
        self.imu_file      = None
        self.gps_file      = None
        self.depth_ts_file = None
        self.cam_status    = {}
        self.crashed_cams  = []
        self.cam_ts_files  = {}

    def _scene_name(self):
        today = datetime.now().strftime("%d-%m-%y")
        count = 1
        try:
            os.makedirs(SAVE_BASE, exist_ok=True)
            existing = [
                d for d in os.listdir(SAVE_BASE)
                if d.startswith(today)
                and os.path.isdir(
                    os.path.join(SAVE_BASE, d))
            ]
            count = len(existing) + 1
        except Exception as e:
            print(f"Scene name error: {e}")
        return f"{today}-{count:02d}"

    def is_hdmi_connected(self):
        try:
            result = subprocess.run(
                ["/usr/sbin/system_profiler", "SPDisplaysDataType", "-json"],
                capture_output=True, text=True, timeout=5
            )
            data = json.loads(result.stdout)
            displays = data.get("SPDisplaysDataType", [])

            for gpu in displays:
                for disp in gpu.get("spdisplays_ndrvs", []):
                    name = disp.get("_name", "").lower()

                    # Ignore built-in display
                    if "built-in" in name or "internal" in name:
                        continue

                    # If anything else exists → external display connected
                    return True

            return False

        except Exception:
            return False

    def get_status(self):
        hdmi = self.is_hdmi_connected()
        battery_pct = -1
        charging    = False
        try:
            result = subprocess.run(
                ["pmset", "-g", "batt"],
                capture_output=True, text=True)
            for line in result.stdout.split("\n"):
                if "%" in line:
                    m = re.search(r"(\d+)%", line)
                    if m:
                        battery_pct = int(m.group(1))
                    if "charging" in line.lower() \
                            and "not charging" \
                            not in line.lower() \
                            and "discharging" \
                            not in line.lower():
                        charging = True
        except Exception:
            pass
        return {
            "hdmi":         hdmi,
            "battery":      battery_pct,
            "charging":     charging,
            "running":      self.running,
            "crashed_cams": self.crashed_cams,
            "cam_status":   self.cam_status
        }

    def find_arducams(self):
        SUFFIX = "c45636d"
        sp = subprocess.run(
            ["/usr/sbin/system_profiler",
             "SPCameraDataType", "-json"],
            capture_output=True, text=True)
        try:
            data    = json.loads(sp.stdout)
            cameras = data.get(
                "SPCameraDataType", [])
        except Exception:
            cameras = []

        sp_locs = []
        for cam in cameras:
            if "Arducam" not in cam.get(
                    "_name", ""):
                continue
            uid = cam.get(
                "spcamera_unique-id", "")
            if uid.startswith("0x"):
                raw = uid[2:]
                if raw.endswith(SUFFIX):
                    loc = raw[
                        :-len(SUFFIX)].zfill(8)
                else:
                    loc = raw.zfill(8)
                sp_locs.append(loc)

        ffmpeg = subprocess.run(
            ["/opt/homebrew/bin/ffmpeg",
             "-f", "avfoundation",
             "-list_devices", "true", "-i", ""],
            capture_output=True, text=True)

        indexes = []
        in_video = False
        for line in ffmpeg.stderr.split("\n"):
            if "AVFoundation video devices" in line:
                in_video = True
                continue
            if "AVFoundation audio devices" in line:
                break
            if in_video and "Arducam_12MP" in line:
                m = re.search(r"\[(\d+)\]", line)
                if m:
                    idx = int(m.group(1))
                    if idx not in indexes:
                        indexes.append(idx)

        result = []
        for loc, idx in zip(sp_locs, indexes):
            info = CAMERA_MAP.get(
                loc,
                (f"unknown_{loc}", "Unknown"))
            result.append({
                "index":    idx,
                "location": loc,
                "name":     info[0],
                "position": info[1]
            })

        result.sort(key=lambda x: x["name"])
        print(f"Found {len(result)} cameras:")
        for c in result:
            print(f"  [{c['index']}] {c['name']} "
                  f"({c['position']}) "
                  f"loc={c['location']}")
        return result

    def start_cameras(self, cameras):
        cam_timestamps  = {}
        self.cam_status = {}
        self.crashed_cams = []
        self.cam_ts_files = {}

        for cam in cameras:
            out = (f"{self.base}/camera/"
                   f"{cam['name']}.mp4")
            t_start = time.time()
            cam_timestamps[cam['name']] = t_start
            self.cam_status[cam['name']] = "recording"

            # Open per-camera frame timestamp file
            ts_path = (f"{self.base}/camera/"
                       f"{cam['name']}_frames.csv")
            self.cam_ts_files[cam['name']] = open(
                ts_path, "w")
            self.cam_ts_files[cam['name']].write(
                "frame_id,t_unix\n")
            self.cam_ts_files[cam['name']].flush()

            cmd = [
                "/opt/homebrew/bin/ffmpeg",
                "-f", "avfoundation",
                "-framerate", "30",
                "-video_size", "1920x1080",
                "-i", str(cam["index"]),
                "-c:v", "libx264",
                "-preset", "ultrafast",
                "-crf", "23", out
            ]
            proc = subprocess.Popen(
                cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL)
            self.ffmpeg_procs.append(proc)
            print(f"  [{cam['index']}] "
                  f"{cam['name']} "
                  f"({cam['position']}) "
                  f"-> {cam['name']}.mp4 "
                  f"t={t_start:.3f}")

            threading.Thread(
                target=self._count_frames,
                args=(proc, cam['name'], t_start),
                daemon=True).start()
            threading.Thread(
                target=self._monitor_ffmpeg,
                args=(proc, cam['name']),
                daemon=True).start()

        sync_path = (f"{self.base}/camera/"
                     f"camera_timestamps.csv")
        with open(sync_path, "w") as f:
            f.write("camera,t_start_unix,"
                    "t_stop_unix,duration_sec,"
                    "fps,resolution\n")
            for name, t in sorted(
                    cam_timestamps.items()):
                f.write(f"{name},{t:.6f},"
                        f",,30,1920x1080\n")
        print("  Camera sync saved")

    def _count_frames(self, proc, name, t_start):
        frame_id  = 0
        interval  = 1.0 / 30.0
        next_time = t_start
        while proc.poll() is None and self.running:
            now = time.time()
            if now >= next_time:
                ts_file = self.cam_ts_files.get(name)
                if ts_file:
                    try:
                        ts_file.write(
                            f"{frame_id},"
                            f"{next_time:.6f}\n")
                        ts_file.flush()
                    except Exception:
                        pass
                frame_id  += 1
                next_time += interval
            else:
                time.sleep(
                    max(0, next_time - now - 0.001))

    def _monitor_ffmpeg(self, proc, name):
        proc.wait()
        if self.running:
            print(f"  WARNING: {name} crashed!")
            self.cam_status[name] = "crashed"
            if name not in self.crashed_cams:
                self.crashed_cams.append(name)
            all_dead = all(
                p.poll() is not None
                for p in self.ffmpeg_procs)
            if all_dead:
                print("All cameras died — "
                      "stopping recording")
                threading.Thread(
                    target=self.stop_recording,
                    daemon=True).start()

    def handle_ipad(self, conn, addr):
        print(f"iPad connected from {addr[0]}")
        try:
            data = conn.recv(512).decode().strip()
        except Exception:
            conn.close()
            return

        if data == "CHECK":
            cameras   = self.find_arducams()
            count     = len(cameras)
            ids_str   = ",".join(
                str(c["index"])
                for c in cameras)
            names_str = ",".join(
                c["name"] for c in cameras)
            reply = (f"CAMERAS_FOUND:{count}"
                     f"|IDS:{ids_str}"
                     f"|NAMES:{names_str}")
            print(f"CHECK -> {count} cameras")
            try:
                conn.send(reply.encode())
            except Exception:
                pass
            conn.close()
            return

        if data == "PING":
            try:
                conn.send(b"PONG")
            except Exception:
                pass
            conn.close()
            return

        if data == "STATUS":
            try:
                status  = self.get_status()
                encoded = json.dumps(
                    status).encode()
                length  = struct.pack(
                    ">I", len(encoded))
                conn.send(length + encoded)
            except Exception:
                pass
            conn.close()
            return

        if data == "BATTERY":
            try:
                result = subprocess.run(
                    ["pmset", "-g", "batt"],
                    capture_output=True, text=True)
                pct      = "-1"
                charging = False
                for line in result.stdout.split("\n"):
                    if "%" in line:
                        m = re.search(
                            r"(\d+)%", line)
                        if m:
                            pct = m.group(1)
                        if "charging" in line.lower() \
                                and "not charging" \
                                not in line.lower() \
                                and "discharging" \
                                not in line.lower():
                            charging = True
                reply = (f"{pct}C"
                         if charging else pct)
                conn.send(reply.encode())
            except Exception:
                conn.send(b"-1")
            conn.close()
            return

        if data.startswith("VERIFY:"):
            path   = data.replace(
                "VERIFY:", "").strip()
            result = self._verify_session(path)
            try:
                encoded = json.dumps(
                    result).encode()
                length  = struct.pack(
                    ">I", len(encoded))
                conn.send(length + encoded)
            except Exception:
                pass
            conn.close()
            return

        if data.startswith("START"):
            try:
                ipad_t0 = float(
                    data.split(":")[1])
            except (IndexError, ValueError):
                ipad_t0 = None

            self.crashed_cams = []
            scene     = self._scene_name()
            self.base = f"{SAVE_BASE}/{scene}"
            for d in ["camera", "ipad/depth",
                      "ipad/poses", "logs"]:
                os.makedirs(
                    f"{self.base}/{d}",
                    exist_ok=True)

            self.imu_file = open(
                f"{self.base}/logs/imu.csv", "w")
            self.gps_file = open(
                f"{self.base}/logs/gps.csv", "w")
            self.depth_ts_file = open(
                f"{self.base}/ipad/"
                f"depth_timestamps.csv", "w")

            self.imu_file.write(
                "t_unix,"
                "ax_with_g,ay_with_g,az_with_g,"
                "gx,gy,gz,"
                "roll,pitch,yaw,"
                "grav_x,grav_y,grav_z,"
                "ax_no_g,ay_no_g,az_no_g\n")
            self.imu_file.flush()
            self.gps_file.write(
                "t_unix,lat,lon,alt,speed_ms\n")
            self.gps_file.flush()
            self.depth_ts_file.write(
                "frame_id,t_unix\n")
            self.depth_ts_file.flush()

            self.depth_count  = 0
            self.imu_count    = 0
            self.gps_count    = 0
            self.ffmpeg_procs = []

            cameras = self.find_arducams()
            self.cam_indexes = [
                c["index"] for c in cameras]
            self.cam_names   = [
                c["name"]  for c in cameras]

            print(f"Starting "
                  f"{len(cameras)} cameras...")
            self.start_cameras(cameras)

            # Wait for ffmpeg to initialise
            print("Waiting for cameras to initialise...")
            time.sleep(0.5)

            # Set t0 AFTER cameras are running
            # Use iPad t0 if provided, else now
            self.t0 = (ipad_t0
                       if ipad_t0
                       else time.time())

            self.running = True
            for fn in [self._recv_imu,
                       self._recv_gps,
                       self._print_stats]:
                threading.Thread(
                    target=fn,
                    daemon=True).start()

            cam_ids   = ",".join(
                str(c["index"])
                for c in cameras)
            cam_names = ",".join(
                c["name"] for c in cameras)
            reply = (f"READY:{self.t0}|"
                     f"CAMERAS:{cam_ids}|"
                     f"NAMES:{cam_names}|"
                     f"PATH:{self.base}")
            try:
                conn.send(reply.encode())
            except Exception:
                pass
            conn.close()
            print(f"Recording -> {self.base}/")
            return

        if data == "STOP":
            print("\nSTOP received from iPad")
            saved_base  = self.base
            saved_depth = self.depth_count
            saved_imu   = self.imu_count
            saved_gps   = self.gps_count
            self.stop_recording()
            reply = (f"STOPPED|"
                     f"PATH:{saved_base}|"
                     f"DEPTH:{saved_depth}|"
                     f"IMU:{saved_imu}|"
                     f"GPS:{saved_gps}")
            try:
                conn.send(reply.encode())
            except Exception:
                pass
            conn.close()

    def stop_recording(self):
        if self.imu_file is None:
            return
        t_stop = time.time()
        self.running = False

        # Stop ffmpeg first
        for proc in self.ffmpeg_procs:
            try:
                if proc.poll() is None:
                    proc.stdin.write(b"q")
                    proc.stdin.flush()
            except Exception:
                pass

        for proc in self.ffmpeg_procs:
            try:
                if proc.poll() is None:
                    proc.wait(timeout=10)
            except Exception:
                try:
                    if proc.poll() is None:
                        proc.kill()
                except Exception:
                    pass

        # Close per-camera frame timestamp files
        for name, f in self.cam_ts_files.items():
            try:
                f.close()
            except Exception:
                pass
        self.cam_ts_files = {}

        # Now extract accurate timestamps
        # using ffprobe (videos are fully written)
        self._extract_frame_timestamps()

        # Update camera_timestamps.csv
        sync_path = (f"{self.base}/camera/"
                     f"camera_timestamps.csv")
        if os.path.exists(sync_path):
            try:
                lines = open(
                    sync_path).readlines()
                with open(sync_path, "w") as f:
                    f.write(
                        "camera,t_start_unix,"
                        "t_stop_unix,"
                        "duration_sec,"
                        "fps,resolution\n")
                    for line in lines[1:]:
                        parts = line.strip(
                            ).split(",")
                        if len(parts) >= 2:
                            t_start = float(
                                parts[1])
                            dur = t_stop - t_start
                            f.write(
                                f"{parts[0]},"
                                f"{t_start:.6f},"
                                f"{t_stop:.6f},"
                                f"{dur:.3f},"
                                f"30,1920x1080\n")
            except Exception as e:
                print(f"Sync update error: {e}")

        print(f"  {len(self.ffmpeg_procs)} "
              f"cameras stopped")

        if self.imu_file:
            self.imu_file.close()
            self.imu_file = None
        if self.gps_file:
            self.gps_file.close()
            self.gps_file = None
        if self.depth_ts_file:
            self.depth_ts_file.close()
            self.depth_ts_file = None

        try:
            imu_path = f"{self.base}/logs/imu.csv"
            df = pd.read_csv(imu_path)
            before = len(df)
            df = df.drop_duplicates('t_unix')
            df = df.sort_values('t_unix')
            df.to_csv(imu_path, index=False)
            print(f"  IMU cleaned: "
                  f"{before} → {len(df)} rows")
        except Exception as e:
            print(f"  IMU cleanup skipped: {e}")

        if self.base:
            info = {
                "t0":           self.t0,
                "t_stop":       t_stop,
                "duration_sec": t_stop - self.t0,
                "scene":        os.path.basename(
                    self.base),
                "cameras":      len(
                    self.cam_indexes),
                "cam_indexes":  self.cam_indexes,
                "cam_names":    self.cam_names,
                "crashed_cams": self.crashed_cams,
                "depth_frames": self.depth_count,
                "imu_samples":  self.imu_count,
                "gps_points":   self.gps_count
            }
            with open(
                f"{self.base}/session_info.json",
                "w"
            ) as f:
                json.dump(info, f, indent=2)
        print(f"Saved: {self.base}/")
        print(f"  Depth:{self.depth_count} "
              f"IMU:{self.imu_count} "
              f"GPS:{self.gps_count}")
        self.base = None

    def _extract_frame_timestamps(self):
        """Use ffprobe to extract accurate
        per-frame pts timestamps from each
        video file after recording."""
        if not self.base:
            return
        cam_path = f"{self.base}/camera"
        if not os.path.exists(cam_path):
            return

        for mp4 in sorted(os.listdir(cam_path)):
            if not mp4.endswith(".mp4"):
                continue
            name     = mp4.replace(".mp4", "")
            mp4_path = f"{cam_path}/{mp4}"
            out_path = (f"{cam_path}/"
                        f"{name}_frames.csv")

            print(f"  Extracting timestamps: {name}")
            try:
                result = subprocess.run([
                    "/opt/homebrew/bin/ffprobe",
                    "-v", "quiet",
                    "-select_streams", "v:0",
                    "-show_entries",
                    "packet=pts_time",
                    "-of", "csv=p=0",
                    mp4_path
                ], capture_output=True, text=True,
                   timeout=60)

                lines = result.stdout.strip(
                    ).split("\n")

                # Get camera start time
                sync_path = (f"{cam_path}/"
                             f"camera_timestamps"
                             f".csv")
                t_start = None
                if os.path.exists(sync_path):
                    try:
                        with open(sync_path) as f:
                            for line in f:
                                if line.startswith(
                                        name + ","):
                                    parts = \
                                        line.split(",")
                                    if len(parts) >= 2:
                                        t_start = float(
                                            parts[1])
                                    break
                    except Exception:
                        pass

                with open(out_path, "w") as f:
                    f.write("frame_id,"
                            "pts_seconds,"
                            "t_unix\n")
                    for i, line in enumerate(lines):
                        line = line.strip()
                        if not line:
                            continue
                        try:
                            pts = float(line)
                            t_unix = (
                                (t_start + pts)
                                if t_start
                                else pts)
                            f.write(
                                f"{i},"
                                f"{pts:.6f},"
                                f"{t_unix:.6f}\n")
                        except ValueError:
                            continue

                frame_count = len([
                    l for l in lines
                    if l.strip()])
                print(f"    {name}: "
                      f"{frame_count} frames")

            except Exception as e:
                print(f"    ffprobe error "
                      f"{name}: {e}")

    def _verify_session(self, path):
        result   = {}
        cam_path = f"{path}/camera"
        cams     = []
        if os.path.exists(cam_path):
            for f in sorted(os.listdir(cam_path)):
                if f.endswith(".mp4"):
                    fp   = f"{cam_path}/{f}"
                    size = os.path.getsize(fp)
                    cams.append({
                        "name": f,
                        "size": size,
                        "ok":   size > 100000
                    })
        result["cameras"] = cams

        depth_path  = f"{path}/ipad/depth"
        depth_count = 0
        if os.path.exists(depth_path):
            depth_count = len([
                f for f in os.listdir(depth_path)
                if f.endswith(".npy")])
        result["depth"] = depth_count

        imu_path = f"{path}/logs/imu.csv"
        imu_rows = 0
        if os.path.exists(imu_path):
            try:
                with open(imu_path) as f:
                    imu_rows = sum(
                        1 for _ in f) - 1
            except Exception:
                pass
        result["imu"] = imu_rows

        gps_path = f"{path}/logs/gps.csv"
        gps_rows = 0
        if os.path.exists(gps_path):
            try:
                with open(gps_path) as f:
                    gps_rows = sum(
                        1 for _ in f) - 1
            except Exception:
                pass
        result["gps"] = gps_rows

        cam_ok  = len(cams) == 6 and all(
            c["ok"] for c in cams)
        data_ok = depth_count > 0 and imu_rows > 0
        result["healthy"] = cam_ok and data_ok
        return result

    def _handle_depth_stream(self, conn):
        print("Depth stream connected")
        frames = 0
        try:
            while True:
                raw_len = b""
                while len(raw_len) < 4:
                    chunk = conn.recv(
                        4 - len(raw_len))
                    if not chunk:
                        print(
                            f"Depth stream ended "
                            f"after {frames} frames")
                        return
                    raw_len += chunk

                msg_len = struct.unpack(
                    ">I", raw_len)[0]

                data = b""
                while len(data) < msg_len:
                    chunk = conn.recv(
                        min(65536,
                            msg_len - len(data)))
                    if not chunk:
                        return
                    data += chunk

                if not self.running \
                        or not self.base \
                        or self.depth_ts_file \
                        is None:
                    continue
                if len(data) < 72:
                    continue

                pose = np.frombuffer(
                    data[8:72],
                    dtype=np.float32
                ).reshape(4, 4)
                pixels = np.frombuffer(
                    data[72:], dtype=np.float32)

                if pixels.size == LIDAR_H * LIDAR_W:
                    depth  = pixels.reshape(
                        LIDAR_H, LIDAR_W)
                    fid    = f"{self.depth_count:06d}"
                    t_now  = time.time()
                    np.save(
                        f"{self.base}/ipad/depth/"
                        f"{fid}.npy", depth)
                    np.save(
                        f"{self.base}/ipad/poses/"
                        f"{fid}.npy", pose)
                    self.depth_ts_file.write(
                        f"{fid},{t_now:.6f}\n")
                    self.depth_ts_file.flush()
                    self.depth_count += 1
                    frames += 1

        except Exception as e:
            print(f"Depth stream error: {e}")
        finally:
            try:
                conn.close()
            except Exception:
                pass

    def _depth_server_always_on(self):
        server = socket.socket(
            socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_REUSEADDR, 1)
        server.bind(("0.0.0.0", DEPTH_PORT))
        server.listen(5)
        server.settimeout(1.0)
        print("Depth server ready "
              "on port 5000 (TCP)")
        while True:
            try:
                conn, addr = server.accept()
                threading.Thread(
                    target=self._handle_depth_stream,
                    args=(conn,),
                    daemon=True).start()
            except socket.timeout:
                continue
            except Exception:
                continue

    def _recv_imu(self):
        sock = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("0.0.0.0", IMU_PORT))
        sock.settimeout(1.0)
        while self.running:
            try:
                data, _ = sock.recvfrom(512)
            except socket.timeout:
                continue
            if len(data) >= 128:
                vals = struct.unpack(
                    "16d", data[:128])
                self.imu_file.write(
                    f"{vals[0]:.6f}," +
                    ",".join(
                        f"{v:.6f}"
                        for v in vals[1:]
                    ) + "\n")
                self.imu_file.flush()
                self.imu_count += 1
            elif len(data) >= 80:
                vals = struct.unpack(
                    "10d", data[:80])
                padding = ",0.000000" * 6
                self.imu_file.write(
                    f"{vals[0]:.6f}," +
                    ",".join(
                        f"{v:.6f}"
                        for v in vals[1:]
                    ) + padding + "\n")
                self.imu_file.flush()
                self.imu_count += 1

    def _recv_gps(self):
        sock = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("0.0.0.0", GPS_PORT))
        sock.settimeout(1.0)
        while self.running:
            try:
                data, _ = sock.recvfrom(256)
            except socket.timeout:
                continue
            if len(data) >= 40:
                vals = struct.unpack(
                    "5d", data[:40])
                self.gps_file.write(
                    ",".join(
                        f"{v:.8f}"
                        for v in vals
                    ) + "\n")
                self.gps_file.flush()
                self.gps_count += 1

    def _print_stats(self):
        while self.running:
            time.sleep(10)
            if self.t0:
                dur = time.time() - self.t0
                print(f"  {dur:.0f}s | "
                      f"Depth:{self.depth_count}"
                      f" | IMU:{self.imu_count}"
                      f" | GPS:{self.gps_count}")

    def listen(self):
        print("=== Bike Rig Master Recorder ===")
        print(f"Listening on port {CMD_PORT}...")
        print("Open BikeRecorder on iPad\n")

        threading.Thread(
            target=self._depth_server_always_on,
            daemon=True).start()

        server = socket.socket(
            socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_REUSEADDR, 1)
        server.bind(("0.0.0.0", CMD_PORT))
        server.listen(10)
        while True:
            try:
                conn, addr = server.accept()
                threading.Thread(
                    target=self.handle_ipad,
                    args=(conn, addr),
                    daemon=True).start()
            except KeyboardInterrupt:
                print("\nShutting down...")
                if self.running:
                    self.stop_recording()
                break


if __name__ == "__main__":
    MasterRecorder().listen()
