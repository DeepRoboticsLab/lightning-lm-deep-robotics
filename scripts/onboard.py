#!/usr/bin/env python3
"""Configure this robot once; load its ROS environment for every onboard command."""

import argparse
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shlex
import signal
import subprocess
import sys
import tempfile
import time

import yaml

ROOT = Path(__file__).resolve().parents[1]
RVIZ = str(ROOT / "install/lightning/lib/lightning/run_rviz")


def file_hash(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def level_quaternion(up):
    if len(up) != 3 or not all(math.isfinite(v) for v in up):
        raise ValueError("The map's upward vector must contain three finite values.")
    length = math.sqrt(sum(v * v for v in up))
    if length < 1e-9:
        raise ValueError("The map's upward vector cannot be zero.")
    x, y, z = [v / length for v in up]
    # Shortest rotation from measured up to +Z; no planar constraint or cloud fit.
    if z < -1 + 1e-10:
        return [1., 0., 0., 0.]
    quaternion = [y, -x, 0., 1. + z]
    norm = math.sqrt(sum(v * v for v in quaternion))
    return [v / norm for v in quaternion]


def process_identity(pid):
    return (Path("/proc") / str(pid) / "stat").read_text().rsplit(")", 1)[1].split()[19]


def mapping_session(settings):
    path = settings.with_suffix(".session.json")
    try:
        session = json.loads(path.read_text())
        if process_identity(session["pid"]) == session["process_start"]:
            return session
    except (OSError, ValueError, KeyError, IndexError):
        pass
    return None


def write_atomic(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(mode="w", dir=str(path.parent), delete=False) as stream:
        temporary = Path(stream.name)
        stream.write(text)
    try:
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


def cpu_list(value):
    if value is None:
        return None
    if not re.fullmatch(r"\d+(?:-\d+)?(?:,\d+(?:-\d+)?)*", value):
        raise ValueError("CPU lists must look like 7 or 4-7.")
    for part in value.split(","):
        bounds = [int(v) for v in part.split("-")]
        first, last = bounds[0], bounds[-1]
        if first > last or last >= (os.cpu_count() or 1):
            raise ValueError("CPU list contains an unavailable CPU: " + value)
    return value


def find_driver_workspace(model):
    launch = "msg_MID360s_launch.py" if model == "mid360s" else "msg_MID360_launch.py"
    candidates = set()
    for pattern in ("*/install/setup.bash", "*/*/install/setup.bash"):
        for setup in Path.home().glob(pattern):
            if any(setup.parent.rglob(launch)):
                candidates.add(setup.parent.parent.resolve())
        if candidates:
            break
    if len(candidates) != 1:
        found = ", ".join(str(path) for path in sorted(candidates)) or "none"
        raise ValueError("Expected one built full Livox driver workspace under your home directory; "
                         "found " + found + ". Configure with --driver-workspace PATH.")
    return str(candidates.pop())


def configure(args):
    previous = json.loads(args.settings.read_text()) if args.settings.exists() else {}
    if previous and previous.get("platform") != args.platform:
        raise ValueError("This checkout is configured for another robot. Use a separate checkout/settings file.")
    domain = args.domain if args.domain is not None else previous.get(
        "domain", 42 if args.platform == "agx" else int(os.environ.get("ROS_DOMAIN_ID", "0")))
    if not 0 <= domain <= 232:
        raise ValueError("ROS domain must be between 0 and 232.")
    driver = args.driver_workspace or previous.get("driver_workspace")
    model = args.driver_model or previous.get("driver_model", "mid360s")
    if args.platform == "agx":
        if not driver:
            driver = find_driver_workspace(model)
        if not driver or not (Path(driver).expanduser() / "install/setup.bash").is_file():
            raise ValueError("Set --driver-workspace to the built full Livox driver workspace.")
        driver = str(Path(driver).expanduser().resolve())
        launch = "msg_MID360s_launch.py" if model == "mid360s" else "msg_MID360_launch.py"
        if not any((Path(driver) / "install").rglob(launch)):
            raise ValueError("The full driver workspace must install " + launch + "; the Lightning-LM "
                             "message-only package cannot start the LiDAR.")
    config_path = args.settings.with_suffix(".yaml")
    config_text = None
    if args.config or not config_path.exists():
        sensor = args.config or ROOT / "config" / ("mid360.yaml" if args.platform == "agx" else "m20_pro.yaml")
        config = yaml.safe_load(Path(sensor).expanduser().read_text())
        config["system"]["with_ui"] = False
        if args.platform == "agx" and args.config is None:
            # Livox manual: IMU origin in LiDAR coordinates is the inverse translation.
            config["fasterlio"]["extrinsic_T"] = [-0.011, -0.02329, 0.04412]
            config["fasterlio"]["extrinsic_R"] = [1, 0, 0, 0, 1, 0, 0, 0, 1]
        config_text = yaml.safe_dump(config, sort_keys=False)
    profile = dict(version=1, platform=args.platform, config=str(config_path), domain=domain,
                   driver_workspace=driver,
                   driver_model=model,
                   app_cpus=cpu_list(args.app_cpus if args.app_cpus is not None else previous.get(
                       "app_cpus", "7" if args.platform == "m20" else None)),
                   rviz_cpus=cpu_list(args.rviz_cpus if args.rviz_cpus is not None else previous.get(
                       "rviz_cpus", "6" if args.platform == "m20" else None)))
    if config_text is not None:
        write_atomic(config_path, config_text)
    write_atomic(args.settings, json.dumps(profile, indent=2) + "\n")
    print("Saved settings:", args.settings)
    print("Runtime configuration:", config_path)
    print("Later commands load both automatically; no shell exports are needed.")


def load_profile(path):
    if not path.is_file():
        raise ValueError("Run 'python3 scripts/onboard.py configure agx' "
                         "or 'configure m20' once first.")
    profile = json.loads(path.read_text())
    if profile.get("version") != 1 or profile.get("platform") not in ("agx", "m20"):
        raise ValueError("Unsupported onboard settings: " + str(path))
    if not isinstance(profile.get("domain"), int) or not 0 <= profile["domain"] <= 232:
        raise ValueError("Invalid ROS domain in onboard settings.")
    if not Path(profile["config"]).is_file():
        raise ValueError("Runtime configuration is missing: " + profile["config"])
    for key in ("app_cpus", "rviz_cpus"):
        cpu_list(profile.get(key))
    return profile


def environment(profile, driver=False, viewer=False):
    # Build each child environment independently. In particular, the Livox driver
    # must not inherit this checkout's message-only package through an old overlay.
    env = dict(os.environ)
    for key in ("AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH", "COLCON_PREFIX_PATH", "PYTHONPATH",
                "LD_LIBRARY_PATH", "ROS_DISTRO", "ROS_VERSION", "ROS_PYTHON_VERSION",
                "RMW_IMPLEMENTATION", "FASTRTPS_DEFAULT_PROFILES_FILE"):
        env.pop(key, None)
    exports = dict(ROS_DOMAIN_ID=str(profile["domain"]),
                   ROS_LOCALHOST_ONLY="1" if profile["platform"] == "agx" else "0",
                   RMW_IMPLEMENTATION="rmw_fastrtps_cpp",
                   FASTRTPS_DEFAULT_PROFILES_FILE=str(ROOT / "config/fastdds.xml"))
    prelude = ""
    if profile["platform"] == "m20":
        if os.geteuid() != 0:
            raise ValueError("M20 applications need the root shell described in README section 6.1.")
        prelude += "source /opt/robot/scripts/setup_ros2.sh || exit $?\n"
    prelude += "export " + " ".join(k + "=" + shlex.quote(v) for k,v in exports.items()) + "\n"
    if driver:
        distro_file = ROOT / "install/lightning/share/lightning/ros_distro"
        distro = distro_file.read_text().strip()
        if distro not in ("foxy", "humble"):
            raise ValueError("Rebuild Lightning-LM with ROS Foxy or Humble first.")
        sources = [Path("/opt/ros") / distro / "setup.bash",
                   Path(profile["driver_workspace"]) / "install/setup.bash"]
    else:
        sources = [ROOT / "scripts/setup.bash"]
    for source in sources:
        prelude += "source " + shlex.quote(str(source)) + " || exit $?\n"
    if viewer:
        if not env.get("DISPLAY"):
            raise ValueError("No X11 display. Open a new laptop desktop terminal and connect with "
                             "ssh -Y -C (for AGX, use -J as in README section 6.2).")
        exports.update(__GLX_VENDOR_LIBRARY_NAME="mesa", LIBGL_ALWAYS_SOFTWARE="1",
                       LP_NUM_THREADS="2", QT_X11_NO_MITSHM="1")
        prelude += "export " + " ".join(k + "=" + shlex.quote(v) for k,v in exports.items()) + "\n"
    # Firmware/ROS setup can print notices. Keep those on stderr so a child's
    # stdout (in particular the sensor probe's JSON) remains machine-readable.
    # A brace group preserves the sourced environment in this shell.
    return env, "{\n" + prelude + '} >&2\nexec "$@"'


def start(command, profile, driver=False, viewer=False, cpus=None, **kwargs):
    env, prelude = environment(profile, driver, viewer)
    if cpus:
        command = ["taskset", "-c", cpu_list(cpus)] + command
    return subprocess.Popen(["bash", "--noprofile", "--norc", "-c", prelude,
                             "lightning-onboard"] + command, cwd=str(ROOT), env=env,
                            start_new_session=True, **kwargs)


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
    if process.returncode is not None and process.returncode < 0 and process.returncode not in (-2, -15):
        print("onboard: child", process.pid, "terminated by", signal.Signals(-process.returncode).name,
              file=sys.stderr)


def wait(process):
    try:
        return process.wait()
    except KeyboardInterrupt:
        return 130
    finally:
        stop(process)


def sensor_probe(profile):
    # This branch runs only inside the selected ROS environment. Raw callbacks
    # avoid constructing thousands of Python point objects per Livox scan.
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Imu, PointCloud2
    common = yaml.safe_load(Path(profile["config"]).read_text())["common"]
    if profile["platform"] == "agx":
        from livox_ros_driver2.msg import CustomMsg
        cloud_type, cloud_topic = CustomMsg, common["livox_lidar_topic"]
    else:
        cloud_type, cloud_topic = PointCloud2, common["lidar_topic"]
    rclpy.init()
    node = rclpy.create_node("lightning_sensor_check_" + str(os.getpid()))
    counts = dict(lidar=0, imu=0)
    acceleration = []
    def callback(key):
        def receive(_):
            counts[key] += 1
        return receive
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(cloud_type, cloud_topic, callback("lidar"), qos, raw=True)
    def receive_imu(message):
        counts["imu"] += 1
        a = message.linear_acceleration
        if all(math.isfinite(v) for v in (a.x, a.y, a.z)):
            acceleration.append((a.x, a.y, a.z))
    node.create_subscription(Imu, common["imu_topic"], receive_imu, qos)
    until = time.monotonic() + 3
    try:
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.1)
        result = dict(counts=counts, topics=[cloud_topic, common["imu_topic"]],
                      publishers=[node.count_publishers(t) for t in [cloud_topic, common["imu_topic"]]],
                      nodes=node.get_node_names())
        if len(acceleration) >= 20:
            mean = [sum(row[i] for row in acceleration) / len(acceleration) for i in range(3)]
            magnitude = math.sqrt(sum(v * v for v in mean))
            variance = sum(sum((row[i] - mean[i]) ** 2 for i in range(3))
                           for row in acceleration) / len(acceleration)
            if magnitude > 1e-6 and math.sqrt(variance) / magnitude < .05:
                result["imu_reference"] = dict(up_in_map=[v / magnitude for v in mean],
                                               samples=len(acceleration),
                                               relative_acceleration_std=math.sqrt(variance) / magnitude)
        print(json.dumps(result))
    finally:
        node.destroy_node()
        rclpy.shutdown()


def check_sensors(args, profile):
    process = start([sys.executable, str(Path(__file__).resolve()), "--settings", str(args.settings),
                     "_probe"], profile, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        out, err = process.communicate(timeout=15)
        if process.returncode:
            raise ValueError("Sensor check failed: " + err.strip())
        try:
            result = json.loads(out)
        except json.JSONDecodeError as error:
            raise ValueError("Sensor check returned invalid JSON. "
                             "stdout: " + repr(out[:1000]) + "; stderr: " + repr(err[:1000])) from error
        return result
    finally:
        stop(process)


def save_map(settings, map_id):
    import rclpy
    from lightning.srv import SaveMap
    rclpy.init()
    node = rclpy.create_node("lightning_map_save_" + str(os.getpid()))
    session = mapping_session(settings)
    try:
        client = node.create_client(SaveMap, "/lightning/save_map")
        if not client.wait_for_service(timeout_sec=5):
            raise ValueError("No mapping save service. Start mapping before saving its map.")
        request = SaveMap.Request()
        request.map_id = map_id
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=120)
        if not future.done():
            raise ValueError("Map save is still pending. Check the mapping terminal before stopping it.")
        response = future.result().response
        if response != 0:
            raise ValueError("Map save failed (response " + str(response) + "). Check the mapping "
                             "terminal and use a new map ID.")
        print("Map saved (response: 0):", ROOT / "data" / map_id)
        if session and session.get("imu_reference") and mapping_session(settings) == session:
            directory = ROOT / "data" / map_id
            metadata = dict(version=1, source="stationary IMU samples before mapping",
                            pcd_sha256=file_hash(directory / "global.pcd"), **session["imu_reference"])
            write_atomic(directory / "map_view.json", json.dumps(metadata, indent=2) + "\n")
            print("Saved the map's gravity reference for RViz leveling.")
        else:
            print("No stationary startup gravity reference; RViz will use the original map frame.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


def view_map(args, profile):
    pcd = args.map.expanduser().resolve()
    if pcd.is_dir():
        pcd /= "global.pcd"
    if not pcd.is_file():
        raise ValueError("Saved map does not exist: " + str(pcd))
    up = args.up
    reference = pcd.parent / "map_view.json"
    if not args.original_frame and up is None and reference.is_file():
        saved = json.loads(reference.read_text())
        if saved.get("version") != 1 or saved.get("pcd_sha256") != file_hash(pcd):
            raise ValueError("The saved gravity reference does not match this PCD. "
                             "Use --original-frame to inspect the original coordinates.")
        up = saved["up_in_map"]
    rotation = level_quaternion(up) if up is not None else [0., 0., 0., 1.]
    environment(profile, viewer=True)  # Diagnose missing X11 before loading a map.
    with tempfile.TemporaryDirectory(prefix="lightning-map-") as directory:
        directory = Path(directory)
        metadata = directory / "map.json"
        topic = "/lightning/saved_map_" + str(os.getpid())
        frame_topic = "/lightning/saved_map_tf_" + str(os.getpid())
        publisher = start(["ros2", "run", "lightning", "publish_map", "--ros-args",
                           "-p", "pcd_file:=" + str(pcd), "-p", "info_file:=" + str(metadata),
                           "-p", "max_points:=" + str(args.max_points),
                           "-p", "display_quaternion:=" + json.dumps(rotation),
                           "-r", "/lightning/saved_map:=" + topic,
                           "-r", "/lightning/saved_map_tf:=" + frame_topic], profile)
        viewer = None
        try:
            until = time.monotonic() + 60
            while not metadata.exists():
                if publisher.poll() is not None:
                    raise ValueError("Map publisher failed. Check the file and rebuild with scripts/build_robot.sh.")
                if time.monotonic() > until:
                    raise ValueError("Timed out loading the saved map.")
                time.sleep(.1)
            info = json.loads(metadata.read_text())
            config = yaml.safe_load((ROOT / "config/saved_map.rviz").read_text())
            manager = config["Visualization Manager"]
            manager["Displays"][1]["Topic"]["Value"] = topic
            view = manager["Views"]["Current"]
            view["Focal Point"] = dict(zip(["X", "Y", "Z"], info["center"]))
            view["Distance"] = max(5., info["radius"] * 2.5)
            rviz_config = directory / "map.rviz"
            rviz_config.write_text(yaml.safe_dump(config, sort_keys=False))
            viewer = start([RVIZ, "-d", str(rviz_config), "--ros-args",
                            "-r", "/tf:=/lightning/tf", "-r", "/tf_static:=" + frame_topic],
                           profile, viewer=True, cpus=profile.get("rviz_cpus"))
            while viewer.poll() is None:
                if publisher.poll() is not None:
                    raise ValueError("Map publisher exited while RViz was running.")
                time.sleep(.2)
            return viewer.returncode
        except KeyboardInterrupt:
            return 130
        finally:
            if viewer:
                stop(viewer)
            stop(publisher)


def main():
    if len(sys.argv) == 4 and sys.argv[1] == "--settings" and sys.argv[3] == "_probe":
        sensor_probe(load_profile(Path(sys.argv[2])))
        return 0
    if len(sys.argv) == 5 and sys.argv[1] == "--settings" and sys.argv[3] == "_save":
        save_map(Path(sys.argv[2]), sys.argv[4])
        return 0
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--settings", type=Path, default=ROOT / "data/onboard.json")
    commands = parser.add_subparsers(dest="command", required=True)
    config = commands.add_parser("configure", help="save the robot settings once")
    config.add_argument("platform", choices=["agx", "m20"])
    config.add_argument("--driver-workspace", type=str)
    config.add_argument("--driver-model", choices=["mid360", "mid360s"])
    config.add_argument("--config", type=Path, help="copy a calibrated sensor YAML instead of the default")
    config.add_argument("--domain", type=int)
    config.add_argument("--app-cpus")
    config.add_argument("--rviz-cpus")
    commands.add_parser("lidar", help="start the Livox driver unless it is already publishing")
    commands.add_parser("slam", help="start live mapping with RViz outputs")
    loc = commands.add_parser("localize", help="localize against a saved tiled map")
    loc.add_argument("map", type=Path)
    loc.add_argument("--trajectory", type=Path, default=ROOT / "data/localization_onboard.tum")
    save = commands.add_parser("save", help="save the running mapping session")
    save.add_argument("map_id")
    rviz = commands.add_parser("rviz", help="view the live scan/path, or a saved map, over SSH X11")
    rviz.add_argument("--map", type=Path, help="saved map directory or PCD file")
    rviz.add_argument("--max-points", type=int, default=500000)
    alignment = rviz.add_mutually_exclusive_group()
    alignment.add_argument("--original-frame", action="store_true", help="ignore saved-map leveling metadata")
    alignment.add_argument("--up", type=float, nargs=3, help="measured upward direction in an older map's frame")
    commands.add_parser("status", help="check live LiDAR and IMU delivery")
    args = parser.parse_args()
    args.settings = args.settings.expanduser().resolve()
    if args.command == "configure":
        configure(args)
        return 0
    profile = load_profile(args.settings)
    if args.command == "rviz":
        if args.map:
            if not 1 <= args.max_points <= 500000:
                raise ValueError("--max-points must be between 1 and 500000.")
            return view_map(args, profile)
        if args.original_frame or args.up is not None:
            raise ValueError("Saved-map alignment options require --map.")
        return wait(start([RVIZ, "-d", str(ROOT / "config/onboard.rviz"), "--ros-args",
                           "-r", "/tf:=/lightning/tf", "-r", "/tf_static:=/lightning/tf_static"],
                          profile, viewer=True, cpus=profile.get("rviz_cpus")))
    if args.command == "save":
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", args.map_id):
            raise ValueError("Map IDs may contain letters, numbers, underscores, dots and hyphens.")
        return wait(start([sys.executable, str(Path(__file__).resolve()), "--settings", str(args.settings),
                           "_save", args.map_id], profile))
    if args.command == "localize" and not (args.map.expanduser().resolve() / "index.txt").is_file():
        raise ValueError("Localization needs a complete map directory containing index.txt and its tiles.")
    if args.command == "lidar" and profile["platform"] != "agx":
        raise ValueError("M20 uses firmware drivers. Start the NOS point-cloud relay as described in section 6.1.")
    # Keep the driver lock in the supervisor, not the sourced ROS child.
    with (args.settings.parent / ".onboard-driver.lock").open("a") as lock:
        if args.command == "lidar":
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                observed = check_sensors(args, profile)
                if all(observed["counts"].values()):
                    print("LiDAR and internal IMU are already publishing; keep that driver running.")
                    return 0
                raise ValueError("The driver command is already running but both streams are not arriving. "
                                 "Check its terminal and Ethernet configuration.")
        observed = check_sensors(args, profile)
        ready = all(observed["counts"].values())
        if args.command == "status":
            print(json.dumps(observed, indent=2))
            return 0 if ready else 1
        if args.command == "lidar":
            if ready:
                print("LiDAR and internal IMU are already publishing; keep that driver running.")
                return 0
            if any(observed["publishers"]):
                raise ValueError("A sensor publisher already exists but both streams are not arriving. "
                                 "Check the existing driver and Ethernet configuration.")
            model = profile["driver_model"]
            if model not in ("mid360", "mid360s"):
                raise ValueError("Invalid Livox model in settings.")
            launch = "msg_MID360s_launch.py" if model == "mid360s" else "msg_MID360_launch.py"
            return wait(start(["ros2", "launch", "livox_ros_driver2", launch], profile, driver=True))
        if not ready:
            raise ValueError("LiDAR and IMU data are required. Start the sensor driver first "
                             "('python3 scripts/onboard.py lidar' on AGX), then retry.")
        if any(n in observed["nodes"] for n in ("lightning_slam", "lightning_localization")):
            raise ValueError("A Lightning-LM application is already running in this ROS domain. Stop it first.")
        command = ["ros2", "run", "lightning", "run_slam_online" if args.command == "slam" else "run_loc_online",
                   "--config", profile["config"], "--rviz"]
        if args.command == "localize":
            command += ["--map_path", str(args.map.expanduser().resolve()), "--trajectory",
                        str(args.trajectory.expanduser().resolve()), "--", "--ros-args",
                        "-r", "/tf:=/lightning/tf", "-r", "/initialpose:=/lightning/initialpose"]
        if args.command == "localize":
            return wait(start(command, profile, cpus=profile.get("app_cpus")))
        session_file = args.settings.with_suffix(".session.json")
        session = dict(pid=os.getpid(), process_start=process_identity(os.getpid()),
                       imu_reference=observed.get("imu_reference"))
        write_atomic(session_file, json.dumps(session) + "\n")
        if not session["imu_reference"]:
            print("No stable startup gravity reference; saved maps will retain their original view frame.", flush=True)
        try:
            return wait(start(command, profile, cpus=profile.get("app_cpus")))
        finally:
            if mapping_session(args.settings) == session:
                session_file.unlink(missing_ok=True)


if __name__ == "__main__":
    # SSH disconnects and service stops must run the same child cleanup as Ctrl+C.
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGHUP, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    try:
        sys.exit(main())
    except (ValueError, OSError, KeyError, yaml.YAMLError, subprocess.TimeoutExpired) as error:
        print("onboard:", error, file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        sys.exit(130)
