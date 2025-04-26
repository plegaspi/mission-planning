import paramiko
from getpass import getpass
from Search_Area import generate_mission_from_params
import webbrowser
import os
import datetime
import yaml

# --- Mission Parameters ---
photo_width = 7728
photo_height = 5152
horizontal_fov = 30.49
vertical_fov = 20.47
overlap = 20  # %
flight_altitude = 22.5
is_reversed = False
row_traversal = False

bounds = [
    [21.4000934, -157.7645798],
    [21.4004355, -157.7637832],
    [21.4005653, -157.7638905],
    [21.4002307, -157.7646790]
]

waypoint_file = "waypoints.json"
html_file = "mission_waypoints.html"
mission_planner_file = "mission.waypoints"


generate_mission_from_params(bounds, photo_width, photo_height,
                             horizontal_fov, vertical_fov,
                             overlap, flight_altitude,
                             waypoint_save_path=waypoint_file,
                             html_save_path=html_file,
                             is_reversed=is_reversed, 
                             row_traversal = row_traversal)


print(f"Opening {html_file} for review...")
webbrowser.open(f"file://{os.path.abspath(html_file)}")




while True:
    print("\nChoose an action:")
    print("1 = Upload waypoints.json as")
    print("2 = Reverse waypoint order and upload")
    print("3 = Quit")

    choice = input("Enter your choice (1/2/3): ").strip()

    if choice == "1":
        break
    elif choice == "2":
        print("Re-generating with reversed waypoint order...")
        is_reversed = True
        generate_mission_from_params(bounds, photo_width, photo_height,
                                     horizontal_fov, vertical_fov,
                                     overlap, flight_altitude,
                                     waypoint_save_path=waypoint_file,
                                     html_save_path=html_file,
                                     is_reversed=is_reversed)
        break
    elif choice == "3":
        print("Exiting without uploading.")
        exit(0)
    else:
        print("Invalid input. Try again.")


remote_ip = "192.168.1.50"
remote_user = "uhdt"
waypoint_remote_path = f"/home/uhdt/ws2_livox/waypoints.json"
map_remote_path = f"/home/uhdt/ws2_livox/mission_waypoints.html"
mission_planner_remote_path = f"/home/uhdt/ws2_livox/mission.waypoints"
remote_yaml_files = [
    "/home/uhdt/UHDT-ODCL-2025/config/flight-testing.yaml",
    "/home/uhdt/UHDT-ODCL-2025/config/config.yaml"
]

try:
    while True:
        password = getpass("Enter SSH password: ")

        try:
            ssh = paramiko.SSHClient()
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            ssh.connect(remote_ip, username=remote_user, password=password)
            sftp = ssh.open_sftp()

            print(f"Uploading {waypoint_file} to {remote_ip}:{waypoint_remote_path}")
            sftp.put(waypoint_file, waypoint_remote_path)
            print("Waypoint file uploaded.")

            print(f"Uploading {mission_planner_file} to {remote_ip}:{mission_planner_remote_path}")
            sftp.put(mission_planner_file, mission_planner_remote_path)
            print("Mission planner file is uploaded.")

            print(f"Uploading {html_file} to {remote_ip}:{map_remote_path}")
            sftp.put(html_file, map_remote_path)
            print("Map file is uploaded.")


            local_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            print(f"Setting remote time to: {local_time}")

            command = f'sudo -S date -s "{local_time}"'
            stdin, stdout, stderr = ssh.exec_command(command, get_pty=True)
            stdin.write(password + "\n")
            stdin.flush()

            out = stdout.read().decode().strip()
            err = stderr.read().decode().strip()
            if err and "password" not in err.lower():
                print(f"Time sync stderr: {err}")
            else:
                print("Remote time synchronized.")

            updated_bounds = {
                "bound_1": bounds[0],
                "bound_2": bounds[1],
                "bound_3": bounds[2],
                "bound_4": bounds[3],
            }

            for yaml_file in remote_yaml_files:
                print(f"Updating {yaml_file}...")

                try:

                    with sftp.open(yaml_file, "r") as f:
                        content = f.read().decode()
                    data = yaml.safe_load(content)


                    if "airdrop" not in data:
                        data["airdrop"] = {}
                    if "boundary" not in data["airdrop"]:
                        data["airdrop"]["boundary"] = {}
                    data["airdrop"]["boundary"].update(updated_bounds)


                    updated_yaml = yaml.dump(data, sort_keys=False)
                    temp_path = yaml_file + ".tmp"
                    with sftp.open(temp_path, "w") as f:
                        f.write(updated_yaml)

                    mv_command = f"sudo -S mv {temp_path} {yaml_file}"
                    stdin, stdout, stderr = ssh.exec_command(mv_command, get_pty=True)
                    stdin.write(password + "\n")
                    stdin.flush()

                    mv_out = stdout.read().decode().strip()
                    mv_err = stderr.read().decode().strip()
                    if mv_err and "password" not in mv_err.lower():
                        print(f"mv stderr: {mv_err}")
                    print(f"Updated {yaml_file}")

                except Exception as e:
                    print(f"Failed to update {yaml_file}: {type(e).__name__}: {e}")

            sftp.close()
            ssh.close()
            break

        except Exception as e:
            print(f"Operation failed: {e}")
            print("Try again or press Ctrl+C to cancel.")

except KeyboardInterrupt:
    print("Upload and sync cancelled by user.")