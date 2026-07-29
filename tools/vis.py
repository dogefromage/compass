import numpy as np
import serial
import time
import viser
from scanf import scanf
import matplotlib.pyplot as plt

DEVICE = "/dev/ttyACM0"
BAUD = 115200


def add_custom_frame(server: viser.ViserServer, color, name):

    frame = server.scene.add_frame(
        f"/{name}",
        show_axes=False,
        origin_radius=0,
    )

    server.scene.add_arrows(
        f"/{name}/xyz",
        colors=color,
        points=np.array(
            [
                [[0, 0, 0], [1, 0, 0]],
                [[0, 0, 0], [0, 1, 0]],
                [[0, 0, 0], [0, 0, 1]],
            ]
        ),
    )

    return frame


def norm_vec(v):
    l = np.linalg.norm(v)
    if l != 0:
        return v / l
    else:
        return v


def run_viser():

    for device in ["/dev/ttyACM0", "/dev/ttyACM1"]:
        try:
            ser = serial.Serial(device, BAUD, timeout=1)
            break
        except Exception as e:
            print(e)

    server = viser.ViserServer()

    handle_body = add_custom_frame(server, (255, 0, 0), "body")
    server.scene.add_grid("/local")

    handle_body_acceleration = server.scene.add_arrows(
        "/body/body_acceleration",
        colors=(0, 150, 150),
        points=[[[0, 0, 0], [0, 0, 0]]],
    )

    handle_local_acceleration = server.scene.add_arrows(
        "/local/local_acceleration",
        colors=(0, 50, 50),
        points=[[[0, 0, 0], [0, 0, 0]]],
        shaft_radius=0.01,
        head_radius=0.03,
    )

    handle_body_field = server.scene.add_arrows(
        "/body/body_field",
        colors=(0, 0, 150),
        points=[[[0, 0, 0], [0, 0, 0]]],
    )

    handle_local_field = server.scene.add_arrows(
        "/local/local_field",
        colors=(0, 0, 50),
        points=[[[0, 0, 0], [0, 0, 0]]],
        shaft_radius=0.01,
        head_radius=0.03,
    )

    while True:

        try:
            line = ser.readline().decode("utf-8").strip()
        except:
            continue

        if line:
            if line.startswith("LOCAL_BODY_TRANSFORM tf_wxyz_xyz"):
                data = scanf("LOCAL_BODY_TRANSFORM tf_wxyz_xyz %f %f %f %f %f %f %f", line)
                if data is None:
                    print(f"WARNING weird format: {line}")
                else:
                    handle_body.wxyz = np.array(data[0:4])

            elif line.startswith("ACC_LOCAL xyz"):
                data = scanf("ACC_LOCAL xyz %f %f %f", line)
                if data is None:
                    print(f"WARNING weird format: {line}")
                else:
                    handle_local_acceleration.points = np.array([[[0, 0, 0], np.array(data) / 10.0]])

            elif line.startswith("ACC_BODY_MEASURED xyz"):
                data = scanf("ACC_BODY_MEASURED xyz %f %f %f", line)
                if data is None:
                    print(f"WARNING weird format: {line}")
                else:
                    handle_body_acceleration.points = np.array([[[0, 0, 0], np.array(data) / 10.0]])

            elif line.startswith("FIELD_LOCAL xyz"):
                data = scanf("FIELD_LOCAL xyz %f %f %f", line)
                if data is None:
                    print(f"WARNING weird format: {line}")
                else:
                    data = norm_vec(data)
                    handle_local_field.points = np.array([[[0, 0, 0], np.array(data)]])

            elif line.startswith("FIELD_BODY_MEASURED xyz"):
                data = scanf("FIELD_BODY_MEASURED xyz %f %f %f", line)
                if data is None:
                    print(f"WARNING weird format: {line}")
                else:
                    data = norm_vec(data)
                    handle_body_field.points = np.array([[[0, 0, 0], np.array(data)]])
            elif line.startswith("FIELD RAW xyz"):
                continue
            else:
                print(">>> " + line)


if __name__ == "__main__":
    run_viser()
