import bluetooth
import json

other_name = "Quadcopter-Noa"
adress = None
port = 0x1001

nearby_devices = bluetooth.discover_devices(lookup_names=True, duration=1)

print(f"Found devices: {nearby_devices}")
for addr, name in nearby_devices:
    if name == other_name:
        adress = addr


if adress is None:
    print("Failed to connect")
    exit() 

print("found name and adress. Going to connect")

socket = bluetooth.BluetoothSocket()
socket.connect((adress, port))

string = '{"x":-1, "json":{"data":1}, "array":[1,2,3,4,5]}'
json_ = json.loads(string)
json_str = str(json_)

socket.send(json_str)


socket.close()
print("DONE")