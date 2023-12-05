import bluetooth

# Scan for Bluetooth adapters
def scan_bluetooth_adapters():
    print("Scanning for Bluetooth adapters...")
    try:
        adapters = bluetooth.discover_devices()
        for adapter in adapters:
            print(f"Found adapter: {adapter}")
            # Use the adapter address to connect to specific devices
    except Exception as e:
        print(f"Error scanning for adapters: {e}")

# Connect to an accelerometer sensor
def connect_accelerometer(adapter_address):
    try:
        # Establish socket connection with the accelerometer
        sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        sock.connect((adapter_address, 1))
        print(f"Connected to accelerometer at {adapter_address}")

        # Read accelerometer data
        while True:
            data = sock.recv(1024)
            print(f"Accelerometer data: {data}")
    except Exception as e:
        print(f"Error connecting to accelerometer: {e}")
        sock.close()

if _name_ == "_main_":
    # Scan for Bluetooth adapters
    scan_bluetooth_adapters()

    # Select an adapter to connect to
    adapter_address = input("Enter the adapter address to connect to: ")

    # Connect to the accelerometer sensor
    connect_accelerometer(adapter_address)