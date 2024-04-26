import json, collections, zlib, sys, struct, csv, os, math, argparse, cbor2

parser = argparse.ArgumentParser(
    description="""Convert your Gyroflow stabilization to a CSV or JSON file.
    Make sure to choose "including processed gyro data" when exporting from gyroflow.
    By default it saves out the rotations as Euler rotation (ZYX).
    It also saves out the data in your footages native frame rate."""
)
parser.add_argument("gyroflow_path", help="The path to your Gyroflow file")
parser.add_argument("-j", "--json_export", help="Output a JSON file instead of a CSV",
                    action="store_true")
parser.add_argument("-s", "--smoothed", help="Calculate from smoothed quaternions",
                    action="store_true")
parser.add_argument("-q", "--quaternions", help="Save data as quaternions",
                    action="store_true")
parser.add_argument("-a", "--all_timestamps", help="Save all timestamps instead of FPS converted",
                    action="store_true")
args = parser.parse_args()

# Arg vaiables
gyroflow = args.gyroflow_path
convert_to_euler = True
data_fields = ["timestamp", "x", "y", "z"]
all_timestamps = False

print("[Gyroflow To CSV]")

if args.smoothed:
    quaternion_type = "smoothed_quaternions"
else:
    quaternion_type = "integrated_quaternions"

if args.quaternions:
    convert_to_euler = False
    data_fields.append("w")

if args.all_timestamps:
    all_timestamps = True

if args.json_export:
    json_export = True
else:
    json_export = False
    
# Global variables
base91_alphabet = ["A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M",
    "N", "O", "P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y", "Z",
    "a", "b", "c", "d", "e", "f", "g", "h", "i", "j", "k", "l", "m",
    "n", "o", "p", "q", "r", "s", "t", "u", "v", "w", "x", "y", "z",
    "0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "!", "#", "$",
    "%", "&", "(", ")", "*", "+", ",", ".", "/", ":", ";", "<", "=",
    ">", "?", "@", "[", "]", "^", "_", "`", "{", "|", "}", "~", "\""]

decode_table = dict((v,k) for k,v in enumerate(base91_alphabet))

# Global functions
def decode91(encoded_str):
    ### Decode Base91 string to a bytearray ###
    v = -1
    b = 0
    n = 0
    out = bytearray()
    for strletter in encoded_str:
        if not strletter in decode_table:
            continue
        c = decode_table[strletter]
        if(v < 0):
            v = c
        else:
            v += c*91
            b |= v << n
            n += 13 if (v & 8191)>88 else 14
            while True:
                out += struct.pack("B", b&255)
                b >>= 8
                n -= 8
                if not n>7:
                    break
            v = -1
    if v+1:
        out += struct.pack("B", (b | v << n) & 255 )
    return out

def printProgressBar (iteration, total, prefix = "", suffix = "", decimals = 1, length = 100, fill = "█", printEnd = "\r"):
    """
    Call in a loop to create terminal progress bar
    @params:
        iteration   - Required  : current iteration (Int)
        total       - Required  : total iterations (Int)
        prefix      - Optional  : prefix string (Str)
        suffix      - Optional  : suffix string (Str)
        decimals    - Optional  : positive number of decimals in percent complete (Int)
        length      - Optional  : character length of bar (Int)
        fill        - Optional  : bar fill character (Str)
        printEnd    - Optional  : end character (e.g. "\r", "\r\n") (Str)
    """
    percent = ("{0:." + str(decimals) + "f}").format(100 * (iteration / float(total)))
    filledLength = int(length * iteration // total)
    bar = fill * filledLength + "-" * (length - filledLength)
    print(f"\r{prefix} |{bar}| {percent}% {suffix}", end = printEnd)
    # Print New Line on Complete
    if iteration == total:
        print()

def quaternion_to_euler_angle(x, y, z, w):
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = math.degrees(math.atan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.degrees(math.asin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = math.degrees(math.atan2(t3, t4))

    return [X, Y, Z]

# Closest number in a list
def closest2(lst, K):
    if lst.get(K):
        return (K,) + tuple(lst.get(K))
    else:
        closest_key = min(lst.keys(), key = lambda key: abs(key-K))
        return (closest_key,) + tuple(lst[closest_key])

# Quaternions
def process():
    if not os.path.exists(gyroflow):
        print(gyroflow, "was not found")
        return
    f = open(gyroflow)
    data = json.load(f)
    data_rows = []
    frame_as_microsec = 1 / (data["video_info"]["fps"] / 1000000)

    print("Decompress quaternion")
    try:
        raw = zlib.decompress(decode91(data["gyro_source"][quaternion_type]))
    except Exception as e:
        print("No quaternions found. Make sure you have saved out your Gyroflow file including processed gyro data")
        return
    try:
        synced_imu_timestamps = cbor2._decoder.loads(zlib.decompress(decode91(data["gyro_source"]["synced_imu_timestamps_with_per_frame_offset"])))
    except Exception as e:
        print("No timestamps found. Make sure you have saved out your Gyroflow file including processed gyro data")
        return

    offsets = []
    try:
        offsets = data["offsets"]
        for key, val in offsets.items(): # Convert from milliseconds to microseconds
            offsets[key] = val*1000
    except Exception as e:
        pass

    print("Unpacking")
    gyro_data = cbor2._decoder.loads(raw)

    synced_gyro_data = {}
    for i, (timestamp, val) in enumerate(gyro_data.items()):
        synced_gyro_data[synced_imu_timestamps[i] * 1000] = val

    item_length = len(gyro_data)

    printProgressBar(0, item_length, prefix = "Processing gyro data:", suffix = "Complete", length = 50)
    if all_timestamps:
        for i, (timestamp, val) in enumerate(synced_gyro_data.items()):
            if json_export:
                data = {}
                data["timestamp"] = timestamp
                if convert_to_euler:
                    x, y, z = quaternion_to_euler_angle(val[0], val[1], val[2], val[3])
                    data["x"] = x
                    data["y"] = y
                    data["z"] = z
                    data_rows.append(data)
                else:
                    data["x"] = val[0]
                    data["y"] = val[1]
                    data["z"] = val[2]
                    data["w"] = val[3]
                    data_rows.append(data)
            else:
                data = [timestamp]
                if convert_to_euler:
                    data.extend(quaternion_to_euler_angle(val[0], val[1], val[2], val[3]))
                else:
                    data.extend([val[0], val[1], val[2], val[3]])
                data_rows.append(data)

            printProgressBar(i + 1, item_length, prefix = "Processing gyro datas:", suffix = "Complete", length = 50)
    else:
        num_frames = data["video_info"]["num_frames"]

        for i in range(num_frames):
            cur_microsec = frame_as_microsec * i
            item = closest2(synced_gyro_data, cur_microsec)

            if json_export:
                data = {}
                data["timestamp"] = item[0]
                if convert_to_euler:
                    x, y, z = quaternion_to_euler_angle(item[1], item[2], item[3], item[4])
                    data["x"] = x
                    data["y"] = y
                    data["z"] = z
                    data_rows.append(data)
                else:
                    data.extend([item[1], item[2], item[3], item[4]])
                    data["x"] = item[1]
                    data["y"] = item[2]
                    data["z"] = item[3]
                    data["w"] = item[4]
                    data_rows.append(data)
            else:
                data = [item[0]]
                if convert_to_euler:
                    data.extend(quaternion_to_euler_angle(item[1], item[2], item[3], item[4]))
                else:
                    data.extend([item[1], item[2], item[3], item[4]])
                data_rows.append(data)

            printProgressBar(i + 1, num_frames, prefix = "Processing gyro data:", suffix = "Complete", length = 50)

    if json_export:
        json_file = os.path.split(gyroflow)
        json_file = os.path.abspath(os.path.join(json_file[0], os.path.splitext(json_file[1])[0] + ".json"))
        print("Writing to JSON file:", json_file)
        #print("fields:", data_fields)
        try:
            with open(json_file, "w", encoding = "utf-8") as json_file:
                # writing the data rows
                json.dump(data_rows, json_file, ensure_ascii = True, indent = "\t")
        except OSError as error:
            print("JSON save error:", error)
    else:
        csv_file = os.path.split(gyroflow)
        csv_file = os.path.abspath(os.path.join(csv_file[0], os.path.splitext(csv_file[1])[0] + ".csv"))
        print("Writing to CSV file:", csv_file)
        with open(csv_file, "w", newline="") as csvfile:
            # creating a csv writer object
            csvwriter = csv.writer(csvfile)

            # writing the fields
            csvwriter.writerow(data_fields)

            # writing the data rows
            csvwriter.writerows(data_rows)
    print("Done")
process()
