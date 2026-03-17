"""
Interactive serial test script for ROSArduinoBridge Arduino firmware.

Use this script to test encoders, servos, IR sensor and drop sequence without ROS.

Examples:
  python test_arduino_bridge.py --port COM3 --baud 57600 --cmd enc
  python test_arduino_bridge.py --port COM3 --baud 57600 --cmd drop
  python test_arduino_bridge.py --port COM3 --baud 57600

If run without --cmd, enters interactive prompt.
"""
import serial
import argparse
import time

PROMPT = "arduino> "

CMD_MAP = {
    'enc': 'e',        # read encoders
    'ir': 'i',         # read IR sensor
    'drop': 'j',       # perform drop
    'servo_write': 's',# servo write: requires args <idx> <pos>
    'servo_read': 't', # servo read: requires arg <idx>
    'pwm': 'o',        # set raw pwm: requires args <p1> <p2>
}


def send_cmd(conn, cmd_str, timeout=1.0):
    # append CR as Arduino uses CR-terminated commands
    cmd = cmd_str + '\r'
    conn.write(cmd.encode('utf-8'))
    # read until CR or LF
    val = ''
    start = time.time()
    while True:
        c = conn.read(1).decode('utf-8')
        if c == '':
            # timeout
            break
        val += c
        if c == '\r' or c == '\n':
            break
        if time.time() - start > timeout:
            break
    return val.strip('\r\n')


def interactive(conn):
    print('Interactive test console. Commands: enc, ir, drop, servo_write <idx> <pos>, servo_read <idx>, pwm <p1> <p2>, exit')
    while True:
        try:
            line = input(PROMPT).strip()
        except (EOFError, KeyboardInterrupt):
            print('\nExiting')
            break
        if not line:
            continue
        if line == 'exit':
            break
        parts = line.split()
        cmd = parts[0]
        if cmd == 'enc':
            resp = send_cmd(conn, 'e')
            print('Encoders:', resp)
        elif cmd == 'ir':
            # ask Arduino to update and print IR
            resp = send_cmd(conn, 'i')
            print('IR:', resp)
        elif cmd == 'drop':
            resp = send_cmd(conn, 'j')
            print('Drop:', resp)
        elif cmd == 'servo_write' and len(parts) >= 3:
            conn_str = f"s {parts[1]} {parts[2]}"
            resp = send_cmd(conn, conn_str)
            print('Servo write:', resp)
        elif cmd == 'servo_read' and len(parts) >= 2:
            conn_str = f"t {parts[1]}"
            resp = send_cmd(conn, conn_str)
            print('Servo read:', resp)
        elif cmd == 'pwm' and len(parts) >= 3:
            conn_str = f"o {parts[1]} {parts[2]}"
            resp = send_cmd(conn, conn_str)
            print('PWM:', resp)
        else:
            print('Unknown or malformed command')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', required=True, help='Serial port (e.g. COM3 or /dev/ttyUSB0)')
    parser.add_argument('--baud', type=int, default=57600)
    parser.add_argument('--cmd', help='Optional single command to run: enc|ir|drop|servo_write|servo_read|pwm')
    parser.add_argument('--args', nargs='*', help='Arguments for the command')
    args = parser.parse_args()

    conn = serial.Serial(args.port, args.baud, timeout=1.0)
    time.sleep(2.0)
    try:
        conn.reset_input_buffer()
    except Exception:
        pass

    if args.cmd:
        if args.cmd == 'enc':
            print(send_cmd(conn, 'e'))
        elif args.cmd == 'ir':
            print(send_cmd(conn, 'i'))
        elif args.cmd == 'drop':
            print(send_cmd(conn, 'j'))
        elif args.cmd == 'servo_write':
            if not args.args or len(args.args) < 2:
                print('servo_write requires two args: idx pos')
            else:
                print(send_cmd(conn, f"s {args.args[0]} {args.args[1]}"))
        elif args.cmd == 'servo_read':
            if not args.args or len(args.args) < 1:
                print('servo_read requires one arg: idx')
            else:
                print(send_cmd(conn, f"t {args.args[0]}"))
        elif args.cmd == 'pwm':
            if not args.args or len(args.args) < 2:
                print('pwm requires two args: p1 p2')
            else:
                print(send_cmd(conn, f"o {args.args[0]} {args.args[1]}"))
        else:
            print('Unknown cmd')
    else:
        interactive(conn)

    conn.close()

if __name__ == '__main__':
    main()
