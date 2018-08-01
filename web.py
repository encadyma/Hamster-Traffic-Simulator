from flask import Flask, jsonify, json
from HamsterAPI.comm_ble import RobotComm
import time
import sys
from threading import Thread
import logging
import Tkinter as tk

logging.basicConfig(level=logging.DEBUG,format='(%(threadName)-10s) %(message)s',)


class RobotEnum(object):
    LEFT_SIDE = 0
    RIGHT_SIDE = 1

    COLOR_NONE = 0
    COLOR_BLUE = 1
    COLOR_GREEN = 2
    COLOR_CYAN = 3
    COLOR_RED = 4
    COLOR_MAGENTA = 5
    COLOR_YELLOW = 6
    COLOR_WHITE = 7


class RobotBehaviorState:
    def __init__(self, robot):
        self._robot = robot
        self.left_motor = 0
        self.right_motor = 0

        self.left_led = 0
        self.right_led = 0

        self.musical_note = 0

        self.creation_time = time.time()
        self.active = True

        self.tick = 0

        self.c_thread = Thread(name='r.thread '+str(id(self._robot))+' manager', target=self.clock_change)
        self.c_thread.setDaemon(True)
        self.c_thread.start()

        logging.warn('starting new robot ' + str(id(self._robot)))

    def clock_change(self):
        while self._robot and self.active:
            self._robot.set_wheel(0, self.left_motor)
            self._robot.set_wheel(1, self.right_motor)

            self._robot.set_led(0, self.left_led)
            self._robot.set_led(1, self.right_led)

            self._robot.set_musical_note(self.musical_note)

            self.tick += 1
            time.sleep(0.01)

    def to_json(self):
        return {
            "id": str(id(self._robot)),
            "left_motor": self.left_motor,
            "right_motor": self.right_motor,
            "creation_time": self.creation_time,
            "is_active": self.active,
            "tick": self.tick
        }

    def get_id(self):
        return str(id(self._robot))


class WebServer:
    def __init__(self, max_robot_num):
        self.comm = None
        self.server = None
        self.s_thread = None
        self.ul_thread = None

        self.robot_list_raw = []
        self.robot_list = {}

        self.robot_num = 0
        self.max_robot_num = max_robot_num  # Complete maximum that the API can handle is 8

        self.frame = tk.Tk()

        self.status = tk.Text(self.frame)
        self.status.pack()

        btn_exit = tk.Button(self.frame, text="Stop and Exit", command=self.stop_server)
        btn_exit.pack()

        self.init_server()

    def init_server(self):
        self.comm = RobotComm(self.max_robot_num)
        self.server = Flask(__name__)

        @self.server.route('/')
        def ws_status():
            return jsonify({
                'online': True,
                'robots': self.format_list()
            })

        @self.server.route('/post/<robot_id>/<path:cmd_path>')
        def ws_raw_update(robot_id, cmd_path):
            cmds = cmd_path.split('/')

            if robot_id not in self.robot_list or not self.robot_list[robot_id].active:
                return jsonify({'online': True, 'success': False, 'error': 'robot_not_found', 'cmd_path': cmds})

            if cmds[0] == 'l_motor':
                self.set_wheel(0, int(cmds[1]), robot_id=robot_id)
            elif cmds[0] == 'r_motor':
                self.set_wheel(1, int(cmds[1]), robot_id=robot_id)
            elif cmds[0] == 'e_stop':
                self.set_wheel(0, 0, robot_id=robot_id)
                self.set_wheel(1, 0, robot_id=robot_id)
            else:
                return jsonify({'online': True, 'success': False, 'error': 'cmd_not_found', 'cmd_path': cmds})

            return jsonify({
                'online': True,
                'success': True,
                'robots': self.format_list(),
                'cmd_path': cmds
            })

        self.s_thread = Thread(
            name='web server', target=self.server.run,
            kwargs={'debug': True, 'use_reloader': False}
        )
        self.s_thread.setDaemon(True)
        self.ul_thread = Thread(name='list watcher', target=self.update_robot_list)
        self.ul_thread.setDaemon(True)

    def format_list(self):
        new_list = dict()

        for r in self.robot_list:
            new_list[r] = self.robot_list[r].to_json()

        return new_list

    def start_server(self):
        self.log_message('Starting BLE communication thread...')
        self.comm.start()

        self.log_message('Starting web server thread...')
        self.s_thread.start()

        self.log_message('Starting robot list watching thread...')
        self.ul_thread.start()

        self.robot_list_raw = self.comm.robotList

        self.log_message('Starting GUI...')
        self.log_message('You may now connect up to ' + str(self.max_robot_num) + ' robot(s).')
        self.frame.mainloop()

    def log_message(self, log):
        self.status.insert(tk.END, '[' + str(time.time()) + '] ' + log + '\n')

    def stop_server(self):
        self.log_message('Killing the server now..')

        self.comm.stop()
        self.comm.join()

        sys.exit()

    def set_wheel(self, motor_num, speed, robot_num = None, robot_id = None):
        r_id = robot_id if robot_id else self.get_robot_id(robot_num)

        if motor_num == RobotEnum.LEFT_SIDE:
            self.robot_list[r_id].left_motor = speed
        elif motor_num == RobotEnum.RIGHT_SIDE:
            self.robot_list[r_id].right_motor = speed

    def set_led(self, led_num, led_color, robot_num = None, robot_id = None):
        r_id = robot_id if robot_id else self.get_robot_id(robot_num)

        if led_num == RobotEnum.LEFT_SIDE:
            self.robot_list[r_id].left_led = led_color
        elif led_num == RobotEnum.RIGHT_SIDE:
            self.robot_list[r_id].right_led = led_color

    def get_wheel(self, motor_num, robot_num = None, robot_id = None):
        r_id = robot_id if robot_id else self.get_robot_id(robot_num)

        if motor_num == RobotEnum.LEFT_SIDE:
            return self.robot_list[r_id].left_motor
        elif motor_num == RobotEnum.RIGHT_SIDE:
            return self.robot_list[r_id].right_motor
        else:
            return 0

    def get_led(self, led_num, robot_num = None, robot_id = None):
        r_id = robot_id if robot_id else self.get_robot_id(robot_num)

        if led_num == RobotEnum.LEFT_SIDE:
            return self.robot_list[r_id].left_led
        elif led_num == RobotEnum.RIGHT_SIDE:
            return self.robot_list[r_id].right_led
        else:
            return 0

    def get_proximity(self, prox_num, robot_num = None, robot_id = None):
        r_id = robot_id if robot_id else self.get_robot_id(robot_num)
        return self.robot_list[r_id]._robot.get_proximity(prox_num)

    def get_floor(self, floor_num, robot_num = None, robot_id = None):
        r_id = robot_id if robot_id else self.get_robot_id(robot_num)
        return self.robot_list[r_id]._robot.get_floor(floor_num)

    def get_robot_id(self, robot_num):
        return str(id(self.robot_list[robot_num]))

    def update_robot_list(self):
        while True:
            if self.robot_num != len(self.robot_list_raw):
                logging.info('[new change to robot list]')
                self.log_message('list watcher: new number of robots connected (' + str(len(self.robot_list_raw)) + ')')

                robot_visited = set()
                robot_all = set()

                # Recalculate everything
                for robot in self.robot_list_raw:
                    # NOTE: the robot id is based on the
                    # robot object in the communication list
                    robot_visited.add(str(id(robot)))
                    if str(id(robot)) not in self.robot_list:
                        self.robot_list[str(id(robot))] = RobotBehaviorState(robot)

                for robot in self.robot_list:
                    robot_all.add(self.robot_list[robot].get_id())

                robot_inactive = robot_all - robot_visited

                for r_id in robot_inactive:
                    self.robot_list[r_id].active = False

                self.robot_num = len(self.robot_list_raw)

            time.sleep(0.1)


def start():
    max_robot_num = 1
    ws = WebServer(max_robot_num)
    ws.start_server()

    #ws.get_wheel(0, robot_num=0)

    while True:
        pass


if __name__ == "__main__":
    try:
        sys.exit(start())
    except KeyboardInterrupt:
        logging.debug("Interrupt received")
        exit(0)
