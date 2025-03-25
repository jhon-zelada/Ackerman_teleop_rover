import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys
import termios
import tty
import math
import select
import time
import threading

MSG = """
Controla tu ROVER!
---------------------------
Teclas de control:
    W : Avanzar
    X : Retroceder
    S : Detener (velocidad 0 en todo)
    A : Avanzar curvando a la izquierda
    D : Avanzar curvando a la derecha

    Q : Aumenta angulo de reudas
    E : Disminuye angulo de ruedas
    R : Aumentar Velocidad Maxima
    F : Reducir Velocidad Maxima
---------------------------
CTRL+C para salir
---------------------------
"""
moveBindings = {
    'w': (1, 0), # linerar speed , ang_velocity
    'x': (-1,0),
    's': (0, 0),
    'a': (1, 1),
    'd': (1,-1),
}

speedBindings = {
    'q': (1, 1.1),
    'e': (1, 0.9),
    'r': (1.1, 1),
    't': (.9, 1),
}
MAX_VELOCITY = 10.0
MAX_ANGLE_RAD = math.pi/2

class TeleopRover(Node):
    def __init__(self):
        super().__init__('teleop_rover')
        self.position_publisher_ = self.create_publisher(Float64MultiArray, '/ang_position_controller/commands', 10)
        self.velocity_publisher_ = self.create_publisher(Float64MultiArray, '/vel_velocity_controller/commands', 10)

        self.max_velocity_steps = 0.5
        self.angle_step = math.radians(5)

        self.current_velocity = 0.0
        self.current_turn_angle = 0.0
        self.direction = 1

        self.ELL_W = 0.20
        self.ELL_T = 0.32
        self.L1 = 0.20

        self.running = True


    
    def get_key(self, timeout=0.1):
        fd = sys.stdin.fileno()
        #old_settings = termios.tcgetattr(fd)
        settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(fd)
            #termios.tcsetattr(fd, termios.TCSANOW, old_settings)  # Prevents echoing
            rlist, _, _ = select.select([sys.stdin], [], [], timeout)
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = ''
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        finally:
            #termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key




    def compute_wheel_angles_and_velocities(self):
        delta = self.current_turn_angle
        Psi = self.current_velocity * math.tan(delta) / self.L1

        phi_FL = math.atan(2 * self.ELL_W * math.tan(delta) / (2 * self.ELL_W - self.ELL_T * math.tan(delta)))
        phi_FR = math.atan(2 * self.ELL_W * math.tan(delta) / (2 * self.ELL_W + self.ELL_T * math.tan(delta)))
        phi_BL = -phi_FL
        phi_BR = -phi_FR

        v_fi = math.sqrt((self.current_velocity - Psi * self.ELL_T / 2)**2 + (Psi * self.L1)**2)
        v_fo = math.sqrt((self.current_velocity + Psi * self.ELL_T / 2)**2 + (Psi * self.L1)**2)
        v_ci = self.current_velocity - Psi * self.ELL_T / 2
        v_co = self.current_velocity + Psi * self.ELL_T / 2
        v_ri = v_fi
        v_ro = v_fo

        v_fi *= self.direction
        v_fo *= self.direction
        v_ci *= self.direction
        v_co *= self.direction
        v_ri *= self.direction
        v_ro *= self.direction

        return [phi_FL, 0, phi_BL, -phi_FR, 0, -phi_BR], [v_fi, v_ci, v_ri, v_fo, v_co, v_ro]

    def publish_commands(self):
        
        positions, velocities = self.compute_wheel_angles_and_velocities()
        pos_msg = Float64MultiArray()
        pos_msg.data = [float(x) for x in positions]
        self.position_publisher_.publish(pos_msg)

        vel_msg = Float64MultiArray()
        vel_msg.data = [float(x) for x in velocities]
        self.velocity_publisher_.publish(vel_msg)

        time.sleep(0.1)


    def vels(self,speed, turn):
        return 'Actualmente:\t velocidad m/s %.2f\tangulo %.2f ' % (speed, turn)
    
def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def main(args=None):
    settings = saveTerminalSettings()
    rclpy.init(args=args)
    teleop = TeleopRover()
    spinner = threading.Thread(target=rclpy.spin, args=(teleop,))
    spinner.start()
    try:
        print(MSG)
        velocity = MAX_VELOCITY/2
        turn_angle = MAX_ANGLE_RAD/2
        vel_dir = 0
        turn_dir= 0
        while True:
            key = teleop.get_key(timeout=0.1)
            if key in moveBindings.keys():
                vel_dir= moveBindings[key][0]
                turn_dir= moveBindings[key][1]
                teleop.direction = math.copysign(1,vel_dir)
            
            elif key in speedBindings.keys():
                velocity *= speedBindings[key][0]
                velocity = max(min(speedBindings[key][0]*velocity, MAX_VELOCITY),-MAX_VELOCITY)
                turn_angle = max(min(speedBindings[key][1]*turn_angle, MAX_ANGLE_RAD),-MAX_ANGLE_RAD)
                print(f'{teleop.vels(velocity, turn_angle)}')
            else:
                vel_dir = 0.0
                turn_dir = 0.0
                if (key == '\x03'):
                    break

            teleop.current_velocity = abs(vel_dir)*velocity
            teleop.current_turn_angle = turn_dir*turn_angle
            teleop.publish_commands()

    except Exception as e:
        print(e)
    finally:
        teleop.destroy_node()
        rclpy.shutdown()
        spinner.join()

        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()
