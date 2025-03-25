import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy
from rclpy.qos import QoSProfile

class JoystickRoverControl(Node):
    def __init__(self):
        super().__init__('joystick_rover_control')
        
        # Publicadores para controlar los ángulos (dirección) y la velocidad de las ruedas
        self.position_publisher_ = self.create_publisher(Float64MultiArray, '/ang_position_controller/commands', 10)
        self.velocity_publisher_ = self.create_publisher(Float64MultiArray, '/vel_velocity_controller/commands', 10)
        
        # Suscripción al joystick
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # Variables de control
        self.max_angle_rad = 1.5708  # 90 grados en radianes
        self.min_velocity = -20.0  # Velocidad mínima negativa para retroceder
        self.max_velocity = 20.0  # Velocidad máxima en m/s
        self.velocity_step = 0.4  # Incremento o decremento de la velocidad en m/s por paso
        self.current_velocity = 0.0  # Velocidad inicial

        # Inicializamos las listas de posiciones de ángulos y velocidades
        self.positions = [0.0] * 6  # Angulos de las ruedas
        self.velocities = [self.current_velocity] * 6  # Velocidades de las ruedas

        # QoS Profile para evitar problemas con la tasa de mensajes
        qos_profile = QoSProfile(depth=10)

    def joy_callback(self, msg):
        # Joystick izquierdo para controlar la dirección y velocidad
        left_stick_x = msg.axes[0]   # Eje horizontal del joystick izquierdo (izquierda/derecha)
        left_stick_y = msg.axes[1]   # Eje vertical del joystick izquierdo (arriba/abajo)
        
        # Imprimir los valores del joystick
        self.get_logger().info(f'Joystick izquierdo (horizontal): {left_stick_x}, (vertical): {left_stick_y}')
        
        # Calcular la velocidad en función de la posición del joystick (mayor inclinación => mayor velocidad)
        # La velocidad será proporcional al valor del eje vertical (-1 para retroceder, 1 para avanzar)
        self.current_velocity = left_stick_y * self.max_velocity
        
        # Actualizar las velocidades de las ruedas (mismas velocidades para todas las ruedas)
        self.velocities = [self.current_velocity] * 6

        # Calcular el ángulo de giro de las ruedas basado en el eje horizontal
        # Mapeamos el valor del joystick (-1 a 1) en un rango de -90° a 90° (en radianes)
        turn_angle = left_stick_x * self.max_angle_rad

        # Configurar los ángulos según las reglas descritas
        self.positions[0] = turn_angle  # ang_izq_1
        self.positions[1] = -turn_angle  # ang_izq_2 (gira en sentido contrario)
        self.positions[2] = turn_angle  # ang_izq_3
        self.positions[3] = -turn_angle  # ang_der_1 (gira en sentido contrario)
        self.positions[4] = turn_angle  # ang_der_2
        self.positions[5] = -turn_angle  # ang_der_3 (gira en sentido contrario)

        # Publicar los nuevos valores de ángulo
        pos_msg = Float64MultiArray()
        pos_msg.data = self.positions
        self.position_publisher_.publish(pos_msg)

        # Publicar los nuevos valores de velocidad
        vel_msg = Float64MultiArray()
        vel_msg.data = self.velocities
        self.velocity_publisher_.publish(vel_msg)

        # Mostrar la velocidad actual en el log
        self.get_logger().info(f'Current Velocity: {self.current_velocity} m/s')

def main(args=None):
    rclpy.init(args=args)
    joystick_rover_control = JoystickRoverControl()
    rclpy.spin(joystick_rover_control)
    joystick_rover_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
