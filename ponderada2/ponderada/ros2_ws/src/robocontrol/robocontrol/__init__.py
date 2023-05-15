import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry as Odometry
from time import sleep
from tf_transformations import euler_from_quaternion
import math

# Diferenca maxima possivel de x, y e angulo para ele conseguir se orientar
MAX_DIFF = 0.1

# Posicoes da rota que devem ser seguidas
global posicoesrota 
posicoesrota = [
[1.0, 2.0],
[2.5, 2.0],
[0.5, 0.5],
[2.0, 2.5],
[4.0, 4.0]
]

# classe responsavel por fazer um nodo do rclpy
class TurtleController(Node):
    
    # classe de init definindo atributos necessarios
    def __init__(self, control_period=0.05):
        super().__init__('turtlecontroller')
        self.x_array = 1

        # definindo as classes de odometria para poder utilizá-las
        self.odom = Odometry()
        self.actualpose = self.odom.pose.pose
        self.actualpose_x = self.actualpose.position.x
        self.actualpose_y = self.actualpose.position.y
        # definindo as posicoes atuais
        self.actualpose_x = posicoesrota[(self.x_array - 1)][0]
        self.actualpose_y = posicoesrota[(self.x_array - 1)][1]

        self.setpointodom = Odometry()
        self.setpoint = self.setpointodom.pose.pose
        self.setpoint_x = self.setpoint.position.x
        self.setpoint_y = self.setpoint.position.y
        # definindo as proximas posicoes
        self.setpoint_x = posicoesrota[self.x_array][0]
        self.setpoint_y = posicoesrota[self.x_array][1]
        print(f'self.pose: x={self.actualpose_x}, y={self.actualpose_y}')
        print(f'self.setpoint: x={self.setpoint_x}, y={self.setpoint_y}')
        
        # criando os publicadores, subscribers e controles de tempo
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.pose_callback, 10)
        self.control_timer = self.create_timer(timer_period_sec = control_period, callback = self.control_callback)

    # Criando um callback de posicao para a odometria
    def pose_callback(self, msg):
        x=msg.pose.pose.position.x
        y=msg.pose.pose.position.y
        ang = msg.pose.pose.orientation
        _, _, self.theta = euler_from_quaternion([ang.x, ang.y, ang.z, ang.w]) # Pegando os angulos(orientacao x,y,z e w) e convertendo para angulos de euler
        self.actualpose_x = x
        self.actualpose_y = y
        self.get_logger().info(f"O robô está em x={round(x, 2)}, y={round(y, 2)}, theta={round(math.degrees(self.theta), 2)}")

    # Criando um callback de controle para o robo conseguir se movimentar
    def control_callback(self):
        msg = Twist()
        self.setpoint_x = posicoesrota[self.x_array][0] # Definindo as proximas posicoes que ele deve seguir atraves da nossa tupla da rota
        self.setpoint_y = posicoesrota[self.x_array][1]

        x_diff = self.setpoint_x - self.actualpose_x # Calculando delta x e delta y
        y_diff = self.setpoint_y - self.actualpose_y
        angle_needed = math.atan2(y_diff, x_diff) # Calculando o angulo completo para ir de um ponto a outro utilizando o delta x e o delta y
        theta_diff = angle_needed - self.theta # Descobrindo quanto precisa se mover
        print(f"x_diff={round(abs(x_diff), 2)}, y_diff={round(abs(y_diff), 2)}, theta_diff={round(abs(theta_diff), 2)}")
        
        # Fazendo as verificacoes para sabermos quanto precisamos virar, andar e quando podemos trocar o ponto
        if (abs(x_diff) < MAX_DIFF and abs(y_diff) < MAX_DIFF):
            self.x_array += 1

        if abs(theta_diff) > (MAX_DIFF -0.05):
            msg.linear.x = 0.0
            msg.angular.z = 0.2 if theta_diff > 0 else -0.2
        elif abs(x_diff) > MAX_DIFF:
            msg.linear.x = 0.2
        print(f"self.x_array= {self.x_array}, self.setpoint_x= {posicoesrota[self.x_array][0]}, self.setpoint_y= {posicoesrota[self.x_array][1]}")
        
        # Verificando para quando a nossa rota chegar na ultima posicao, parar todos os movimentos.
        if self.x_array == (len(posicoesrota) - 1):
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.angular.z = 0.0
            self.publisher.publish(msg)
            self.get_logger().info("Cheguei no meu destino.")
            exit()
        self.publisher.publish(msg)
            
# Funcao main que inicia o nodo rclpy que nos permite comunicar-nos dessa forma com o simulador
def main(args=None):
    rclpy.init(args=args)
    tc = TurtleController()
    rclpy.spin(tc)
    tc.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
