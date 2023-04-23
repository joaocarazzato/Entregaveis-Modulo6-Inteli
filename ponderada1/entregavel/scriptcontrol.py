import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn, SetPen, Kill
from time import sleep

# Criando uma classe para realziar uma programacao OOP utilizando o nodo do ROS para publicar, enviar, receber, etc
class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        # Criando os publicadores e clientes, isso permite a manipulacao do nodo do turtlesim
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.clientcolor_ = self.create_client(SetPen, 'turtle1/set_pen')
        self.clientspawn_ = self.create_client(Spawn, 'spawn')
        self.clientkill_ = self.create_client(Kill, 'kill')
        self.timer_ = self.create_timer(3, self.move_turtle)
        self.twist_msg_ = Twist()
        self.requestspawn_ = Spawn.Request()
        self.requestcolor_ = SetPen.Request()
        self.requestkill_ = Kill.Request()

    # Funcao para matar a tartaruga
    def kill_turtle(self, name):
        self.requestkill_.name = name

        self.futurekill_ = self.clientkill_.call_async(self.requestkill_)

    # Funcao para trocar a cor da caneta da tartaruga
    def color_turtle(self, r, g, b, width, off, name):
        self.clientcolor_ = self.create_client(SetPen, f'{name}/set_pen')
        self.requestcolor_.r = r
        self.requestcolor_.g = g
        self.requestcolor_.b = b
        self.requestcolor_.width = width
        self.requestcolor_.off = off

        self.futurecolor_ = self.clientcolor_.call_async(self.requestcolor_)

    # Funcao para mover a tartaruga
    def pos_turtle(self, x, y, z, name, hz):
        self.publisher_ = self.create_publisher(Twist, f'{name}/cmd_vel', hz)
        self.twist_msg_.linear.x = x
        self.twist_msg_.linear.y = y
        self.twist_msg_.angular.z = z
        
        self.publisher_.publish(self.twist_msg_)

    # Funcao para criar outra tartaruga
    def spawn_turtle(self, x, y, theta, name):
        self.requestspawn_.x = x
        self.requestspawn_.y = y
        self.requestspawn_.theta = theta
        self.requestspawn_.name = name

        self.futurespawn_ = self.clientspawn_.call_async(self.requestspawn_)

    # Funcao que realiza todos os movimentos necessários para a criacao do desenho 
    def move_turtle(self):
        self.spawn_turtle(0.0, 1.0, 0.0, 'turtle2')
        sleep(1)
        self.color_turtle(0, 0, 255, 100, 0, 'turtle2')
        self.color_turtle(255, 255, 255, 3, 0, 'turtle1')
        sleep(1)
        self.pos_turtle(10.0, 0.0, 0.0, 'turtle2', 100)
        sleep(2)
        self.pos_turtle(0.0, 2.0, 0.0, 'turtle2', 100)
        sleep(2)
        self.pos_turtle(-10.0, 0.0, 0.0, 'turtle2', 100)
        sleep(2)
        self.pos_turtle(0.0, 2.0, 0.0, 'turtle2', 100)
        sleep(2)
        self.pos_turtle(10.0, 0.0, 0.0, 'turtle2', 100)
        sleep(2)
        self.pos_turtle(0.0, 2.0, 0.0, 'turtle2', 100)
        sleep(2)
        self.pos_turtle(-10.0, 0.0, 0.0, 'turtle2', 100)
        sleep(2)
        self.pos_turtle(0.0, 2.0, 0.0, 'turtle2', 100)
        sleep(2)
        self.pos_turtle(10.0, 0.0, 0.0, 'turtle2', 100)
        sleep(2)
        self.pos_turtle(0.0, 2.0, 0.0, 'turtle2', 100)
        sleep(2)
        self.pos_turtle(-10.0, 0.0, 0.0, 'turtle2', 100)
        sleep(2)
        self.kill_turtle('turtle2')
        sleep(1)
        self.pos_turtle(1.0, 0.0, 0.0, 'turtle1', 10)
        sleep(2)
        self.pos_turtle(-2.0, 0.0, 0.0, 'turtle1', 10)
        sleep(2)
        self.color_turtle(255, 0, 0, 3, 0, 'turtle1')
        sleep(2)
        self.pos_turtle(0.5, -2.0, 0.0, 'turtle1', 10)
        sleep(2)
        self.pos_turtle(1.0, 0.0, 0.0, 'turtle1', 10)
        sleep(2)
        self.pos_turtle(0.5, 2.0, 0.0, 'turtle1', 10)
        sleep(2)
        self.color_turtle(255, 255, 255, 3, 0, 'turtle1')
        sleep(2)
        self.pos_turtle(0.2, 0.0, 0.0, 'turtle1', 10)
        sleep(2)
        self.pos_turtle(0.0, 0.2, 0.0, 'turtle1', 10)
        sleep(2)
        self.pos_turtle(-2.4, 0.0, 0.0, 'turtle1', 10)
        sleep(2)
        self.pos_turtle(0.0, -0.2, 0.0, 'turtle1', 10)
        sleep(2)
        self.pos_turtle(0.2, 0.0, 0.0, 'turtle1', 10)
        sleep(2)
        self.pos_turtle(-0.2, 0.0, 0.0, 'turtle1', 10)
        sleep(2)
        self.pos_turtle(0.0, 0.2, 0.0, 'turtle1', 10)
        sleep(2)
        self.pos_turtle(1.1, 0.0, 0.0, 'turtle1', 10)
        sleep(2)
        self.pos_turtle(0.0, 1.0, 0.0, 'turtle1', 10)
        sleep(0.1)
        self.color_turtle(255, 0, 0, 3, 0, 'turtle1')
        sleep(2)
        self.pos_turtle(0.2, 0.0, 0.0, 'turtle1', 10)
        sleep(2)
        self.pos_turtle(0.0, -1.0, 0.0, 'turtle1', 10)
        sleep(2)
        self.color_turtle(255, 255, 255, 3, 0, 'turtle1')
        sleep(2)
        self.pos_turtle(1.1, 0.0, 0.0, 'turtle1', 10)
        sleep(2)
        self.pos_turtle(0.0, -0.2, 0.0, 'turtle1', 10)
        sleep(2)
        self.pos_turtle(-1.2, 0.0, 0.0, 'turtle1', 10)
        sleep(2)
        self.pos_turtle(0.0, -1.0, 0.0, 'turtle1', 10)
        sleep(0.1)
        self.color_turtle(0, 0, 255, 3, 0, 'turtle1')
        sleep(2)
        self.color_turtle(255, 255, 0, 3, 0, 'turtle1')
        sleep(1)
        self.pos_turtle(0.0, 1.0, 3.0, 'turtle1', 10)
        sleep(2)
        self.pos_turtle(0.0, -1.0, -3.0, 'turtle1', 10)
        sleep(2)
        self.pos_turtle(0.0, 1.0, -3.0, 'turtle1', 10)
        sleep(2)
        self.pos_turtle(0.0, -1.0, 3.0, 'turtle1', 10)
        sleep(2)
        self.pos_turtle(0.0, 1.0, 0.0, 'turtle1', 10)
        sleep(0.3)
        self.color_turtle(0, 0, 255, 3, 0, 'turtle1')
        sleep(2)
        self.spawn_turtle(1.5, 10.0, 0.0, 'turtle3')
        self.spawn_turtle(3.0, 10.0, 0.0, 'turtle4')
        self.spawn_turtle(4.5, 10.0, 0.0, 'turtle5')
        self.spawn_turtle(5.0, 10.0, 0.0, 'turtle6')
        self.spawn_turtle(7.0, 9.0, 0.0, 'turtle7')
        self.spawn_turtle(8.0, 9.0, 0.0, 'turtle8')
        self.spawn_turtle(9.0, 10.0, 0.0, 'turtle9')
        self.spawn_turtle(10.0, 10.0, 0.0, 'turtle10')
        sleep(1)
        self.pos_turtle(-1.0, 3.0, 5.5, 'turtle3', 100)
        self.pos_turtle(-1.0, 3.0, 6.0, 'turtle4', 100)
        self.pos_turtle(-1.0, 3.0, 5.5, 'turtle5', 100)
        self.pos_turtle(-0.5, 1.0, -4.0, 'turtle6', 100)
        self.pos_turtle(0.0, 1.5, 0.0, 'turtle7', 100)
        self.pos_turtle(0.0, 1.0, 0.0, 'turtle8', 100)
        self.pos_turtle(0.5, 0.0, 0.0, 'turtle9', 100)
        self.pos_turtle(0.5, 0.0, 0.0, 'turtle10', 100)
        sleep(2)
        self.kill_turtle('turtle3')
        self.kill_turtle('turtle4')
        self.kill_turtle('turtle5')
        self.kill_turtle('turtle8')
        self.pos_turtle(0.0, 0.0, 4.0, 'turtle6', 100)
        self.pos_turtle(1.5, -1.0, -2.0, 'turtle7', 100)
        self.pos_turtle(-1.0, 2.0, 4.5, 'turtle9', 100)
        self.pos_turtle(-0.25, 0.0, 0.0, 'turtle10', 100)
        sleep(2)
        self.kill_turtle('turtle7')
        self.kill_turtle('turtle9')
        self.pos_turtle(0.0, -0.8, 0.0, 'turtle6', 100)
        self.pos_turtle(0.0, 0.25, 0.0, 'turtle10', 100)
        sleep(2)
        self.pos_turtle(0.0, 0.6, 0.0, 'turtle6', 100)
        self.pos_turtle(0.0, -1.0, 0.0, 'turtle10', 100)
        sleep(2)
        self.kill_turtle('turtle10')
        self.pos_turtle(-1.0, 1.0, 5.0, 'turtle6', 100)
        sleep(2)
        self.kill_turtle('turtle6')
        sleep(1)
        

# Funcao para iniciar o script(nodo do ROS(rclpy)) e suas funcoes
def main(args=None):
    rclpy.init(args=args)
    turtle_controller = TurtleController()
    rclpy.spin(turtle_controller)
    rclpy.shutdown()

# Inicia o script
if __name__ == '__main__':
    main()
