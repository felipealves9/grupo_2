#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from geometry_msgs.msg import PoseStamped,Twist # De onde ele vai marcar a posição (setpoint_position)
from std_msgs.msg import String 
from rclpy.qos import qos_profile_sensor_data

class SimpleTakeoffLand(Node):
    def __init__(self): # função inicial
        super().__init__('simple_takeoff_land') # nome do node
        self.get_logger().info('Esperando conexão com MAVROS...') 

        # Clientes para os serviços MAVROS
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode') # Set mode (Tem que ir para GUIDED)
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming') # Armar o drone
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff') # Fazer o takeoff
        self.land_client = self.create_client(CommandTOL, '/mavros/cmd/land') # Fazer o pouso
        # Missão para ele começar a andar
        ## Para onde ir:
        #Criei um publisher que manda onde ele deve ir
        self.setpoint_position = self.create_publisher(PoseStamped,'/mavros/setpoint_position/local',qos_profile_sensor_data) #o qos adapta o tipo de comunicação, ele impediu de dar erro
        # Publisher para enviar a velocidade de correção para o MAVROS:
        self.velocity_pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        self.timer = self.create_timer(timer_period_sec=0.1,callback=self.controle_missao)# Timer de publicação
        ## Onde ele está:
        self.subscriber = self.create_subscription(PoseStamped,'/mavros/local_position/pose',self.posicao_atual,qos_profile_sensor_data)
        self.create_subscription(Twist,'/cmd_vel',self.align,10)
        self.create_subscription(String,'/align_bool_',self.align_bool,10)
        # Aguarda convexão MAVROS
        # Qualquer service pode ser usado para testar a conexão, não necessariamente o arming
        while rclpy.ok() and not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Aguardando serviços do MAVROS...')

        # criando as coordenadas
        self.atual_x = 0.0
        self.atual_y = 0.0
        self.atual_z = 0.0

        self.alvo_x = 0.0
        self.alvo_y = 0.0
        self.alvo_z = 8.0
        # Lista dos caminhos que ele deve fazer
        self.waypoints = [
            [-5.93,3.2,8.0],
            [5.93,3.2,8.0],
            [5.93,1.6,8.0],
            [-5.93,1.6,8.0],
            [-5.93,0.0,8.0],
            [5.93,0.0,8.0],
            [5.93,-1.6,8.0],
            [-5.93,-1.6,8.0],
            [-5.93,-3.2,8.0],
            [5.93,-3.2,8.0],
            [0.0,0.0,8.0]
        ]
        self.current_waypoint_index = 0 # Primeiro sistema de cordenadas que ele publica
        
        # Roda o movimento
        self.get_logger().info('Conexão estabelecida. Iniciando sequência de voo.')


        self.pending_action = 'set_mode'
        self.pending_future = None
        self.align_base = False
        self._correction_velocity = [0.0, 0.0, 0.0]
        
    def set_mode(self, mode_name): # ativa o modo em que ele passa a ser guiado
        self.get_logger().info('set_mode...')
        req = SetMode.Request()
        req.custom_mode = mode_name
        
        return self.set_mode_client.call_async(req) # Retorna que ele mandou para sabermos que foi enviado
        
    def arm(self): # prepara para o takeoff
        self.get_logger().info('arm...')
        req = CommandBool.Request()
        req.value = True
        
        return self.arming_client.call_async(req) # retorna um pedido para armar
         
    def takeoff(self, altitude=8.0): ## Sai do chao e vai até a altura desejada
        self.get_logger().info('takeoff...')
        req = CommandTOL.Request()
        req.altitude = altitude
        self.get_logger().info(f'Decolando para {altitude} metros.')
        
        return self.takeoff_client.call_async(req) # Pedindo o take off
      
    def land(self):  # pousa em baixo dele
        self.get_logger().info('landing...')

        req = CommandTOL.Request()
        
        return self.land_client.call_async(req)

    def publicar_objetivo(self): # onde o Drone tem que ir
        self.get_logger().info('Fazendo o caminho...')
        mensagem = PoseStamped()
        # mandar as coordenadas em um referencial
        mensagem.header.stamp = self.get_clock().now().to_msg()
        mensagem.header.frame_id = 'map'  # (0,0,0)
        # Posição onde ele quer chegar
        mensagem.pose.position.x = self.alvo_x
        mensagem.pose.position.y = self.alvo_y
        mensagem.pose.position.z = self.alvo_z
        mensagem.pose.orientation.w = 1.0 # sem rotação (por causa dos quaternions)

        alvo = self.waypoints[self.current_waypoint_index] # o objetivo atual

        self.alvo_x = alvo[0]
        self.alvo_y = alvo[1]
        self.alvo_z = alvo[2]
        
        self.setpoint_position.publish(mensagem) # publica onde ele quer chegar
        # Calcular a distância que é aceitavel para concluir um alvo
        distancia_do_alvo = ((self.alvo_x-self.atual_x)**2 + 
                             (self.alvo_y-self.atual_y)**2 + 
                             (self.alvo_z-self.atual_z)**2)**0.5
    
        if distancia_do_alvo < 0.1:
            # ELE CHEGOU
            # Se tiver algum objetivo faltando, continuar missão
            if self.current_waypoint_index < len(self.waypoints)-1:
                self.current_waypoint_index = self.current_waypoint_index + 1
                return None
            else: # acabou a missão
                self.get_logger().info('Caminho concluido!')
                return 'ok'
        return None      
        
    def posicao_atual(self,mensagem): # onde o Drone está
        self.atual_x = mensagem.pose.position.x
        self.atual_y = mensagem.pose.position.y
        self.atual_z = mensagem.pose.position.z
    def align_bool(self, msg):
        if msg.data == 'align':
            self.align_base = True
        else: 
            self.align_base = False

    def align(self, msg_twist):
        """Recebe as correções enviadas pelo nó de visão"""
        self._correction_velocity = [
            msg_twist.linear.x, 
            msg_twist.linear.y, 
            msg_twist.linear.z
        ]
    def controle_missao(self):
        if self.pending_action == 'set_mode': # ele já começa aqui mesmo
    # None quando não está esperando nenhuma resposta
            if self.pending_future == None: 
                self.pending_future = self.set_mode('GUIDED')
    # o .done() indica se a tarefa parou de executar (obteve resposta)
            if self.pending_future.done(): 
                self.pending_action = 'arm'
                self.pending_future = None
                self.get_logger().info('set_mode [ok]')

        elif self.pending_action == 'arm':
            if self.pending_future == None:
                self.pending_future = self.arm()
            if self.pending_future.done():
                self.pending_action = 'takeoff'
                self.pending_future = None
                self.get_logger().info('arm [ok]')

        elif self.pending_action == 'takeoff':
            if self.pending_future == None:
                self.pending_future = self.takeoff(altitude=8.0)
            if self.pending_future.done() and self.atual_z > 7.6:
                self.pending_action = 'road'
                self.pending_future = None
                self.get_logger().info('takeoff [ok]')

        elif self.pending_action == 'road':
            # Se a visão detetou o alvo, pausa os waypoints e alinha sobre a base
            if self.align_base: # alinhar
                twist_msg = Twist()
                twist_msg.linear.x = self._correction_velocity[0]
                twist_msg.linear.y = self._correction_velocity[1]
                twist_msg.linear.z = self._correction_velocity[2]
                self.velocity_pub.publish(twist_msg)
                self.get_logger().info('Alinhando...')
            else: 
                # Se NÃO vai alinhar:
                stop_twist = Twist()
                self.velocity_pub.publish(stop_twist)
                # continua  normalmente:
                if self.pending_future == None:
                    self.pending_future = self.publicar_objetivo()
                    
                if self.pending_future == 'ok': # o ok é retornado pela função posição quando não há mais waypoints
                    self.pending_action = 'landing'
                    self.pending_future = None
                    self.get_logger().info('road [ok]')

        elif self.pending_action == 'landing':
            if self.pending_future == None:
                self.pending_future = self.land()
            if self.pending_future.done():
                self.pending_action = '' # Acabou e então ele não pode para sempre ficar preso nesse loop de mandar que a missão terminou
                self.pending_future = None
                self.get_logger().info('A Missão foi finalizada.')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleTakeoffLand()
    try: # Ele fica dando loop
        rclpy.spin(node)
    except KeyboardInterrupt: # menos quando aperta ctrl + c
        node.get_logger().info('CANCELADO!')
        node.land() # Manda pousar (landing)
    finally: # E por final termina de rodar
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()