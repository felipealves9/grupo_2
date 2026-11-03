#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from geometry_msgs.msg import PoseStamped # De onde ele vai marcar a posição (setpoint_position)
from std_msgs.msg import String 
from std_msgs.msg import Float32MultiArray
from rclpy.qos import qos_profile_sensor_data
import time
class SimpleTakeoffLand(Node):
    def __init__(self): # função inicial
        super().__init__('simple_takeoff_land') # nome do node
        '''
        Soluções:
        @@@@@   Subscribers   @@@@@
            '/mavros/local_position/pose' local exato em que ele está
            '/cmd_vel' alinhamento ordenado pela camera
            '/finish' detectado todas as bases
        @@@@@   Publishers    @@@@@
            '/mavros/setpoint_position/local' publica as posições da varredura
            '/mavros/setpoint_velocity/cmd_vel_unstamped' alinhamento do drone
            '/camera/status' para ligar a câmera e desligar
            '/align/ok' para permitir que ela scaneie a forma q ela quer
        @@@@@   Clients   @@@@@
            '/mavros/set_mode' define 'GUIDED'
            '/mavros/cmd/arming' armamento
            '/mavros/cmd/takeoff' takeoff
            '/mavros/cmd/land' pouso ou landing
        '''
        self.get_logger().info('Esperando conexão com MAVROS...') 

        # Clients
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode') # Set mode (Tem que ir para GUIDED)
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming') # Armar o drone
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff') # Fazer o takeoff
        self.land_client = self.create_client(CommandTOL, '/mavros/cmd/land') # Fazer o pouso

        # Publishers
        self.setpoint_position = self.create_publisher(PoseStamped,'/mavros/setpoint_position/local',qos_profile_sensor_data) 
        self.ativar_camera = self.create_publisher(String,'/camera/status',10)
        self.ok_align = self.create_publisher(String,'/align/ok',10)
        # Subscribers
        self.subscriber = self.create_subscription(PoseStamped,'/mavros/local_position/pose',self.posicao_atual,qos_profile_sensor_data)
        self.create_subscription(Float32MultiArray,'/cmd_vel',self.align,10)
        self.finish_sub = self.create_subscription(String,'/finish',self.finish,10)

        # Aguarda conexão MAVROS
        # Qualquer service pode ser usado para testar a conexão, não necessariamente o arming
        while rclpy.ok() and not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Aguardando serviços do MAVROS...')

        # criando as coordenadas
        self.atual_x = 0.0
        self.atual_y = 0.0
        self.atual_z = 0.0

        self.alvo_x = 0.0
        self.alvo_y = 0.0
        self.alvo_z = 2.0
        # Lista dos caminhos que ele deve fazer
        self.waypoints = [
            [-5.93,3.2,2.0],
            [5.93,3.2,2.0],
            [5.93,1.6,2.0],
            [-5.93,1.6,2.0],
            [-5.93,0.0,2.0],
            [5.93,0.0,2.0],
            [5.93,-1.6,2.0],
            [-5.93,-1.6,2.0],
            [-5.93,-3.2,2.0],
            [5.93,-3.2,2.0],
            [0.0,0.0,2.0]
        ]
        self.current_waypoint_index = 0 # Primeiro sistema de cordenadas que ele publica
        
        # Roda o movimento
        self.get_logger().info('Conexão estabelecida. Iniciando sequência de voo.')


        self.pending_action = 'set_mode'
        self.pending_future = None

        self.align_base = False # define se será alinhado ou não
        
        self.correct_x = 0.0
        self.correct_y = 0.0
        self.correct_z = 0.0

        self.timer = self.create_timer(timer_period_sec=0.1,callback=self.controle_missao) # Execução da missão
    
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
         
    def takeoff(self, altitude=2.0): ## Sai do chao e vai até a altura desejada
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
        mensagem = PoseStamped()
        # mandar as coordenadas em um referencial
        mensagem.header.stamp = self.get_clock().now().to_msg()
        mensagem.header.frame_id = 'map'  # (0,0,0)
        alvo = self.waypoints[self.current_waypoint_index] # o objetivo atual
        self.get_logger().info(f'Indo para {alvo}. {self.current_waypoint_index}/{len(self.waypoints)}')
        
        self.alvo_x = alvo[0]
        self.alvo_y = alvo[1]
        self.alvo_z = alvo[2]

        # Posição onde ele quer chegar
        mensagem.pose.position.x = self.alvo_x
        mensagem.pose.position.y = self.alvo_y
        mensagem.pose.position.z = self.alvo_z
        mensagem.pose.orientation.w = 1.0 # sem rotação (por causa dos quaternions)

        self.setpoint_position.publish(mensagem) # publica onde ele quer chegar
        # Calcular a distância que é aceitavel para concluir um alvo
        distancia_do_alvo = ((self.alvo_x-self.atual_x)**2 + 
                                (self.alvo_y-self.atual_y)**2 + 
                                (self.alvo_z-self.atual_z)**2)**0.5
        if distancia_do_alvo < 0.5:
            if self.atual_z <1.7:
                msg = String()
                msg.data = 'ok'
                self.ok_align.publish(msg)
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

    def align(self, msg_area):
        """Recebe as correções enviadas pelo nó de visão"""
        
        self.y_area = msg_area.data[0] # os eixos estão invertidos 
        self.x_area = msg_area.data[1] # os eixos estão invertidos
        self.color_id = msg_area.data[2] # a cor!!!!
        self.z_area = 2.1 + self.color_id # self color id é um valor entre 0.0 0.1 ou 0.2 -> (8.1)
        if self.x_area == 0.0 and self.y_area == 0.0:
            self.align_base = False
        else:
            ja_existe_cor = any(len(waypoint) >= 3 and waypoint[2] == self.z_area for waypoint in self.waypoints)
            if not ja_existe_cor:
                self.align_base = True
                distancia = ((self.x_area*0.03)**2+(self.y_area*0.03)**2)**0.5
        
                new_waypoint = [self.atual_x+(self.x_area*0.01),self.atual_y+(self.y_area*0.01),self.z_area]
                anterior = self.waypoints[self.current_waypoint_index-1]
                distancia_do_anterior = ((anterior[0]-new_waypoint[0])**2+(anterior[1]-new_waypoint[1])**2)**0.5
                if distancia > 0.3 and distancia_do_anterior > 0.3:
                    self.waypoints.insert(self.current_waypoint_index,new_waypoint)
                    self.get_logger().info('new waypoint...')
                    time.sleep(1)
    def controle_missao(self):
        if self.pending_action == 'set_mode': # ele já começa aqui mesmo
        
            if self.pending_future == None: # None quando não está esperando nenhuma resposta
                self.pending_future = self.set_mode('GUIDED')
            
            if self.pending_future.done(): # o .done() indica se a tarefa parou de executar (obteve resposta)
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
                self.pending_future = self.takeoff(altitude=2.0)
            if self.pending_future.done() and self.atual_z > 1.7:
                self.pending_action = 'road'
                self.pending_future = None
                self.get_logger().info('takeoff [ok]')

        elif self.pending_action == 'road':
            if self.current_waypoint_index > 0:
                ativar_msg = String()
                ativar_msg.data = 'on'
                self.ativar_camera.publish(ativar_msg)
            if self.pending_future == None:
                self.pending_future = self.publicar_objetivo()
            if self.pending_future == 'ok': # o ok é retornado pela função posição quando não há mais waypoints
                self.pending_action = 'landing'
                self.pending_future = None
                self.get_logger().info('road [ok]')
        elif self.pending_action == 'landing':
            ativar_msg = String()
            ativar_msg.data = 'off'
            self.ativar_camera.publish(ativar_msg)
            if self.pending_future == None:
                self.pending_future = self.land()
            if self.pending_future.done():
                self.pending_action = '' # Acabou e então ele não pode para sempre ficar preso nesse loop de mandar que a missão terminou
                self.pending_future = None
                self.get_logger().info('A Missão foi finalizada.')
    def finish(self,msg):
        self.current_waypoint_index = 0
        self.waypoints.clear()
        finish = [0.0,0.0,2.0]
        self.waypoints.append(finish)
        self.destroy_subscription(self.finish_sub)
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