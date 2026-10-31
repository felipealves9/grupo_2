#!/usr/bin/env python3

"""Ponto de partida para o nó do trainee.
Desafio:
  1. Assine /camera/image_raw (sensor_msgs/Image) e /camera/camera_info
     (sensor_msgs/CameraInfo).

  2. Publique em /cmd_vel (geometry_msgs/Twist) as correções de posição
     para aproximar um pouco o drone sobre o alvo.
"""
import os
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo,Image as ROSImage

os.makedirs('image', exist_ok=True)

class TraineeVisionNode(Node):
    """Estrutura inicial: preencha com a sua solução de visão computacional."""

    def __init__(self):
        super().__init__('trainee_vision_node')
        '''
        Soluções:
        Subscriber '/camera/image_raw' para receber a imagem da câmera
        Publisher '/cmd_vel' para Mandar onde o drone deve ir (através do tratamento das imagens da câmera)
        Subscriber '/camera/camera_info' para a calibração da câmera
        '''

        self.align = False
        #subscribers
        self.create_subscription(ROSImage,'/camera/image_raw',self._image_callback,10) # Recebe a imagem da camera
        self.info_sub = self.create_subscription(CameraInfo, '/camera/camera_info', self.info_callback, 10)

        #publishers
        self.align_base_pub = self.create_publisher(Twist,'/cmd_vel',10)
        self.mode_align = self.create_publisher(String,'/align_bool_',10)
        self.create_timer(1.0 / 20.0,self.scan_route)
        self.visao = self.create_publisher(ROSImage,'/camera/edited',10)
        ## IMAGE CALLBACK
        
        self.bridge = CvBridge() # conversor de ros para OpenCV
        
        self.list_ = []
        self.cor_ja_existe = False
        self.forma_ja_existe = False
        self.scan = False
        self.color_done = False
        self.x_middle = 320
        self.y_middle = 240
# -------------------- fonte: https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco/blob/main/ros2_aruco/ros2_aruco/aruco_node.py
    def info_callback(self, info_msg):
        '''
        informações da calibração
        ------------------
        k -> distorção radial
        p -> distorção tangencial
        ------------------
        Ele captura os dados que foram dados pelo simulation_node e depois se destrói.
        '''
        self.info_msg = info_msg # Dados da câmera
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3)) 
        self.distortion = np.array(self.info_msg.d) #captura a distorção da câmera
        # Assume que os parâmetros permanecerão os mesmos
        self.get_logger().info(f'Largura da câmera: {info_msg.width} px')
        self.get_logger().info(f'Altura da câmera: {info_msg.height} px')
        self.get_logger().info(f'Matriz Intrínseca K:\n{self.intrinsic_mat}')
        self.destroy_subscription(self.info_sub) # destroi o subscriber
#---------------------------
    def _image_callback(self,msg):
        '''
        1 - Recebe os frames da câmera (subscriber)
        2 - cria a direção pra ir perto da base
        '''
        # - LER a imagem NA CAMERA
        try:
            #Converter o frame para o OpenCV
            self.frame = self.bridge.imgmsg_to_cv2(msg,'bgr8')
            self.hsvImage = cv2.cvtColor(self.frame,cv2.COLOR_BGR2HSV)
            self.scan = True
        except Exception as e:
            self.get_logger().error('Erro na conversão da imagem')
            return
    def distance_pixel(self):
        '''
        retorna um booleano se a base estiver muito longe
        '''
        if self.x_middle-320 > 200 or self.x_middle-320 < -200: # 60% (por motivo nenhum)
            return True
        elif self.y_middle-240 > 150 or self.y_middle-240 < -150: # 60% (por motivo nenhum)
            return True
        else:
            return False
#--------------------------------
    def align_base(self):
        '''
        Reconhece o ponto central da base e manda pousar nele
        '''
        string_msg = String()
        if self.distance_pixel() == True and self.color_done == True:
            msg = Twist()
            error_x = self.x_middle - 320
            error_y = self.y_middle - 240
            msg.linear.x = -0.0015 * error_x
            msg.linear.y = -0.0015 * error_y
            msg.linear.z = 0.0
            # Correção da publicação da String no ROS 2
            string_msg.data = 'align'
            self.mode_align.publish(string_msg)
            self.align_base_pub.publish(msg)
        else:
            string_msg.data = 'stop'
            self.mode_align.publish(string_msg)
            stop_msg = Twist()
            self.align_base_pub.publish(stop_msg)
    def color(self): # tudo sobre a cor
        '''
        Encontra cores
        (Aqui ele faz todas as máscaras que precisa)
        Ele não pode repetir indo na mesma cor!!!
        '''
        # filtro branco
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 40, 255])
        mask_white = cv2.inRange(self.hsvImage, lower_white, upper_white)

        '''
        cores verde e azul da imagem
        '''

        lowerLimit_blue = np.array([100, 180, 150])
        upperLimit_blue = np.array([140, 255, 255])
        mask_blue = cv2.inRange(self.hsvImage,lowerLimit_blue,upperLimit_blue)
            
        lowerLimit_green = np.array([35, 180, 150])
        upperLimit_green = np.array([85, 255, 255])
        mask_green = cv2.inRange(self.hsvImage,lowerLimit_green,upperLimit_green)
        mask_BG = cv2.bitwise_or(mask_blue,mask_green)

        '''
        existir a cor vermelha(nos dois intervalos)
        '''

        lower_red1 = np.array([0, 180, 150])
        upper_red1 = np.array([10, 255, 255])
        mask_red1 = cv2.inRange(self.hsvImage, lower_red1, upper_red1)
        # Faixa 2 do Vermelho
        lower_red2 = np.array([170, 180, 150])
        upper_red2 = np.array([180, 255, 255])
        mask_red2 = cv2.inRange(self.hsvImage, lower_red2, upper_red2)
        mask_reds = cv2.bitwise_or(mask_red1, mask_red2)
        # une os 3
        mask_colors = cv2.bitwise_or(mask_BG,mask_reds)
        # 3. SUBTRAI O BRANCO

        self.mask_colors_notwhite = cv2.bitwise_and(mask_colors, cv2.bitwise_not(mask_white))
        self.contours, hierarchy = cv2.findContours(self.mask_colors_notwhite, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for i,contour in enumerate(self.contours):
            # Definir onde está para chegar mais perto
            area = cv2.contourArea(contour)
            if area < 20:  # para ignorar ruídos
                continue
            # critérios de determinar um contorno
            epsilon = 0.02 * cv2.arcLength(contour, True)
            self.approx = cv2.approxPolyDP(contour, epsilon, True) # aproximação polinomial
            # delimitar a região:
            x, y, w, h = cv2.boundingRect(self.approx)
            self.x_middle = int(x + w / 2) # imp
            self.y_middle = int(y + h / 2) # imp
            frame_cropped = self.frame[y:y+h, x:x+w]
            hsv_roi = cv2.cvtColor(frame_cropped, cv2.COLOR_BGR2HSV)
            # Encontra a cor
            self.color_id = None
            self.color_name = ""
            if cv2.inRange(hsv_roi, lowerLimit_blue, upperLimit_blue).any():
                self.color_id = 0
                self.color_name = 'azul'
            elif cv2.inRange(hsv_roi, lowerLimit_green, upperLimit_green).any():
                self.color_id = 1
                self.color_name = 'verde'
            elif cv2.inRange(hsv_roi, lower_red1, upper_red1).any() or cv2.inRange(hsv_roi, lower_red2, upper_red2).any():
                self.color_id = 2
                self.color_name = 'vermelho'
            # confere se já viu essa cor antes
            for item in self.list_:
                if item[0] == self.color_id:
                    self.cor_ja_existe = True
                    break
                else:
                    self.cor_ja_existe = False
            if self.color_id is not None and self.cor_ja_existe == False:
                #pode ir atrás
                self.align = True
                self.color_done = True
            else:
                self.align = False
                self.x_middle = 320
                self.y_middle = 240

    def detect_color_shape(self):
        '''
        Faz a leitura da forma e une a informação da forma e da cor
        '''
        vertices = len(self.approx) # quant de vertices
        # Identificação da forma geométrica real de dentro
        if vertices == 3 or vertices == 4:
            form_name = 'triangulo'
            self.form_id = 3
        elif vertices == 6 or vertices == 5:
            form_name = 'Hexagono'
            self.form_id = 6
        else:
            form_name = 'Circulo'
            self.form_id = 8
        for item in self.list_:
            if item[1] == self.form_id:
                self.forma_ja_existe = True
                break
            else:
                self.forma_ja_existe = False
        if self.color_id is not None and self.forma_ja_existe == False:
            #pode anotar e seguir o curso
            self.list_.append([self.color_id,self.form_id])
            self.get_logger().info(f'detectada a forma: {form_name} na cor: {self.color_id}')
            cv2.imwrite(os.path.join('.','image',f'frame_{form_name}_{self.color_name}.jpg'), self.frame)
            self.color_done = False
        else:
            self.get_logger().info('...')

    def scan_route(self):
        '''
        Definir todo o processo de escaneamento
        1 - encontrar o (meio) dos pixels que apresentam a base colorida 
        2 - chegar mais perto
        3 - fazer o scan
        4 - parar de mandar mover
        '''
        try:
            # publicar imagem editada:
            # Exemplo de edição no OpenCV antes de publicar:
            # Desenha um círculo vermelho no ponto central encontrado (x_middle, y_middle)
            cv2.circle(self.frame, (int(self.x_middle), int(self.y_middle)), 10, (0, 0, 255), -1)

            # Desenha o texto do status na imagem
            cv2.putText(self.frame, f"Alinhando: {self.align}", (30, 50), 
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            ros_img_msg = self.bridge.cv2_to_imgmsg(self.frame, encoding='bgr8')
            ros_img_msg.header.stamp = self.get_clock().now().to_msg()
            ros_img_msg.header.frame_id = 'camera_link'  # ou o frame da sua câmera
            self.visao.publish(ros_img_msg)
        except Exception as e:
            self.get_logger().error(f'Erro ao publicar imagem editada: {e}')
        if len(self.list_) < 3 and self.scan == True:
            self.color() # encontrar cores que estão faltando
            self.align_base()
            if self.color_done and self.distance_pixel()==False: # se ele estiver dentro da distancia tolerada:
                self.color()
                self.detect_color_shape()
        elif len(self.list_)==3:
            self.get_logger().info(self.list_)
def main(args=None):
    rclpy.init(args=args)
    node = TraineeVisionNode()
    try:    
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()