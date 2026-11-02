#!/usr/bin/env python3
'''
Node_Vision
'''
import os
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo,Image as ROSImage

os.makedirs('image', exist_ok=True) # diretório com as imagens das formas e cores capturadas

class TraineeVisionNode(Node):
    def __init__(self):
        super().__init__('trainee_vision_node')
        '''
        Soluções:
        @@@@@   Subscribers   @@@@@
            '/camera/image_raw' para receber a imagem da câmera
            '/camera/status' para ligar ou desligar o serviço de scaneamento
            '/align/ok' para permitir a captura
        @@@@@   Publishers    @@@@@
            '/cmd_vel' para Mandar onde o drone deve ir (através do tratamento das imagens da câmera)
            '/finish' detectou todas as bases
        '''
        #subscribers
        self.create_subscription(ROSImage,'/camera/image_raw',self._image_callback,10) # Recebe a imagem da camera
        self.create_subscription(String,'/camera/status',self.status,10)
        self.create_subscription(String,'/align/ok',self.detect_color_shape,10)
        #publishers
        self.align_base_pub = self.create_publisher(Float32MultiArray,'/cmd_vel',10)
        self.finish = self.create_publisher(String,'/finish',10)
        ## IMAGE CALLBACK
        
        self.bridge = CvBridge() # conversor de ros para OpenCV

        self.align = False
        self.list_ = []

        self.cor_ja_existe = False
        self.forma_ja_existe = False

        self.x_middle = 320
        self.y_middle = 240
        self.status_camera = None
        self.create_timer(1.0/20,self.scan_route)

    def status(self,msg):
        self.status_camera = msg.data

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
        except Exception as e:
            self.get_logger().error('Erro na conversão da imagem')
            return

    def distance_pixel(self):
        '''
        retorna um booleano se a base estiver muito longe
        '''
        if self.x_middle-320 > 200 or self.x_middle-320 < -200:
            return False
        elif self.y_middle-240 > 170 or self.y_middle-240 < -170: 
            return False
        else:
            return True

    def align_base(self):
        '''
        Reconhece o ponto central da base e manda pousar nele
        '''
        msg = Float32MultiArray()
        if self.distance_pixel() == False:
            error_x = float(self.x_middle - 320)
            error_y = float(self.y_middle - 240)
            msg.data = [-error_x,-error_y]
            # Correção da publicação da String no ROS 2
            self.align_base_pub.publish(msg)
        else:
            msg.data =[0.0,0.0]
            self.align_base_pub.publish(msg)

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
        mask = cv2.bitwise_or(mask_blue,mask_green)
        mask = cv2.bitwise_or(mask,mask_reds)

        # 3. SUBTRAI O BRANCO
        self.mask_colors_notwhite = cv2.bitwise_and(mask, cv2.bitwise_not(mask_white))
        self.contours, hierarchy = cv2.findContours(self.mask_colors_notwhite, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not self.contours: # se não detectar nada
            msg = Float32MultiArray()
            msg.data = [0.0,0.0]
            self.align_base_pub.publish(msg)
            return

        # como forma de evitar ruído, pegar o maior contorno
        maior_contour = max(self.contours, key=cv2.contourArea)
        area = cv2.contourArea(maior_contour)

        if area < 20:
            msg = Float32MultiArray()
            msg.data = [0.0,0.0]
            self.align_base_pub.publish(msg)
            return

        # critérios de determinar um contorno
        epsilon = 0.01 * cv2.arcLength(maior_contour, True)
        self.approx = cv2.approxPolyDP(maior_contour, epsilon, True) # aproximação polinomial
        
        # delimitar a região:
        x, y, w, h = cv2.boundingRect(self.approx)
        self.x_middle = int(x + w / 2) # imp
        self.y_middle = int(y + h / 2) # imp
        frame_cropped = self.frame[y:y+h, x:x+w]
        hsv_roi = cv2.cvtColor(frame_cropped, cv2.COLOR_BGR2HSV)

        cv2.rectangle(self.frame, (x,y),(x+w,y+h), (255,255,120), 2)

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
        self.cor_ja_existe = any(item[0] == self.color_id for item in self.list_)
        cv2.putText(self.frame,f'{self.color_name}',(x-4,y-4), cv2.FONT_HERSHEY_SIMPLEX,2,(0,0,0),2)
        if self.color_id is not None and self.cor_ja_existe == False:
            self.align_base()

    def detect_color_shape(self,msg):
        '''
        Faz a leitura da forma e une a informação da forma e da cor
        '''
        try:
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

            self.forma_ja_existe = any(item[1] == self.form_id for item in self.list_)

            if not self.forma_ja_existe:
                #pode anotar e seguir o curso
                self.list_.append([self.color_id,self.form_id])
                self.get_logger().info(f'detectada a forma: {form_name} na cor: {self.color_id}')
                cv2.imwrite(os.path.join('.','image',f'frame_{form_name}_{self.color_name}.jpg'), self.frame)
        except Exception as e:
            self.get_logger().info('erro')
    def scan_route(self):
        '''
        Definir todo o processo de escaneamento
        1 - encontrar o (meio) dos pixels que apresentam a base colorida 
        2 - chegar mais perto se tiver necessidade
        3 - fazer o scan
        4 - parar de mandar mover
        '''
        if self.status_camera == 'on':
            cv2.imshow('imagem',self.frame)
            cv2.waitKey(20)
            if len(self.list_) < 3:
                self.color() # encontrar cores que estão faltando
            else:
                msg = String()
                msg.data = ''
                self.finish.publish(msg)
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