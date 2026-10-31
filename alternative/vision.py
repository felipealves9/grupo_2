#!/usr/bin/env python3

import os
import time
import math  
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool

class MissionVisionNode(Node):
    def __init__(self):
        super().__init__('mission_vision_node')
        self.bridge = CvBridge()
        
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.status_sub = self.create_subscription(Bool, '/mission/scanning', self.status_callback, 10)
        self.sequence_pub = self.create_publisher(String, '/mission/sequence', 10)
        
        self.found_targets = []
        self.mission_completed = False
        self.vision_active = False

        self.color_order = {
            'Vermelho': 1,
            'Verde': 2,
            'Azul': 3
        }
        self.get_logger().info("Visão iniciada. Aguardando o drone chegar no ponto de início da varredura...")

    def status_callback(self, msg):
        if msg.data and not self.vision_active:
            self.get_logger().info("OLHOS ABERTOS! O drone iniciou a varredura limpa na arena.")
        self.vision_active = msg.data

    def image_callback(self, msg):
        if self.mission_completed or not self.vision_active:
            return
            
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Erro na conversão: {e}')
            return

        self.img_height, self.img_width, _ = frame.shape
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # =========================================================================
        # 1. ISOLAMENTO DO AMBIENTE: Encontrar os Círculos Brancos base da arena
        # =========================================================================
        # Branco possui baixa Saturação (S) e alto Brilho/Valor (V)
        lower_white = np.array([0, 0, 150])
        upper_white = np.array([180, 60, 255])
        mask_white = cv2.inRange(hsv, lower_white, upper_white)

        # Encontrar os contornos das áreas brancas
        white_contours, _ = cv2.findContours(mask_white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Criar uma máscara preta do mesmo tamanho do frame original
        roi_mask = np.zeros(frame.shape[:2], dtype=np.uint8)

        # Preencher de branco na máscara vazia apenas os contornos grandes (Círculos de 800mm)
        for cnt in white_contours:
            area = cv2.contourArea(cnt)
            if area > 1000:  # Filtra pequenos ruídos brancos do cenário simulado
                cv2.drawContours(roi_mask, [cnt], -1, 255, -1)

        # Aplicar a máscara no frame HSV original. 
        # Tudo que estiver fora dos círculos brancos se torna preto absoluto (zeros).
        hsv_roi = cv2.bitwise_and(hsv, hsv, mask=roi_mask)

        # =========================================================================
        # 2. BUSCA DOS ALVOS: Procurar cores APENAS dentro da área isolada (ROI)
        # =========================================================================
        mask_red1 = cv2.inRange(hsv_roi, np.array([0, 40, 40]), np.array([10, 255, 255]))
        mask_red2 = cv2.inRange(hsv_roi, np.array([170, 40, 40]), np.array([180, 255, 255]))
        mask_red = mask_red1 | mask_red2
        
        mask_green = cv2.inRange(hsv_roi, np.array([35, 40, 40]), np.array([85, 255, 255]))
        mask_blue = cv2.inRange(hsv_roi, np.array([90, 40, 40]), np.array([150, 255, 255]))

        self.process_color_mask(mask_red, 'Vermelho')
        self.process_color_mask(mask_green, 'Verde')
        self.process_color_mask(mask_blue, 'Azul')

        if len(self.found_targets) == 3:
            self.publish_sequence()
            self.mission_completed = True

    def process_color_mask(self, mask, color_name):
        if any(t['color'] == color_name for t in self.found_targets):
            return

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            
            if area > 150:
                x, y, w, h = cv2.boundingRect(cnt)
                
                if x <= 5 or y <= 5 or (x + w) >= self.img_width - 5 or (y + h) >= self.img_height - 5:
                    continue

                aspect_ratio = float(w) / h
                
                if 0.2 <= aspect_ratio <= 5.0:
                    
                    # Extent Básico (mantido apenas para os prints do log)
                    rect = cv2.minAreaRect(cnt)
                    box_width, box_height = rect[1]
                    box_area = box_width * box_height
                    if box_area == 0:
                        continue
                    extent = area / box_area
                    
                    # Vértices (Epsilon reduzido para 0.01 = 1% do perímetro)
                    # Força as curvas do Círculo a quebrarem em muitos pequenos segmentos, 
                    # enquanto as retas dos polígonos continuam sólidas.
                    peri = cv2.arcLength(cnt, True)
                    approx = cv2.approxPolyDP(cnt, 0.01 * peri, True)
                    num_vertices = len(approx)

                    shape = None
                    
                    # Hierarquia Absoluta
                    if num_vertices <= 4:
                        shape = "Triângulo"      # Padrão é 3 (margem para 4)
                    elif 5 <= num_vertices <= 8:
                        shape = "Hexágono"       # Padrão é 6 (absorve ruídos de serrilhado do Gazebo)
                    else:
                        shape = "Círculo"        # Curvas com epsilon 0.01 gerarão facilmente de 10 a 16 vértices

                    if shape:
                        self.found_targets.append({
                            'shape': shape,
                            'color': color_name,
                            'order': self.color_order[color_name]
                        })
                        
                        self.get_logger().info(f"ALVO CONFIRMADO: {shape} {color_name} (Área: {area:.1f}, Vértices: {num_vertices}, Extent: {extent:.2f})")
                        break

    def publish_sequence(self):
        sorted_targets = sorted(self.found_targets, key=lambda x: x['order'])
        sequence_list = [target['shape'] for target in sorted_targets]
        
        sequence_str = " -> ".join(sequence_list)
        
        msg = String()
        msg.data = sequence_str
        self.sequence_pub.publish(msg)
        
        self.get_logger().info("\n=======================================================")
        self.get_logger().info(f"VARREDURA CONCLUÍDA. SEQUÊNCIA DE ÁUDIO DEVE SER:")
        self.get_logger().info(f"🔊 {sequence_str}")
        self.get_logger().info("=======================================================\n")

        audio_map = {
            "Triângulo": "som1.mp3",
            "Hexágono": "som2.mp3",
            "Círculo": "som3.mp3"
        }

        caminho_base = "/home/avant/simulation/starts/avant_ws/src/pkg_avant/pkg_avant"

        for shape in sequence_list:
            arquivo_audio = audio_map.get(shape)
            if arquivo_audio:
                caminho_completo = os.path.join(caminho_base, arquivo_audio)
                self.get_logger().info(f"🎵 Tocando áudio do {shape} ({arquivo_audio})...")
                
                comando = f"ffmpeg -loglevel quiet -i {caminho_completo} -f wav - | paplay"
                os.system(comando)
                
                time.sleep(1.0)

def main(args=None):
    rclpy.init(args=args)
    node = MissionVisionNode()
    try:    
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
