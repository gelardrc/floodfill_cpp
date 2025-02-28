#!/usr/bin/env python3
# -*- coding: utf-8 -*-
## codigo de teste para wavefront ##
import rospy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
import numpy as np
import random
import time
import matplotlib.pyplot as plt
import math
import json
import rospkg
import heapq
from tf.transformations import quaternion_from_euler
import sys

OCCUPIED_THRESHOLD = 50  # Valor mínimo para considerar uma célula ocupada
#BUFFER_RADIUS = 1 # Células ao redor de obstáculos tratadas como ocupadas

class node:
  def __init__(self,pose,peso):
    self.pose = pose
    self.peso = peso
    self.parent = [] ## o parent agora e uma lista
    self.dist = None

def init_nodes(field,init_peso=float('inf')):
  nodes = {}
  all_nodes = {}
  for x_i in range(field.shape[0]):
    for y_i in range(field.shape[1]):
      if field[x_i,y_i] == 0:
        all_nodes[x_i,y_i] =  node(pose=[x_i,y_i],peso=init_peso)

        actual  = node(pose=[x_i,y_i],peso=init_peso)

        nodes[actual.pose[0],actual.pose[1]] = actual


  return all_nodes,nodes

def calculate_orientation(x1, y1, x2, y2):
    """Calcula a orientação (em radianos) entre dois pontos"""
    dx = x2 - x1
    dy = y2 - y1
    angle = math.atan2(dy, dx)  # Calcula o ângulo entre os dois pontos
    return angle

def quaternion_from_yaw(yaw):
    """Converte um ângulo (yaw) em um quaternion"""
    q = quaternion_from_euler(0, 0, yaw)
    return q

def send_msg(path,origin):
    path_msg = Path()
    path_msg.header.frame_id = "map"
    path_msg.header.stamp = rospy.Time.now()
    resolution = 1
    for i in range(len(path)):
        (x, y) = path[i]
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        
        # Converter para coordenadas do mundo real
        pose.pose.position.x = y * resolution + origin.x #+ 0.5  ## tive que inverter aqui nao sei exatamente o pq, mas depois eu vejo
        pose.pose.position.y = x * resolution + origin.y #+ 0.5
        pose.pose.position.z = 0

        # Se não for o último waypoint, calcular a orientação para o próximo ponto
        if i < len(path) - 1:
            next_x, next_y = path[i + 1]
            yaw = calculate_orientation(y, x, next_y, next_x)
            quaternion = quaternion_from_yaw(yaw)
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]
        else:
            # No último waypoint, manter a orientação anterior ou definir sem rotação
            pose.pose.orientation.w = 1.0  # Sem rotação

        path_msg.poses.append(pose)

    if path_msg.poses:
        rospy.loginfo_once("Publishing path with {0} waypoints...".format(len(path_msg.poses)))
        path_pub.publish(path_msg)
    else:
        rospy.logwarn("Path is empty. Nothing to publish.")

def send_start_goal(start, goal, origin):
    # Criando dois marcadores independentes
    start_marker = Marker()
    goal_marker = Marker()

    # Configurações gerais para ambos os marcadores
    for marker in [start_marker, goal_marker]:
        marker.header = Header(frame_id="map")
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.a = 1.0  # Marcador opaco
        marker.pose.orientation = Quaternion(x=0, y=0.7071, z=0, w=0.7071)

    # Configurações específicas do marcador de início
    start_marker.id = 1
    start_marker.color.r = 0.0
    start_marker.color.g = 0.0
    start_marker.color.b = 1.0
    start_marker.pose.position = Point(
        x=start[1] + origin.x + 0.5, # it need to be inverted because of how gazebo works
        y=start[0] + origin.y + 0.5, 
        z=0
    )

    # Configurações específicas do marcador de objetivo
    goal_marker.id = 2
    goal_marker.color.r = 1.0
    goal_marker.color.g = 0.0
    goal_marker.color.b = 0.0
    goal_marker.pose.position = Point(
        x=goal[1] + origin.x + 0.5, 
        y=goal[0] + origin.y + 0.5, 
        z=0
    )

    # Publicando os marcadores
    s_g_pub.publish(start_marker)
    s_g_pub.publish(goal_marker)

def visualize_grid_with_weights(grid_nodes, origin):
    #rospy.loginfo(f"tamanho do nodes {len(grid_nodes)}")
    for node in grid_nodes:
        marker = Marker()
        marker.header = Header(frame_id="map")
        marker.ns = "grid_weights"
        #rospy.loginfo(f"no : {grid_nodes[node[0],node[1]].peso}")
        #marker.id = hash(grid_nodes[node[0],node[1]].peso)  # Garante um ID único baseado na pose
        marker.id = random.randint(0,1000)  # Garante um ID único baseado na pose
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        # Texto com o peso do nó
        marker.text = str(grid_nodes[node[0],node[1]].peso)

        # Define a escala e a cor do texto
        marker.scale.z = 0.5  # Tamanho do texto
        marker.color.a = 1.0  # Opacidade do texto
        marker.color.r = 1.0  # Cor vermelha
        marker.color.g = 1.0  # Cor verde
        marker.color.b = 1.0  # Cor azul

        # Define a posição do texto
        marker.pose.position = Point(
            y=grid_nodes[node[0],node[1]].pose[0] + origin.x + 0.5,
            x=grid_nodes[node[0],node[1]].pose[1] + origin.y + 0.5,
            z=0.1  # Eleva ligeiramente o texto para evitar sobreposição com o grid
        )
        marker.pose.orientation = Quaternion(0, 0, 0, 1)

        # Publica o marcador no tópico de visualização
        grid_pub.publish(marker)

def apply_buffer_to_map(data, buffer):
    """Aplica um buffer ao redor de células ocupadas"""
    buffered_map = np.copy(data)
    height, width = data.shape

    for x in range(height):
        for y in range(width):
            if data[x, y] >= OCCUPIED_THRESHOLD:
                for i in range(-buffer, buffer + 1):
                    for j in range(-buffer, buffer + 1):
                        nx, ny = x + i, y + j
                        if 0 <= nx < height and 0 <= ny < width:
                            buffered_map[nx, ny] = OCCUPIED_THRESHOLD
    return buffered_map

def calculate_angle(p1, p2, p3):
    """
    Calcula o ângulo entre três pontos consecutivos.
    """
    v1 = (p2[0] - p1[0], p2[1] - p1[1])
    v2 = (p3[0] - p2[0], p3[1] - p2[1])
    dot_product = v1[0] * v2[0] + v1[1] * v2[1]
    mag_v1 = math.sqrt(v1[0] ** 2 + v1[1] ** 2)
    mag_v2 = math.sqrt(v2[0] ** 2 + v2[1] ** 2)
    if mag_v1 * mag_v2 == 0:
        return 0
    cos_theta = dot_product / (mag_v1 * mag_v2)
    return math.acos(max(-1, min(1, cos_theta)))

def calculate_path_score(start, goal, path, time, nodes):
    """
    Calcula a pontuação de um caminho com base em overlaps e mudanças de direção.
    
    path: lista de listas [[x1, y1], [x2, y2], ...]
    """
    overlaps = 0
    direction_changes = 0
    visited_points = set()
    
    not_visited = [[x,y] for [x,y] in nodes if [x,y] not in path]
   
    coverage_area = 100 - (100*(len(not_visited)-1))/(len(nodes)) # del start
    
    for i in range(len(path) - 1):
        point = tuple(path[i])  # Converte o ponto para uma tupla para ser armazenado no set
        if point in visited_points:
            overlaps += 1
        visited_points.add(point)

        # Calcula mudanças de direção
        if i < len(path) - 2:
            angle = calculate_angle(path[i], path[i + 1], path[i + 2])
            if angle > math.pi / 4:  # Mudança de direção significativa (> 45 graus)
                direction_changes += 1

    return {
        "start": start,
        "goal": goal,
        "overlaps": overlaps,
        "direction_changes": direction_changes,
        "time": time,
        "coverage_area": coverage_area,
        "score": overlaps + direction_changes  # Você pode ajustar essa fórmula conforme necessário
        
    }

def save_score_to_file(data, filename, start, goal, method):
    rospack = rospkg.RosPack()
    dir = rospack.get_path('wavefront_ros')
    filename= method +"_"+ str(start) + "_" + str(goal) +"_"+  filename
    filename = dir+"/results/"+filename
    with open(filename, "w") as f:
        json.dump(data, f, indent=4)
        rospy.loginfo((f"Dados salvos em {filename}"))

# --- Novo método floodfill ---
def floodfill(grid, start):
    """Executa o FloodFill com backtracking físico explícito.
       Retorna a matriz visited e o caminho percorrido."""
    rows, cols = grid.shape
    visited = np.zeros_like(grid)
    path = [start]  # Caminho físico completo (incluindo backtracking)
    stack = [start]  # Pilha para controle do caminho
    visited[start] = 1

    while stack:
        current = stack[-1]
        x, y = current

        # Gerar direções aleatórias
        directions = [(x+1, y), (x-1, y), (x, y+1), (x, y-1)]
        random.shuffle(directions)

        found = False
        for dx, dy in directions:
            # Verificar se a direção é válida e não visitada
            if 0 <= dx < rows and 0 <= dy < cols:
                if grid[dx, dy] == 0 and visited[dx, dy] == 0:
                    visited[dx, dy] = 1
                    path.append((dx, dy))  # Movimento para frente
                    stack.append((dx, dy))
                    found = True
                    break

        # Backtracking se nenhuma direção válida for encontrada
        if not found:
            if len(stack) > 1:
                # Registrar movimento de volta (backtracking)
                previous = stack[-2]
                path.append(previous)  # Movimento para trás
            stack.pop()

    return visited, path
# --- Fim do novo método floodfill ---

def main(msg):

    #rospy.loginfo(f"\033[93mWavefront Method: {"floodfill"}\033[0m")
    rospy.loginfo(f"Floodill method")
    width = msg.info.width
    height = msg.info.height
    resolution = msg.info.resolution
    origin = msg.info.origin.position
    field = np.array(msg.data).reshape((height, width))
    rospy.loginfo("Map loaded....")
    field = apply_buffer_to_map(field, BUFFER_RADIUS)
    field = np.where(field != 0, 1, 0) ## isso é so pra deixar como na minha implementacao

    all_nodes, nodes = init_nodes(field, init_peso=0) ## nodes are all free nodes in space and all_nodes are all of them
 
    default_goal  = [29,44] # random goal 
    default_start = [34,25] # random start 

    start = eval(rospy.get_param("start", default_start))

    if field[start[0], start[1]] == 1:
      rospy.loginfo(f"Bad start, {start} it's not in free_space")
      while field[start[0], start[1]] == 1:
        rospy.loginfo_once("Searching a valid one..")
        start = [random.randint(0, height-1), random.randint(0, width-1)]
      rospy.loginfo("Changing start for --> {}".format(start))

    
    goal =  eval(rospy.get_param("goal", default_goal))

    if field[goal[0], goal[1]] == 1:
      rospy.loginfo(f"Bad goal, {goal} it's not in free_space")
      while field[goal[0], goal[1]] == 1:
        rospy.loginfo_once("Searching a valid one...")
        goal = [random.randint(0, height-1), random.randint(0, width-1)]
      rospy.loginfo("Changing goal for --> {}".format(goal))

    rospy.loginfo("Start node : [{0},{1}] , goal : [{2},{3}]".format(start[0], start[1], goal[0], goal[1]))

    rospy.loginfo("Starting floodfill ....")

    begin = time.time()

    rospy.loginfo("Finding path ....")
    _,  path = floodfill(field, tuple(start))
    path = [list(t) for t in path] # transform path, a list of tuples, into a list of lists 
    
    
    end = time.time() - begin
    rospy.loginfo("Calculating score ....")
    score = calculate_path_score(start, goal, path, end, nodes)
    save_score_to_file(score, "score.json", start, goal, method="floodfill")
    rospy.loginfo("Path score : {} ".format(score))

    rospy.loginfo("Printing path ....")

    rate = rospy.Rate(1)
    
    while not rospy.is_shutdown():
      send_msg(path, msg.info.origin.position)
      send_start_goal(start, goal, origin)
      visualize_grid_with_weights(nodes, origin)
      rate.sleep()
    
    # print(field,path)

if __name__ == '__main__':
    
    rospy.init_node("floodfill_cpp", anonymous=False)
    node_name = rospy.get_name() + "/"

    BUFFER_RADIUS_DEFAULT = 1
    BUFFER_RADIUS = int(rospy.get_param(node_name + "BUFFER_RADIUS", BUFFER_RADIUS_DEFAULT))
    animated = bool(rospy.get_param(node_name + "animated", False))

    path_pub = rospy.Publisher('/planned_path', Path, queue_size=10)
    s_g_pub = rospy.Publisher('/s_g', Marker, queue_size=10)
    grid_pub = rospy.Publisher("/weigh_grid", Marker)

    rospy.Subscriber("/map", OccupancyGrid, main)

    rospy.spin()