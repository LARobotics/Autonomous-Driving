import math
import sim
import time
import datetime
import cv2
import os
import numpy as np
import sys
import rospy
import std_msgs.msg as std_msg
import sensor_msgs.msg as sensor_msg
from cv_bridge import CvBridge
# import torch
# import torchvision
# import PIL.Image
import glob
from typing import Final
# from torchvision import datasets, models, transforms

raio = 0.15     # roda
poss_vel = [10, 12.5, 15, 17.5, 20]
poss_kp = [0.1, 0.111, 0.117, 0.12, 0.12]
vel_init = poss_vel[3]
kp = poss_kp[3]
vel = vel_init
print(vel)
View: Final = True
Print: Final = False
Results: Final = False
Produce_images: Final = False
stop = True
stop1 = False
top = 60
num_lines = 20
line_ang = 3    # verificar e ajustar conforme situação ?
frames_array = []
img_process_time1 = []
img_process_time2 = []
img_process_time3 = []
img_process_time4 = []
coll_count = 0
pos_count = 0
lap = -0.5
lap_time = 0
new_lap = False
j = 0
start = -1
pos_Yant = 0

vel_ant = 0
ang = 0
fps = 0  # so serve para ver quantos frames tenho por segund
fps1 = 0
frames = 0
m1 = -5
m2 = -5
on = False
d_ant = 0
sum_d = 0
turn = 0
obst_pass = False
obst = False
crosswalk = 1
pos_ant = 0
situacao = 0
sit_ant = 1
green_c = 0
p11 = (0, 0)
p12 = (0, 0)
p13 = (0, 0)
p21 = (0, 0)
p22 = (0, 0)
p23 = (0, 0)
p33 = (0, 0)
turn_timer = 0
fr = 1
end = True
end1 = True

frame = 0
new_frame = 0
resolution = 0


def read_light(press, fim, para):
    global crosswalk, turn, situacao, vel
    if crosswalk:
        if press == ord('y'):  # seguir
            print("seguir")
            para = False
        elif press == ord('u'):  # parar
            print("para")
            para = True
        elif press == ord('i'):  # esq
            print("virar esq")
            turn = 1
            situacao = 4
        elif press == ord('o'):  # dir
            print("virar dir")
            turn = 2
            situacao = 4
        elif press == ord('p'):  # estacionar
            print("estacionar")
            para = True
        elif press == ord('t'):  # fim
            print("fim")
            fim = False
    return para, fim


def image_converter(img, rsl):
    img = np.array(img, dtype=np.uint8)
    img.resize([rsl[1], rsl[0], 3])
    img = cv2.flip(img, 0)
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    return img


def init_tracking():
    img = cv2.imread('/home/ines_arib/Uni/simulations/pista2.jpeg')
    track_w, track_h = img.shape[:2]
    track_w = round(track_w / 2.4)
    track_h = round(track_h / 2.4)
    # pixel_x = track_w / 16.7
    # pixel_y = track_h / 6.95
    img = cv2.resize(img, (track_h, track_w))
    pos = (track_h / 2, track_w / 2)

    return pos, img


def get_line(points, view1=1, color=(255, 0, 0)):
    # print("[0, b], [-b/m, 0]", points)
    x_m = 0
    y_m = 0
    sum_num = 0
    sum_den = 0
    m = 0
    p1 = 0
    p2 = 0
    b = -5
    if len(points):
        for i in range(len(points)):
            x_m += points[i][0]
            y_m += points[i][1]
        x_m /= len(points)
        y_m /= len(points)
        for i in range(len(points)):
            sum_num += (points[i][0]-x_m)*(points[i][1]-y_m)
            sum_den += (points[i][0]-x_m)**2
        if sum_den == 0:
            sum_den = 1
        m = sum_num/sum_den
        b = y_m-m*x_m
        if m == 0:
            m = 1

        a = 3
        if 0 <= int(b) < height:
            p1 = (0, int(b))
            a = 0
        elif 0 <= int(-b/m) < width:
            p1 = (int(-b/m), 0)
            a = 1
        elif 0 <= int(m*width+b) < height:
            p1 = (width, int(m*width+b))

        if 0 <= int(-b/m) < width and a < 1:
            p2 = (int(-b/m), 0)
        elif 0 <= int(m*width+b) < height and a < 2:
            p2 = (width, int(m*width+b))
        elif 0 <= int((height-b)/m) <= width:
            p2 = (int((height-b)/m), height)
        if view1:
            cv2.line(image, p1, p2, color, 1)

    return m, b, p1, p2


def ransac(points, rand_lines, side):
    left = 0
    if side == "left":
        left = 1
    index = np.random.rand(rand_lines, 2)
    points_for_lines = []
    index1 = np.empty(shape=[rand_lines, 2])
    calc_lines = []
    count_dots = []
    for i in range(rand_lines):
        for j in range(2):
            index1[i][j] = int(index[i][j]*100 % len(points))
            a = points[int(index1[i][j])]
            points_for_lines.append((a[0], a[1]))

    i = 0
    while i in range(len(points_for_lines)):
        if points_for_lines[i+1] != points_for_lines[i]:
            m, b, q1, q2 = get_line((points_for_lines[i], points_for_lines[i+1]), color=(255, 0, 255), view1=0)
            calc_lines.append([m, b])
            i += 2
        else:
            del points_for_lines[i]
            del points_for_lines[i]

    for q in range(len(calc_lines)):
        count = 0
        for c in range(len(points)):
            xa = (points[c][1] - (calc_lines[q][1] + round(math.cos(calc_lines[q][0]), 3)*5)) / calc_lines[q][0]
            xb = (points[c][1] - (calc_lines[q][1] - round(math.cos(calc_lines[q][0]), 3)*5)) / calc_lines[q][0]
            ya = calc_lines[q][0] * points[c][0] + (calc_lines[q][1] + round(math.cos(calc_lines[q][0]), 3)*5)
            yb = calc_lines[q][0] * points[c][0] + (calc_lines[q][1] - round(math.cos(calc_lines[q][0]), 3)*5)

            if left:
                if xb < points[c][0] < xa and yb < points[c][1] < ya:
                    count += 1
            else:
                if xa < points[c][0] < xb and yb < points[c][1] < ya:
                    count += 1
        count_dots.append(count)
    # print("array:", count_dots)
    final_ind = 0
    for g in range(len(count_dots)):
        if count_dots[final_ind] <= count_dots[g]:
            final_ind = g
    m_final = 0
    b_final = 0
    p1 = (0, 0)
    p2 = (0, 0)
    inc = 0
    if count_dots[final_ind] > 4:
        for g in range(len(count_dots)):
            if count_dots[final_ind]-count_dots[g] < 1:
                m_final += calc_lines[g][0]
                b_final += calc_lines[g][1]
                inc += 1
        m_final /= inc
        b_final /= inc
        # print("Values:", m_final, b_final)
    if (m_final < -0.05-0.07*crosswalk and left) or (not left and m_final > 0.05+0.07*crosswalk):
        _, _, p1, p2 = get_line(([0, b_final], [-b_final/m_final, 0]), color=(255, 0, 0), view1=View)
        if max(p1[1], p2[1]) == p2[1]:
            p3 = p1
            p1 = p2
            p2 = p3
    else:
        m_final = -5
        b_final = -5
    m_final = round(m_final, 4)
    b_final = round(b_final, 4)

    return m_final, b_final, p1, p2


def get_img(msg):
    global frame, resolution, new_frame
    bridge = CvBridge()
    # print(msg.header, "\n", msg.height, msg.width)
    resolution = [msg.width, msg.height]
    frame = cv2.flip(bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough'), 0)
    new_frame = 1
    # cv2.imshow("sensor", frame)


def get_pos(msg):
    global act_pos, last_pos, pos_Yant, lap, pos_count, start, lap_time, tracking_file, new_lap
    pos_x = msg.data[0]
    pos_y = msg.data[1]
    if start == -1:
        pos_Yant = pos_y
        start = time.time()
    if pos_count > 0:
        cv2.circle(tracking, (int(last_pos[0] * 48 + init_pos[0]), int(last_pos[1] * 48 + init_pos[1])), 3,
                   (0, 0, 255), -1)
        act_pos = (np.round(pos_y, 3), np.round(pos_x, 3))
        cv2.circle(tracking, (int(act_pos[0] * 48 + init_pos[0]), int(act_pos[1] * 48 + init_pos[1])), 3,
                   (0, 255, 255), -1)
        pos_count = 0
        last_pos = act_pos
    else:
        pos_count = pos_count+1
        act_pos = (np.round(pos_y, 3), np.round(pos_x, 3))
    if (pos_y > 0 >= pos_Yant) or (pos_y < 0 <= pos_Yant):
        if lap < 0:
            start = time.time()  # começa a contar o tempo
        lap_time = np.round(time.time()-start, 3)
        lap += 0.5
        new_lap = True
    pos_Yant = pos_y


def obst_sensor_detect(msg):
    global obst_pass
    obst_pass = msg.data


def reset_sim(msg):
    global coll_count, lap, lap_time, tracking_file, tracking, dirname, tracking_dir, last_pos, new_lap, start
    if msg.data:
        vel_pub.publish(0)
        turn_pub.publish(0)
        sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot_wait)
        time.sleep(1)
        if Results and lap > 0:
            # if Print:
            print("lap:", lap, "\ttime per lap:", lap_time/lap, "sec")
            tracking_file.write("\nlap: {}\ttime per lap: {} sec\n".format(lap, lap_time/lap))
            cv2.imwrite(os.path.join(tracking_dir, "tracking_{}_{}_{}.jpg".format(dirname, lap, lap_time/lap)), tracking)
        last_pos, tracking = init_tracking()
        coll_count += 1
        lap = -0.5
        start = -1
        sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot_wait)


def create_image_dir():
    a = 1
    dirn = 'lap' + str(a)
    while os.path.exists(dirn):
        a += 1
        dirn = "lap" + str(a)
    print(dirn)
    os.mkdir(dirn)
    return dirn


def ajusta_veloc(press, fim, para):
    global vel
    vel_base = 2.5
    if press == ord('w'):
        vel += vel_base
    elif press == ord('s'):
        vel -= vel_base
    elif press == ord(' '):  # space/emergency stop
        if para:
            para = False
        elif not para:
            para = True
    elif press == 27:
        fim = False

    return para, fim


def verify_lane():

    l_size = 30  # line width
    lane_image = thresh.copy()
    # lane_image = cv2.erode(lane_image, kernel)  # , iterations=2)

    mask_l = np.zeros(lane_image.shape, dtype=np.uint8)
    if m1 != -5:
        corners = np.array([[(p11[0] - l_size, p11[1]), (p11[0] + l_size, p11[1]),
                            (p12[0] + l_size, p12[1]), (p12[0] - l_size, p12[1])]], dtype=np.int32)
        cv2.fillPoly(mask_l, corners, 255)
        left = cv2.bitwise_and(lane_image, mask_l)
    else:
        left = mask_l

    mask_l = np.zeros(lane_image.shape, dtype=np.uint8)
    if m2 != -5:
        corners = np.array([[(p21[0] - l_size, p21[1]), (p21[0] + l_size, p21[1]),
                            (p22[0] + l_size, p22[1]), (p22[0] - l_size, p22[1])]], dtype=np.int32)
        cv2.fillPoly(mask_l, corners, 255)
        right = cv2.bitwise_and(lane_image, mask_l)
    else:
        right = mask_l

    kernel2 = np.ones((3, 3), np.uint8)
    left = cv2.morphologyEx(left, cv2.MORPH_CLOSE, kernel2)
    right = cv2.morphologyEx(right, cv2.MORPH_CLOSE, kernel2)

    # left = cv2.erode(left, kernel2, iterations=2)
    # right = cv2.erode(right, kernel2, iterations=2)

    return left, right


def green_blocks(img):
    green_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    cor_min = np.array([40, 30, 30])
    cor_max = np.array([75, 255, 255])
    green_img = cv2.inRange(green_img, cor_min, cor_max)

    # get center of mass
    countors = cv2.findContours(green_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    mask_g = np.zeros(green_img.shape, dtype=np.uint8)
    return mask_g, green_img, countors


def threshold_white(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    cor_min = np.array([0, 0, 230])
    cor_max = np.array([180, 25, 255])
    gray = cv2.inRange(gray, cor_min, cor_max)

    # CROSSWALK
    edges = cv2.Canny(gray, 50, 150, apertureSize=3)
    lines = cv2.HoughLines(edges, 1, np.pi / 180, 50, 50, 70)

    blur = cv2.GaussianBlur(gray, (3, 3), 0)
    _, final = cv2.threshold(blur, 100, 255, cv2.THRESH_BINARY)
    return final, lines


def get_lane_points(centr):
    global line_ang, height, height2, width, image, thresh, lim_max, lim_min
    save_points = []
    for li in range(int(360 / line_ang)):
        ant = centr
        py = height2 + int(math.sin(math.radians(line_ang * li)) * height)
        if lim_max <= py <= lim_min:
            if crosswalk:
                k = height / 3
            else:
                k = height / 5
            first = 1
            px = using_width + int(math.cos(math.radians(line_ang * li)) * k)
            py = height2 + int(math.sin(math.radians(line_ang * li)) * k)
            ant = [py, px]
            while py in range(top, height) and px in range(width) and first > 0:
                if abs(thresh[int(abs(py)), int(abs(px))] - thresh[ant[0], ant[1]]) > 1:
                    if View:
                        image[int(abs(py)), int(abs(px))] = (0, 255, 0)
                    if crosswalk:
                        save_points.append([px, py])
                    elif situacao == 1:
                        first += 1
                        if first == 3:
                            save_points.append([px, py])
                            first = 0
                    elif situacao == 2:
                        save_points.append([px, py])
                        first += 1
                        if first == 3:
                            first = 0
                    else:
                        save_points.append([px, py])
                        first = 0
                else:
                    if View:
                        image[int(abs(py)), int(abs(px))] = (0, 0, 255)
                ant = [py, px]
                px = using_width + int(math.cos(math.radians(line_ang * li)) * k)
                py = height2 + int(math.sin(math.radians(line_ang * li)) * k)
                k += 1
    return save_points


def error_distribution(y_def, slope1, slope2, y_intr1, y_intr2, goal_point):
    global using_height
    if slope1 != -5:
        x_1 = (y_def - y_intr1) / slope1
    else:
        x_1 = 0
    if slope2 != -5:
        x_2 = (y_def - y_intr2) / slope2
    else:
        x_2 = width
    dist_middle = 70
    if x_1 == 0 and x_2 == width:
        if d_ant < 0:
            dist = d_ant + 10
        elif d_ant > 0:
            dist = d_ant - 10
        else:
            dist = 0
        if (d_ant < 0 < dist) or (d_ant > 0 > dist):
            dist = 0
        dist = using_width - dist
    elif x_2 == width:
        dist = x_1 + dist_middle
    elif x_1 == 0:
        dist = x_2 - dist_middle
    else:
        dist = ((max(x_1, x_2) - min(x_1, x_2)) / 2) + min(x_1, x_2)
        goal_point = (int((y_intr2 - y_intr1) / (m1 - m2)), int(m1 * ((y_intr2 - y_intr1) / (m1 - m2)) + y_intr1))

    if situacao == 2:
        slope2 = -5
    if situacao == 5:
        slope1 = -5
    if goal_point[1] > y_def:
        dist = goal_point[0]
    # print(goal_point, y_def)
    dist = int(dist)
    return x_1, x_2, slope1, slope2, dist, goal_point


def turn_left(wid, dist, change_dirc, n_vel, ant_vel, sit, timer_turn):  # last_p, act_p,
    if time.time() - timer_turn > 4:  # abs(last_p[1] - act_p[1]) > 2 or last_p[0] - act_p[0] > 1:
        change_dirc = 0
        n_vel = ant_vel
        sit = 0
        timer_turn = 0
    elif time.time() - timer_turn > 1.5:  # if last_p[0] - act_p[0] < -1.6 and abs(last_p[1] - act_p[1]) < 0.8:
        dist = wid / 4 - 10
        dist = int(dist)
    return dist, change_dirc, n_vel, sit, timer_turn


def turn_right(wid, dist, change_dirc, n_vel, ant_vel, sit, timer_turn):   # last_p, act_p,
    if time.time() - timer_turn > 2:  # abs(last_p[1] - act_p[1]) > 1 or last_p[0] - act_p[0] < -1:
        change_dirc = 0
        n_vel = ant_vel
        sit = 0
        timer_turn = 0
    elif time.time() - timer_turn > 1:   # last_p[0] - act_p[0] > 1.1 and abs(last_p[1] - act_p[1]) < 1:
        dist = 4 * wid / 5
        dist = int(dist)

    return dist, change_dirc, n_vel, sit, timer_turn


def do_turn(wid, dist, change_dirc, ant_vel):
    global vel, act_pos, last_pos, situacao, turn_timer
    if crosswalk == 1:
        ant_vel = vel
        turn_timer = time.time()
        last_pos = act_pos
    else:
        vel = 5.0
        # print(time.time() - turn_timer)
    if change_dirc == 1:
        dist, change_dirc, vel, sit, turn_timer = turn_left(wid, dist, change_dirc, vel, ant_vel, situacao, turn_timer)
        # last_pos, act_pos,
    elif change_dirc == 2:
        dist, change_dirc, vel, sit, turn_timer = turn_right(wid, dist, change_dirc, vel, ant_vel, situacao, turn_timer)
        # last_pos, act_pos,

    return dist, change_dirc, ant_vel


def get_green_center(img):
    global obst
    moment = cv2.moments(img)
    cx = int(moment["m10"] / moment["m00"])
    cy = int(moment["m01"] / moment["m00"])
    if m1 != -5 and m2 != -5:
        if cy > m1 * cx + b1 and cy > m2 * cx + b2:
            obst = True
        else:
            obst = False

    elif m2 == -5:
        if cy > m1 * cx + b1:
            obst = True
        else:
            obst = False
    elif m1 == -5:
        if cy > m2 * cx + b2:
            obst = True
        else:
            obst = False
    return cy, cx


# def network_init():
#    global device, model
#    model.fc = torch.nn.Linear(512, 4)
#    model = model.to(device)
#    model.load_state_dict(torch.load('models/model3_20ep_8bs.pth', map_location=device))
#    return model
#
#
# def preprocess_img(img):
#    global device
#    cv2.resize(img, (224, 224))
#    mean = torch.Tensor([0.485, 0.456, 0.406]).cpu()
#    std = torch.Tensor([0.229, 0.224, 0.225]).cpu()
#    img = PIL.Image.fromarray(img)
#    img = transforms.functional.to_tensor(img).to(device)
#    img.sub_(mean[:, None, None]).div_(std[:, None, None])
#    return img[None, ...]
#
#
# def network_result(img):
#    img = preprocess_img(img)
#    output = model(img).detach().cpu().numpy().flatten()
#    x_ang = output[0]
#    y_vel = output[1]
#    x_ang = int(224 * (x_ang / 2.0 + 0.5)) / 10
#    y_vel = int(224 * (y_vel / 2.0 + 0.5)) / 10
#    return x_ang, y_vel


fourcc = cv2.VideoWriter_fourcc(*'MPEG')
out = cv2.VideoWriter('output.avi', fourcc, 20.0, (512, 256))  # (width, height)

sim.simxFinish(-1)   # just in case, close all opened connections
clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)    # Connect to CoppeliaSim

if clientID != -1:  # verifica que se conseguiu ligar a simulação
    sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot_wait)
    time.sleep(1)
    sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot_wait)

init_pos, tracking = init_tracking()
act_pos = init_pos
last_pos = init_pos
dirname = create_image_dir()

# cv2.circle(tracking, (int(-2.8 * 58 + init_pos[0]), int(1 * 58 + init_pos[1])), 2, (0, 0, 255), -1)
# cv2.circle(tracking, (int(-2.8 * 58 + init_pos[0]), int(0.3 * 58 + init_pos[1])), 2, (0, 0, 255), -1)
# cv2.circle(tracking, (int(-3.6 * 58 + init_pos[0]), int(1 * 58 + init_pos[1])), 2, (0, 0, 255), -1)
# cv2.circle(tracking, (int(-3.6 * 58 + init_pos[0]), int(0.3 * 58 + init_pos[1])), 2, (0, 0, 255), -1)

# cv2.circle(tracking, (int(2.8 * 58 + init_pos[0]), int(0.8 * 58 + init_pos[1])), 2, (0, 255, 255), -1)
# cv2.circle(tracking, (int(2.8 * 58 + init_pos[0]), int(0.3 * 58 + init_pos[1])), 2, (0, 255, 255), -1)
# cv2.circle(tracking, (int(2 * 58 + init_pos[0]), int(0.8 * 58 + init_pos[1])), 2, (0, 255, 255), -1)
# cv2.circle(tracking, (int(2 * 58 + init_pos[0]), int(0.3 * 58 + init_pos[1])), 2, (0, 255, 255), -1)

if Results:
    tracking_dir = "tracking_" + str(vel_init)
    print(tracking_dir)
    tracking_file = open('tracking_{}.txt'.format(vel_init), 'a')
    tracking_file.write("\n________________  {}    -    {}  __________________\n".format(dirname, datetime.datetime.now().strftime("%I:%M%p, %B %d, %Y")))

out2 = cv2.VideoWriter('output2.avi', fourcc, 20.0, (512, 256))
rospy.init_node('vision_control')
vel_pub = rospy.Publisher('/velocity_Data', std_msg.Float32, queue_size=2)
turn_pub = rospy.Publisher('/turn_Data', std_msg.Float32, queue_size=2)
end_sim = rospy.Publisher('/end', std_msg.Bool, queue_size=1)
camera_sub = rospy.Subscriber('/camera_Data', sensor_msg.Image, get_img)
collis_sub = rospy.Subscriber('/collision_Data', std_msg.Bool, reset_sim)
posit_sub = rospy.Subscriber('/position_Data', std_msg.Float32MultiArray, get_pos)
obstac_sub = rospy.Subscriber('/object_Data', std_msg.Bool, obst_sensor_detect)
r = rospy.Rate(10)  # 10hz

print("start")
# start = time.time()  # começa a contar o tempo
kernel = np.ones((3, 3), np.uint8)
while not new_frame:
    time.sleep(0.1)

# fr_time = time.time()
# fr2_time = time.time()
new_frame = 0
width, height = resolution
using_height = int(height / 2)
using_width = int(width / 2)
height2 = height - 1
center = [height2, using_width]
lim_min = height
lim_max = height / 10

while not rospy.is_shutdown():  # or thresh[height-1, using_width]==0:#end:
    if new_frame:
        # fr2_time = time.time()
        new_frame = 0
        image = frame.copy()
        image2 = frame.copy()

        #      THRESHOLD
        thresh, line_count = threshold_white(image)
        # erode = cv2.erode(thresh, kernel)  # , iterations=2)

        # VERIFY LANE
        left_half, right_half = verify_lane()
        # cv2.imshow("left", left_half)
        # cv2.imshow("right", right_half)
        # left_c = cv2.countNonZero(left_half)
        # right_c = cv2.countNonZero(right_half)
        contours_left, _ = cv2.findContours(left_half, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        contours_right, _ = cv2.findContours(right_half, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        thresh = cv2.dilate(thresh, kernel)  # , iterations=2)
        white_c = cv2.countNonZero(thresh)

        # VERIFY GREEN BLOCKS
        mask, green, cnts = green_blocks(image)

        # Definição estrategia
        if situacao == 1 and 1.6 < act_pos[1]:
            situacao = 1.1
            top = using_height - 30
        elif situacao == 1.1 and abs(ang) < 12 and m1 != -5 and m2 != -5:
            situacao = 0
            top = 60
        elif len(line_count) > 80 and situacao != 1:    # and situacao != 1.1:
            crosswalk = 1
            on = False
            # top = 60
            situacao = 0
            mask1 = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel=np.ones((3, 80)), iterations=2)
            thresh = cv2.subtract(thresh, mask1)
        elif 1 > act_pos[1] > 0.4 and 2.8 < abs(act_pos[0]) < 3.6 and turn != 1:  # CROSS = CHANGE LANE
            situacao = 1
            crosswalk = 0
            top = using_height - 20
        elif 0.8 > act_pos[1] > 0.3 and 2 < abs(act_pos[0]) < 2.8 and turn != 2:
            # situacao = 1.1
            top = using_height - 20
        elif green_c > 1500 and situacao != 4 and obst:
            situacao = 2
        elif crosswalk == 1 and white_c > 11000:
            situacao = 0
            crosswalk = 1
            top = 60
            mask1 = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel=np.ones((3, 80)), iterations=2)
            thresh = cv2.subtract(thresh, mask1)
            # get next step from signal
        else:
            crosswalk = 0
            if vel != vel_ant and vel_ant != 0:
                vel = vel_init

        # cv2.imshow("thresh", thresh)
        # if situacao != sit_ant:
        #    print(situacao)
        #    sit_ant = situacao

        point_arr = get_lane_points(center)

        #       SEPARATE LEFT POINTS FROM RIGHT POINTS
        points_left = []
        points_right = []
        for p in range(len(point_arr)):
            if point_arr[p][0] < using_width:
                points_left.append(point_arr[p])
            elif point_arr[p][0] > using_width:
                points_right.append(point_arr[p])

        #       GET SLOPE LINES
        m1 = -5  # CLEAN VALUES *JUST IN CASE*
        m2 = -5
        b1 = -5
        b2 = -5
        d = 0

        # img_process_time1.append(time.time() - fr2_time)
        if len(points_left) > 2:
            m1, b1, p11, p12 = ransac(points_left, num_lines, "left")
        if len(points_right) > 2:
            m2, b2, p21, p22 = ransac(points_right, num_lines, "right")
        # img_process_time2.append(time.time() - fr2_time)

        if m1 == -5:
            p11 = (0, height)
            p13 = (0, 0)
            p12 = (0, 0)
            p33 = p22
        if m2 == -5:
            p22 = (width, 0)
            p23 = (width, 0)
            p21 = (width, height)
            p33 = p12

        y = using_height - 60
        if situacao == 5:
            m1 = -5
            b1 = -5
        x1, x2, m1, m2, d, p33 = error_distribution(y, m1, m2, b1, b2, p33)
        if View:
            cv2.circle(image, (d, y), 5, (0, 255, 255))

        # if (d_ant < 0 and d > 0) or (d_ant > 0 and d < 0) or vel == 0:
        #     sum_d = 0
        p13 = (int(x1), y)
        p23 = (int(x2), y)
        # if Print:
        #    print(p11, p13, p12, p21, p23, p22, p33)
        if View:
            cv2.line(image, p13, p23, (0, 255, 0))
        # sum_d += d

        roi_corners = np.array([[(0, height), p11, p33, p21, (width, height)]], dtype=np.int32)
        cv2.fillPoly(mask, roi_corners, 255)
        # cv2.imshow('green_lim', mask)  # OBSTACLE
        crop_image = cv2.bitwise_and(green, mask)
        green_c = cv2.countNonZero(crop_image)

        if (len(contours_left) == 1 and len(contours_right) > 1 and green_c < 100 and not obst_pass and not crosswalk and situacao == 0) or situacao == 5:
            if not on:
                vel_ant = vel
                print("wrong lane ?")
                on = True
            else:
                print(len(contours_left), m1, b1, "\t", len(contours_right), m2, b2)
                situacao = 5
                vel = vel_init*0.6
                d = d - x1
                d = d + x2 + 20
                d = int(d)
                if View:
                    cv2.circle(image, (d, y), 5, (255, 0, 255))
            if len(contours_right) < 1 and b1 == -5:
                vel = vel_ant
                situacao = 0
                on = False
        else:
            on = False

        if turn > 0:  # turn right or left by order of light
            d, turn, vel_ant = do_turn(width, d, turn, vel_ant)
            if View:
                cv2.circle(image, (d, y), 5, (255, 0, 255))

        if View:
            cv2.circle(image, p33, 5, (0, 0, 255))  # vermelho
            # cv2.circle(image, p11, 5, (0, 255, 255))  # amarelo
            # cv2.circle(image, p13, 5, (0, 0, 255))      # vermelho
            # cv2.circle(image, p12, 5, (0, 255, 0))      # verde
            # cv2.circle(image, p22, 5, (255, 0, 0))    # azul
            # cv2.circle(image, p23, 5, (255, 255, 0))    # azul claro
            # cv2.circle(image, p21, 5, (255, 0, 255))  # rosa

        # Verify block in lane

        # Actuate in case of obstacle
        if green_c:
            c_y, c_x = get_green_center(green)
            if obst:
                cv2.circle(image, (c_x, c_y), 2, (0, 255, 0), 1)
            else:
                cv2.circle(image, (c_x, c_y), 2, (0, 0, 255), 1)
            cv2.imshow('green', crop_image)  # OBSTACLE

        if situacao == 2 and obst is True and not crosswalk and turn == 0:
            vel = vel_init*0.6
            d = d - x1
            d = x1 - d - 20
            d = int(d)
            if View:
                cv2.circle(image, (d, y), 5, (255, 0, 255))
            if green_c == 0 or obst_pass:
                situacao = 0
                cv2.destroyWindow("green")
                vel = vel_ant
                obst = False

        d = using_width - d
        ang = np.round(d * kp, 3)  # + sum_d*1/1000
        d_ant = d
        if vel > vel_init*0.6:
            if abs(ang) < 11:
                vel = vel_init
            elif 13 > abs(ang) >= 11:
                vel = vel_init*0.9
            elif 15 > abs(ang) >= 13:
                vel = vel_init*0.8
            else:
                vel = vel_init*0.7

        # img_process_time3.append(time.time() - fr2_time)
        if View:
            cv2.putText(image, str(ang), (2, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
                        cv2.LINE_AA)
        if Print:
            # print(m1, b1,";", m2, b2)
            # print("\tW_c:", white_c, "\tlines:", len(line_count), "\tG_c:", green_c,)# "\tmove:", np.round(act_pos, 3))
            print(crosswalk, "\tsit:", situacao, "\t", "\tvel:", vel, "\tang:", ang)

        if Results and new_lap:
            # if Print:
            if lap_time == 0:
                j = 0
            print(j, " half lap time: ", lap_time)
            tracking_file.write("{}\t".format(lap_time))
            j = j + 1
            new_lap = False

        tecla = cv2.waitKey(1)
        # stop1, end1 = read_light(tecla, end1, stop1)
        # print(white_c)
        # if stop1 or not end1:
        #     if not on:
        #         vel_ant = vel
        #         on = True
        #     vel = 5
            # print(image[height-20, using_width])
        #    if white_c > 25000 and (stop1 or not end1):
        #        stop = True
        #        stop1 = False
        #        vel = vel_ant
        #        on = False
        stop, end = ajusta_veloc(tecla, end, stop)

        # limita angulo
        if ang < -45:
            ang = -45
        if ang > 45:
            ang = 45

        # publica valores
        tecla = cv2.waitKey(1)
        stop, end = ajusta_veloc(tecla, end, stop)
        if stop:
            vel_pub.publish(0)
        else:
            vel_pub.publish(vel)
        turn_pub.publish(ang)
        out2.write(image2)
        cv2.imshow('tracking', tracking)

        cv2.imshow('image', image)
        out.write(image)
        if Produce_images:
            cv2.imwrite(os.path.join(dirname, "{}_{}_{}_{}_px{}_{}.jpg".format(int(ang * 10), int(vel * 10), dirname,
                        fr, d + using_width, y)), cv2.resize(image2, (224, 224)))
            image2 = cv2.resize(image2, (224, 224))

        # if time.time() - fr_time >= 1:  # check fps
        #     fr_time = time.time()
        #     frames_array.append(fr)
        #     fr = 0
        # else:
        #     fr = fr + 1
        # img_process_time4.append(time.time() - fr2_time)

    if not end:
        out.release()
        cv2.destroyAllWindows()
        if Results:
            if lap == 0:
                lap = 1.1
            lap_time = np.round(time.time() - start, 3)
            tracking_file.write("{}\t".format(lap_time))
            cv2.imwrite(os.path.join(tracking_dir, "tracking_{}_{}_{}.jpg".format(dirname, lap, lap_time/lap)), tracking)
            tracking_file.write("\nlaps: {}\ttime per lap: {} sec\t{}\t {}\n".format(lap+0.5, np.round(lap_time/lap, 3), coll_count+1, vel))
        print("laps:", lap, "\ttime per lap:", lap_time/lap, "sec\t collisions:", coll_count, "\tvel:", vel)
        # print(frames_array, img_process_time1, img_process_time2, img_process_time3, img_process_time4)
        end_sim.publish(True)
        rospy.signal_shutdown("END")
    r.sleep()
