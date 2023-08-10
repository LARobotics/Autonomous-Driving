import cv2
import sim
import time
import datetime
import sys
import os
import rospy
import std_msgs.msg as std_msg
import sensor_msgs.msg as sensor_msg
import numpy as np
import torch
import torchvision
import PIL.Image
import glob
from typing import Final
from cv_bridge import CvBridge
from torchvision import datasets, models, transforms

mean = torch.Tensor([0.485, 0.456, 0.406]).cpu()
std = torch.Tensor([0.229, 0.224, 0.225]).cpu()
print(torch.cuda.is_available())
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(device)

Results: Final = False
poss_vel = [10, 12.5, 15, 17.5, 20]
poss_kp = [0.15, 0.25, 0.3, 0.35, 0.4]    # [0.1, 0.111, 0.117, 0.12, 0.12]
vel_init = poss_vel[2]
kp = poss_kp[2]
print(vel_init, kp)
vel = vel_init
img = 0
new_frame = 0
end = True
stop = True
# resolution = []
coll_count = 0
pos_count = 0
lap = -0.5
lap_time = 0
new_lap = False
j = 0
start = -1
pos_Yant = 0
x = 0
# y = 0
fr=0
frames_array = []
img_process_time1 = []
img_process_time2 = []
img_process_time3 = []
img_process_time4 = []


def init_tracking():
    track_img = cv2.imread('/home/ines_arib/Uni/simulations/pista2.jpeg')
    track_w, track_h = track_img.shape[:2]
    track_w = round(track_w / 2.4)
    track_h = round(track_h / 2.4)
    track_img = cv2.resize(track_img, (track_h, track_w))
    pos = (track_h / 2, track_w / 2)

    return pos, track_img


def process_img(image):
    global device
    image=cv2.resize(image, (224, 224))
    image = PIL.Image.fromarray(image)
    image = transforms.functional.to_tensor(image).to(device)
    image.sub_(mean[:, None, None]).div_(std[:, None, None])
    return image[None, ...]


def get_img(msg):
    global img, resolution, new_frame
    bridge = CvBridge()
    # print(msg.header, "\n", msg.height, msg.width)
    resolution = [msg.width, msg.height]
    img = cv2.flip(bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough'), 0)
    new_frame = 1


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


def reset_sim(msg):
    global coll_count, lap, lap_time, tracking_file, tracking, dirname, tracking_dir, last_pos, new_lap, start
    if msg.data:
        vel_pub.publish(0)
        turn_pub.publish(0)
        sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot_wait)
        time.sleep(1)
        if Results and lap > 0:
            # if Print:
            print("lap:", lap, "\ttime per lap:", lap_time / lap, "sec")
            tracking_file.write("\nlap: {}\ttime per lap: {} sec\n".format(lap, lap_time / lap))
            cv2.imwrite(os.path.join(tracking_dir, "tracking_net_{}_{}_{}.jpg".format(vel_init, lap, np.round(lap_time/lap, 3))),
                        tracking)
        last_pos, tracking = init_tracking()
        coll_count += 1
        lap = -0.5
        start = -1
        sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot_wait)
    return 0


def get_pos(msg):
    global act_pos, last_pos, pos_Yant, lap, pos_count, start, lap_time, tracking_file, new_lap, tracking
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


rospy.init_node('vision_control')
vel_pub = rospy.Publisher('/velocity_Data', std_msg.Float32, queue_size=2)
turn_pub = rospy.Publisher('/turn_Data', std_msg.Float32, queue_size=2)
end_sim = rospy.Publisher('/end', std_msg.Bool, queue_size=1)
camera_sub = rospy.Subscriber('/camera_Data', sensor_msg.Image, get_img)
collis_sub = rospy.Subscriber('/collision_Data', std_msg.Bool, reset_sim)
pos_sub = rospy.Subscriber('/position_Data', std_msg.Float32MultiArray, get_pos)
r = rospy.Rate(10)  # 10hz

model = torchvision.models.resnet18(pretrained=True)
model.fc = torch.nn.Linear(512, 4)
model = model.to(device)

model.load_state_dict(torch.load('models/model5_20ep_8bs.pth', map_location=device))
model = model.eval()

init_pos, tracking = init_tracking()
act_pos = init_pos
last_pos = init_pos

sim.simxFinish(-1)   # just in case, close all opened connections
clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID != -1:  # verifica que se conseguio ligar a simulação
    sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot_wait)
    time.sleep(1)
    sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot_wait)

if Results:
    tracking_dir = "tracking_network"
    tracking_file = open('tracking_network.txt', 'a')
    tracking_file.write("\n________________  network {}    -    {}  __________________\n".format(vel_init, datetime.datetime.now().strftime("%I:%M%p, %B %d, %Y")))


fr_time = time.time()
print("ready")

while not rospy.is_shutdown():
    if new_frame:
        fr2_time = time.time()
        # print(fr2_time)
        new_frame = 0
        img2 = process_img(img)
        #img_process_time1.append(time.time() - fr2_time)

        output = model(img2).detach().cpu().numpy().flatten()
        #img_process_time2.append(time.time() - fr2_time)
        x = output[0]
        x = int(224 * (x / 2.0 + 0.5))/10
        # y = output[1]
        # y = int(224 * (y / 2.0 + 0.5))/10
        # print(x, y)
        x = x*(kp+1)
        tecla = cv2.waitKey(1)
        stop, end = ajusta_veloc(tecla, end, stop)
        turn_pub.publish(x)

        if abs(x) < 12:
            vel = vel_init
        elif 15 > abs(x) > 12:
            vel = vel_init * 0.95
        else:
            vel = vel_init * 0.9

        if stop:
            vel_pub.publish(0)
        else:
            vel_pub.publish(vel)
        #img_process_time3.append(time.time() - fr2_time)

        if Results and new_lap:
            # if Print:
            if lap_time == 0:
                j = 0
            print(j, "\thalf lap time: ", lap_time, "\tvel", vel)
            tracking_file.write("{}\t".format(lap_time))
            j = j+1
            new_lap = False
        cv2.imshow('tracking', tracking)

        cv2.putText(img, str(np.round(x, 2)), (2, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.imshow("sensor", img)

        #if time.time() - fr_time >= 1:  # check fps
        #    fr_time = time.time()
        #    frames_array.append(fr)
        #    fr = 0
        #else:
        #    fr = fr + 1
        #img_process_time4.append(time.time() - fr2_time)

    if not end:
        cv2.destroyAllWindows()
        if Results:
            if lap == 0:
                lap = 1.1
            tracking_file.write("{}\t".format(np.round(lap_time, 3)))
            cv2.imwrite(os.path.join(tracking_dir, "tracking_net_{}_{}_{}.jpg".format(vel_init, lap, lap_time/lap)), tracking)
            tracking_file.write("\nlaps: {}\ttime per lap: {} sec\t{}\t {}\n".format(lap, np.round(lap_time/lap, 3), coll_count, vel))
        print("laps:", lap, "\ttime per lap:", lap_time/lap, "sec\t collisions:", coll_count, "\tvel:", vel)
        # print(frames_array, img_process_time1, img_process_time2, img_process_time3, img_process_time4)
        end_sim.publish(True)
        rospy.signal_shutdown("END")
    r.sleep()