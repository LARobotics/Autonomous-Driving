import cv2
import sys
import rospy
import std_msgs.msg as std_msg
import sensor_msgs.msg as sensor_msg
import numpy as np
import torch
import torchvision
import PIL.Image
import glob
from torchvision import datasets, models, transforms

mean = torch.Tensor([0.485, 0.456, 0.406]).cpu()
std = torch.Tensor([0.229, 0.224, 0.225]).cpu()
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")


def preprocess(image):
    global device
    image=cv2.resize(image, (224, 224))
    image = PIL.Image.fromarray(image)
    image = transforms.functional.to_tensor(image).to(device)
    image.sub_(mean[:, None, None]).div_(std[:, None, None])
    return image[None, ...]


def get_img(msg):
    bridge = CvBridge()
    # print(msg.header, "\n", msg.height, msg.width)
    image_opencv = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    image_opencv = cv2.flip(image_opencv, 0)
    cv2.imshow("sensor", image_opencv)
    cv2.waitKey(1)


global a
a = 0
rospy.init_node('vision_control')
vel_pub = rospy.Publisher('/velocity_Data', std_msg.Int64, queue_size=2)
turn_pub = rospy.Publisher('/turn_Data', std_msg.Float32, queue_size=2)
camera_sub = rospy.Subscriber('/camera_Data', sensor_msg.Image, get_img)
r = rospy.Rate(10)  # 10hz

path = glob.glob("lap30/*.jpg")
print("start")

model = torchvision.models.resnet18(pretrained=True)
model.fc = torch.nn.Linear(512, 4)
model = model.to(device)

model.load_state_dict(torch.load('models/model3_20ep_8bs.pth', map_location=device))
# model = torch.load('models/model_5ep_32bs.pth', map_location=device)
model = model.eval()

for i in path:
    img = cv2.imread(i)
    img = preprocess(img)
    output = model(img).detach().cpu().numpy().flatten()
    x = output[0]
    y = output[1]
    x = int(224 * (x / 2.0 + 0.5))
    y = int(224 * (y / 2.0 + 0.5))
    print(i, x, y)

# while not rospy.is_shutdown():
#    vel_pub.publish(int(a))
#    r.sleep()
    # a = a+0.1
