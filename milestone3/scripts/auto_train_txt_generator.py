# Program to make a list of all images in a fodler for DNN training

import os
import re
# test_remainder = 10 # Place every __ image in test.txt instead


path = "/home/robot/yolo_training/training_attempt_3/training_images"
train = open("/home/robot/yolo_training/training_attempt_3/setup/train.txt","w")
# file2 = open("/home/robot/yolo_training/training_attempt_2/test.txt","r+")


L1 = []
# L2 = []

file_type = re.compile('[Jj][Pp][Gg]') #regex to see if its a jpg or txt file

for filename in os.listdir(path):
    check = file_type.search(filename)


    if check!=None:
        L1.append('data/obj/'+filename+'\n')


train.writelines(L1)
# file2.writelines(L2)

train.close()
# file2.close()

print("Done!")