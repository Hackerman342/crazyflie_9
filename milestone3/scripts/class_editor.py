#!/usr/bin/env python3

import os
import re

"""
Changes the class of yolo labelled images
Only works for images with one object (for now)
"""

path = "/home/robot/yolo_training/training_images/proper_order_images/roundabout"
file_type = re.compile('[Tt][Xx][Tt]')
count = 0 # Verification of number of files

old_class = '4'
new_class = '3'

for filename in os.listdir(path):
    check = file_type.search(filename)
    print(filename)

    if check!=None:
        count += 1

        file = open(path + '/' + filename, "r")
        text = file.read()
        file.close()
        file = open(path + '/' + filename, "w")
        #print(text[len(old_class):])
        file.write(new_class + text[len(old_class):])
        file.close()

print("Edited", count, "files")

