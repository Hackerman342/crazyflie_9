import os
import re

"""
Relabels txt and jpg files in a provided folder
"""

def main():
    i_jpg = 1
    i_txt = 1

    file_type1 = re.compile('[Jj][Pp][Gg]') #regex to see if its a jpg or txt file
    file_type2 = re.compile('[Tt][Xx][Tt]')


    for filename in os.listdir(path):
        check_1 = file_type1.search(filename)
        check_2 = file_type2.search(filename)


        if check_1!=None:
            dst = name_jpg + str(i_jpg) + ".jpg"
            src = path+ "/"+filename
            dst = path+"/"+dst
            os.rename(src, dst)
            i_jpg += 1

        if check_2!=None:
            dst = name_txt + str(i_txt) + ".txt"
            src = path+ "/"+filename
            dst = path+"/"+ dst
            os.rename(src, dst)
            i_txt += 1

    print('Done!')



if __name__ == '__main__':
    path="/home/robot/yolo_training/training_attempt_2/training_images/roundabout" #Set path here to the folder

    name_jpg = "roundabout"  # Self explanatory ;)
    name_txt = name_jpg

    main()