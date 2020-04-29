# Program to show various ways to read and
# write data in a file.
### MUST RUN FROM CRAZYFLIE_9 PATH FOR FILE TO BE IN PROPER PLACE"
img_count_0 = 200
img_count_1 = 198
img_count_2 = 288
img_count_3 = 125
img_count_4 = 201

test_remainder = 10 # Place every __ image in test.txt instead


file1 = open("/home/robot/yolo_training/training_attempt_2/train.txt","r+")
file2 = open("/home/robot/yolo_training/training_attempt_2/test.txt","r+")


L1 = []
L2 = []

for i in range(1, img_count_0 + 1):
    if i % test_remainder != 0:
        L1.append("data/obj/narrows_from_left"+str(i)+".jpg\n")
    else:
        L2.append("data/obj/narrows_from_left"+str(i)+".jpg\n")

for i in range(1, img_count_1 + 1):
    if i % test_remainder != 0:
        L1.append("data/obj/narrows_from_right"+str(i)+".jpg\n")
    else:
        L2.append("data/obj/narrows_from_right"+str(i)+".jpg\n")

for i in range(1, img_count_2 + 1):
    if i % test_remainder != 0:
        L1.append("data/obj/no_bicycle"+str(i)+".jpg\n")
    else:
        L2.append("data/obj/no_bicycle"+str(i)+".jpg\n")

for i in range(1, img_count_3 + 1):
    if i % test_remainder != 0:
        L1.append("data/obj/residential"+str(i)+".jpg\n")
    else:
        L2.append("data/obj/residential"+str(i)+".jpg\n")

for i in range(1, img_count_4 + 1):
    if i % test_remainder != 0:
        L1.append("data/obj/roundabout"+str(i)+".jpg\n")
    else:
        L2.append("data/obj/roundabout"+str(i)+".jpg\n")

file1.writelines(L1)
file2.writelines(L2)

file1.close()
file2.close()

print("Done!")