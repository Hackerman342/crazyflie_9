# Program to show various ways to read and
# write data in a file.
### MUST RUN FROM CRAZYFLI_9 PATH FOR FILE TO BE IN PROPER PLACE"
img_count = 201


file = open("milestone3/scripts/train.txt","w+")

# \n is placed to indicate EOL (End of Line)
#L = ["This is Delhi \n","This is Paris \n","This is London \n"]
L = []
for i in range(img_count):
    L.append("data/obj/roundabout"+str(i)+".jpg\n")
file.writelines(L)
file.close() #to change file access modes