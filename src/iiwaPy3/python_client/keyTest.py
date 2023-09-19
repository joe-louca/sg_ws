import cv2

raw_img = cv2.imread('/home/joe/RobotiqSideProfile.png')

while True:
    cv2.imshow("Contacts",raw_img)
    k = cv2.waitKey()
   
    if k == -1:
        continue
    else:
         print(k)
##    elif k == -1:
##        continue
##    else:
##        print(k)
