
#Renato Fernandes Rodrigues
#Universidade Federal do ABC
#Engenharia de Instrumentacao, automacao e robotica
#Trabalho de Graduacao

import argparse
# import imutils
import time
import cv2
import os
import numpy as np
import math
from Configs import Configs

# construct the argument parse and set flags
ap = argparse.ArgumentParser()
ap.add_argument("-d", "--display", type=int, default=1,help="#Show display? Y=1 N=0")
ap.add_argument("-r", "--raspberry", type=int, default=0,help="# runnning on Raspberry Pi? Y=1 N=0")
ap.add_argument("-s", "--send", type=int, default=0,help="#Send commands to ESP? Y=1 N=0")
args = vars(ap.parse_args())

flag_display = args["display"]
flag_raspberry = args["raspberry"]
flag_send = args["send"]

# Raspberry pi mode initialization
if flag_raspberry > 0:
    from pivideostream import PiVideoStream
    from picamera.array import PiRGBArray
    from picamera import PiCamera

# Configure file JSON path
configs = Configs(os.path.dirname(os.path.realpath(__file__))+'/Configs.json')

# Socket initialization
if flag_send > 0:
    import socket
    HOST = configs.get("host")
    PORT = configs.get("port")
    tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    dest = (HOST, PORT)
    tcp.settimeout(0.0)
    print "Sending commands to "+str(dest)
    try:
        tcp.connect(dest)
    except socket.error as msg:
        if msg[0] != 115:
            print "Socket Error "+str(msg[0])
            print "Abort operation, try again later."
            raise SystemExit

# Global variables
plotWidth = configs.get("cameraRes")[0]
plotHeigth = configs.get("cameraRes")[1]
hand_color_l = np.array(configs.get("hand_color_l"))
hand_color_h = np.array(configs.get("hand_color_h"))
span_h = configs.get("span_h")
span_l = configs.get("span_l")
span_s = configs.get("span_s")
timer_send_start = 0
timer_list = []
timer_start = 0
timer_list_flag = True
timer_list_2 = []
loop_timer_start = 0
timer_send_start = 0
calibrate_hand_flag = False
command = " "
status_detector = " "
calibration_counter = 0
command_buffer = []
flag_calibration_timeout = False
calibration_timeout_counter = 0
# Variables initialization

plot = np.zeros((plotHeigth, plotWidth, 3), np.uint8)
frame = np.zeros((plotHeigth, plotWidth, 3), np.uint8)
frame_hls = np.zeros((plotHeigth, plotWidth, 3), np.uint8)

kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))

print "Color Ranges Loaded: "+str(hand_color_h)+" to "+str(hand_color_l)

def send_command(comm):
    global timer_send_start
    data = ""
    if flag_send > 0:
        if int((cv2.getTickCount() - timer_send_start)/ cv2.getTickFrequency()*1000) > 200:
            print "Command to send:"+comm
            try:
                if comm != "":
                    tcp.send(comm)
                    data = tcp.recv(1024)
                    print "Received: "+str(data)
                    if data == "O!\r\n":
                        print "Sucessfull Sent!"
            except:
                print "Send Failed"
            timer_send_start = cv2.getTickCount()

def calc_multi_params(start,end,far,center):
    xs,ys = start
    xe,ye = end
    xf,yf = far
    xc,yc = center
    dist_start = math.sqrt(math.pow((xs-xf),2)+math.pow((ys-yf),2))
    dist_end = math.sqrt(math.pow((xe-xf),2)+math.pow((ye-yf),2))
    dist_start_end = math.sqrt(math.pow((xe-xs),2)+math.pow((ye-ys),2))
    angle = math.degrees(math.acos((dist_start*dist_start + dist_end*dist_end - dist_start_end*dist_start_end)/(2*dist_start*dist_end)))
    dist_start_center = math.sqrt(math.pow((xs-xc),2)+math.pow((ys-yc),2))
    dist_end_center = math.sqrt(math.pow((xe-xc),2)+math.pow((ye-yc),2))
    dist_far_center = math.sqrt(math.pow((xf-xc),2)+math.pow((yf-yc),2))
    return int(angle),int(dist_start),int(dist_end),int(dist_start_center),int(dist_end_center),int(dist_far_center)

def calc_distance(p1,p2):
    return int(math.sqrt(math.pow((p1[0]-p2[0]),2)+math.pow((p1[1]-p2[1]),2)))

def calc_meanpoint(p1,p2):
    return int((p1[0]+p2[0])/2),int((p1[1]+p2[1])/2)

def start_timer():
    global timer_start
    timer_start = cv2.getTickCount()

def end_timer(name):
    global timer_list
    timer_list.append([name,int((cv2.getTickCount() - timer_start)/ cv2.getTickFrequency()*1000000)])


def update_trackbars(arg1):
    # print arg1
    global span_h
    global span_l
    global span_s
    span_h = cv2.getTrackbarPos("H","plots")
    span_l = cv2.getTrackbarPos("L","plots")
    span_s = cv2.getTrackbarPos("S","plots")

    configs.set("span_h",span_h)
    configs.set("span_l",span_l)
    configs.set("span_s",span_s)

# def update_command(, center)

if flag_display:
    cv2.namedWindow("plots")
    cv2.createTrackbar("H","plots",span_h,255,update_trackbars)
    cv2.createTrackbar("L","plots",span_l,255,update_trackbars)
    cv2.createTrackbar("S","plots",span_s,255,update_trackbars)


# initialize and configure camera module
if(flag_raspberry>0):
    print("Starting Raspberry Pi Camera module Thread")
    vs = PiVideoStream().start()
else:
    print "Start OpenCV Video Capture module"
    vs = cv2.VideoCapture(0)
time.sleep(2.0)
try:
    while True:
        # store fps
        timer_list =[]
        timer_list.append(["FPS",int(cv2.getTickFrequency()/(cv2.getTickCount() - loop_timer_start))])
        loop_timer_start = cv2.getTickCount()

        finger_list_filtered = []
        finger_num = 0
        contour_solidity = 0
        center = (0,0)
        defects_list = []
        hull_rp = np.array([])
        mean_dist_center = 0
        start_timer()
        if flag_raspberry > 0:
            frame_full = vs.read()
        else:
            ret,frame_full = vs.read()
            if ret == False:
                continue
        frame = frame_full[plotHeigth/4:plotHeigth*3/4,plotWidth/4:plotWidth*3/4]
        frame = cv2.flip(frame,-1)
        end_timer("read")

        start_timer()
        frame_hls = cv2.cvtColor(frame.copy(),cv2.COLOR_BGR2HLS)
        end_timer("cvtColor HSL")

        if calibrate_hand_flag:
            start_timer()
            roi_range = 20
            roi_hist_p1 = (frame.shape[1]/2 - roi_range,frame.shape[0]/2 - roi_range)
            roi_hist_p2 = (frame.shape[1]/2 + roi_range,frame.shape[0]/2 + roi_range)
            cv2.rectangle(frame,roi_hist_p1,roi_hist_p2,[255,0,255],2)

            roi_hist = frame_hls[roi_hist_p1[1]:roi_hist_p2[1],roi_hist_p1[0]:roi_hist_p2[0]]

            hist_h = cv2.calcHist([roi_hist],[0],None,[256],[0,256])
            hist_l = cv2.calcHist([roi_hist],[1],None,[256],[0,256])
            hist_s = cv2.calcHist([roi_hist],[2],None,[256],[0,256])

            max_hist_h = sorted(range(len(hist_h)), key=lambda k: hist_h[k],reverse=True)[:3]
            max_hist_l = sorted(range(len(hist_l)), key=lambda k: hist_l[k],reverse=True)[:3]
            max_hist_s = sorted(range(len(hist_s)), key=lambda k: hist_s[k],reverse=True)[:3]

            hand_color_l = np.clip([min(max_hist_h)-span_h,min(max_hist_l)-span_l,min(max_hist_s)-span_s],0,255)
            hand_color_h = np.clip([max(max_hist_h)+span_h,max(max_hist_l)+span_l,max(max_hist_s)+span_s],0,255)

            print "h:"+str(max_hist_h)+"\n\n\tl:"+str(max_hist_l)+"\n\n\ts:"+str(max_hist_s)
            configs.set("hand_color_l",[hand_color_l[0],hand_color_l[1],hand_color_l[2]])
            configs.set("hand_color_h",[hand_color_h[0],hand_color_h[1],hand_color_h[2]])
            print hand_color_l
            print hand_color_h

            end_timer("calibrate")
            calibrate_hand_flag = False
            timer_list_flag = True

        # Threshold Image
        start_timer()

        mask0 = cv2.inRange(frame_hls,hand_color_l,hand_color_h)

        end_timer("inRange")

        # Erode and dilate
        start_timer()

        # mask = cv2.erode(mask0, kernel,iterations=2)
        mask = cv2.dilate(mask0, kernel,iterations=4)
        mask2 = cv2.erode(mask, kernel,iterations=4)

        end_timer("Dilate/Erode")

        # Find contours in mask
        start_timer()

        _,cnts, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts2 = sorted(cnts,key=cv2.contourArea,reverse=True)[:1]

        finger_num = -1
        for c in cnts2:

            # Area filter
            contour_area = cv2.contourArea(c)
            if contour_area < 3000 or contour_area > 30000:
                status_detector = "Status: Out of Area range ("+str(contour_area)+")"
                # print status_detector
                continue

            # center of mass of hand area mask
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # calculate all distances of points to center
            dist_center_c = []
            for i in xrange(len(c)):
                dist_center_c.append((c[i][0],calc_distance(c[i][0],center)))
            dist_center_c = sorted(dist_center_c,key = lambda s: s[1],reverse=True)

            # calculate median radius of all contours points to center
            mean_dist_center = 0
            median_dist_center = np.median([dist_center_c[i][1] for i in range(0,len(dist_center_c))], axis=0)
            mean_dist_center = int(1.1*np.average([dist_center_c[i][1] for i in range(0,len(dist_center_c))],axis =0))

            hull_rp = cv2.convexHull(c,returnPoints = True)
            # print "hull_rp\n"+str(hull_rp)

            # calculate solidity
            hull_area = cv2.contourArea(hull_rp)
            contour_solidity = float(contour_area)/hull_area
            if contour_solidity > 0.9:
                finger_num = 0
                continue

            # get the hull and defects points
            hull = cv2.convexHull(c,returnPoints = False)
            defects = cv2.convexityDefects(c,hull)

            # construct a list of defects points and distances
            defects_list = []
            for i in range(defects.shape[0]):
            	s,e,f,d = defects[i,0]
            	start = tuple(c[s][0])
            	end = tuple(c[e][0])
            	far = tuple(c[f][0])
            	angle,dist_start,dist_end,dist_start_center,dist_end_center,dist_far_center = calc_multi_params(start,end,far,center)
            	defects_list.append([start,end,far,angle,dist_start,dist_end,dist_start_center,dist_end_center,dist_far_center])
                                        # 0   1   2   3       4           5           6               7               8

            # filter by openin finger angles
            defects_list = sorted(defects_list, key = lambda s: s[3]) [:10]
            finger_list=[]
            for i in range(0,len(defects_list)):
                if defects_list[i][3] < 100 and defects_list[i][3] > 5:
                    if defects_list[i][6] > mean_dist_center:
                        if 0.5 < float(defects_list[i][4])/defects_list[i][6] < 1.3:
                            finger_list.append(defects_list[i][0])
                            cv2.line(frame,defects_list[i][2],defects_list[i][0],[255,255,255],2)
                    if defects_list[i][7] > mean_dist_center:
                        if 0.5 < float(defects_list[i][5])/defects_list[i][7] < 1.3:
                            finger_list.append(defects_list[i][1])
                            cv2.line(frame,defects_list[i][2],defects_list[i][1],[255,255,255],2)
            finger_list = sorted(finger_list)

            # if it din't find any valid defects,
            # get the farthest point to be a candidate for finger
            if len(finger_list) == 0:
                finger_list.append(dist_center_c[0][0])
                finger_list_filtered.append((dist_center_c[0][0][0],dist_center_c[0][0][1]))
                finger_num = 1
                continue


            # remove near finger points
            finger_list = sorted(finger_list, key = lambda s: s[0])
            nearst_dist = 20    #max distance between point to mean points
            finger_list_filtered = []
            for i in xrange(len(finger_list)):
                flag_duplicated = False
                point_sum = []
                for k in xrange(len(finger_list)):
                    if calc_distance(finger_list[i],finger_list[k]) < nearst_dist:
                        point_sum.append(finger_list[k])
                filtered_meanpoint = tuple(map(lambda y: int(sum(y) / float(len(y))), zip(*point_sum)))

                if filtered_meanpoint not in finger_list_filtered:
                    finger_list_filtered.append(filtered_meanpoint)

            finger_num = len(finger_list_filtered)

        end_timer("Find Contours")

        # Check for Calibration
        start_timer()
        if calibration_counter < 20:
            command_buffer.append(finger_num)
            calibration_counter = calibration_counter + 1
        else:
            min_mode_command = max(set(command_buffer), key=command_buffer.count)
            min_mode_command_percent = 100*command_buffer.count(min_mode_command)/len(command_buffer)
            print "Mode fingers: "+str(min_mode_command)+" | "+str(min_mode_command_percent)+"%"


            if min_mode_command == 5 and flag_calibration_timeout:
                flag_calibration_timeout = False
                hand_color_l = hand_color_l_temp
                hand_color_h = hand_color_h_temp
                print "Got 5 Fingers! System are calibrated"

            if not flag_calibration_timeout and min_mode_command < 0:
                calibration_timeout_counter = calibration_timeout_counter + 1
                if calibration_timeout_counter >= 3:
                    flag_calibration_timeout = True
                    calibration_timeout_counter = 0
                    print "Timeout Calibration!! put yout hand in the middle and open all your fingers"
            if flag_calibration_timeout:
                calibrate_hand_flag =True
                hand_color_l_temp = hand_color_l
                hand_color_h_temp = hand_color_h
                print "Calibration on!"
            command_buffer = []
            command_buffer.append(finger_num)
            calibration_counter = 0

        end_timer("Calibration_check")

        # Set behaviours - pode checar se o comando for maior que 50% predominandte, senao conta tambem e anda calibar
        start_timer()
        if not flag_calibration_timeout:
            if finger_num < 0:
                command = "O"

            if finger_num == 0:
                command = "S"

            finger_angle_offset = 0
            finger_angle_range = 25
            if finger_num == 1:
            	finger_angle = int(math.degrees(math.acos(float(finger_list_filtered[0][0] - center[0])/(calc_distance(finger_list_filtered[0],center)))))
            	if finger_angle > (90+finger_angle_range+finger_angle_offset) :
            		command = "FL"
            	else:
            		if finger_angle < (90-finger_angle_range+finger_angle_offset):
            			command = "FR"
            		else:
            			command = "F0"
            if finger_num == 2:
                mean_2_fingers = ((finger_list_filtered[0][0]+finger_list_filtered[1][0])/2,(finger_list_filtered[0][1]+finger_list_filtered[1][1])/2)
                finger_angle = int(math.degrees(math.acos(float( mean_2_fingers[0]- center[0])/(calc_distance(mean_2_fingers,center)))))
                if finger_angle > (90+finger_angle_range+finger_angle_offset) :
                    command = "BL"
                else:
                    if finger_angle < (90-finger_angle_range+finger_angle_offset):
                        command = "BR"
                    else:
                        command = "B0"

            if 3 <= finger_num <= 4:
                command = "N"

            if finger_num >= 5:
                command = "X"
        end_timer("Command")

        # Display it if flag is setted
        start_timer()
        if flag_display:
            if finger_num >= 0:
                status_detector = "Status: "+str(finger_num)+" - command:"+command
            # print status_detector
            for i in xrange(0,len(defects_list)):
                cv2.putText(frame,str(i),defects_list[i][2] , cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 0, 255), 2)
            for i in xrange(0,len(finger_list_filtered)):
                cv2.putText(frame,str(i),finger_list_filtered[i] , cv2.FONT_HERSHEY_SIMPLEX,1, (0, 255,0), 2)
                cv2.line(frame,center,finger_list_filtered[i],[0,255,0],2)
            cv2.circle(frame, center, 10, (0,255,255),-1)
            # cv2.circle(frame, center2, 10, (255,0,255),-1)
            cv2.putText(frame,str(command),center, cv2.FONT_HERSHEY_SIMPLEX,1, (255, 255,0), 2)
            cv2.drawContours(frame, [hull_rp], -1, (255,0,0),lineType = cv2.LINE_8, thickness = 2)
            # cv2.circle(frame, center, int(dist_center_c[len(dist_center_c)-2][1]), (0,150,150),2)
            cv2.circle(frame, center, int(mean_dist_center), (55,250,50),2)
            cv2.putText(frame,status_detector,(20,20), cv2.FONT_HERSHEY_SIMPLEX,0.5, (255, 255,0), 2)
            cv2.putText(frame,"solidity: "+str("{0:2.2f}".format(contour_solidity)),(20,40), cv2.FONT_HERSHEY_SIMPLEX,0.5, (255, 255,0), 2)
            # cv2.putText(frame,status_detector,(20,60), cv2.FONT_HERSHEY_SIMPLEX,0.4, (255, 255,0), 2)
            cv2.imshow("plots",frame)
            cv2.imshow("mask2",mask2)
            # cv2.imshow("mask0",mask0)

        end_timer("plot")

        start_timer()
        # Send commands via TCP socket
        send_command(command)
        command = " "
        end_timer("Send commands")

        timer_list_2.append(timer_list)
        if timer_list_flag:
            print "Timer_list:\n instant: " +str(timer_list)
            timer_list_mean = []
            for k in xrange(0,len(timer_list_2[0])):
                timer_list_mean.append((timer_list[k][0],"{0:2.2f}".format(np.average([timer_list_2[i][k][1] for i in range(0,len(timer_list_2))],axis = 0))))

            print " average: "+ str(timer_list_mean)
            timer_list_2 = []
            timer_list_flag = False

        k = cv2.waitKey(1) & 0xFF
        if k == 27 or k == ord('q'):
            timer_list_mean = []
            for k in xrange(0,len(timer_list_2[0])):
                timer_list_mean.append((timer_list[k][0],"{0:2.2f}".format(np.average([timer_list_2[i][k][1] for i in range(0,len(timer_list_2))],axis = 0))))

            print " average: "+ str(timer_list_mean)
            send_command("X")
            command = " "
            break
        if k == ord('d'):
            flag_display = not(flag_display)
        if k == ord('c'):
            # cv2.destroyAllWindows()
            calibrate_hand_flag =True
        if k == ord('t'):
            timer_list_flag = True
except KeyboardInterrupt:
    pass
print "Closing program..."
# Clean windows
if flag_display:
    cv2.destroyAllWindows()

# Stop camera module
if(flag_raspberry>0):
    vs.stop()
else:
    vs.release()


# http://www2.ic.uff.br/iwssip2010/Proceedings/nav/papers/paper_128.pdf
