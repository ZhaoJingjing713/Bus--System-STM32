import socket
import sys
import pymysql
import numpy as np
import time
import face_recognition
import dlib

id_num=0

def GPS(text,id_num):
    #try:
    n=((text.split(",")[0]))
    s=((text.split(",")[1]))
    number=((text.split(",")[2]))
    db = pymysql.connect('140.143.54.132','root','6a406dee537e603d','STM32_GPS')

    cursor = db.cursor()
    #number="9"
    cursor.execute('INSERT INTO NS(N,E,num,id) '
                    'VALUES (%s,%s,%s,%d)'%(n,s,number,id_num))
    db.commit()

    db.close()
    cursor.close()
    #except:
        #print("1231123")

# 1. 创建套接字
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# 2. 绑定一个本地信息
localaddr = ("192.168.43.23",1000)
udp_socket.bind(localaddr)  # 必须绑定自己电脑的ｉｐ以及ｐｏｒｔ，其他的不行
known_obama_image = face_recognition.load_image_file("mym.jpg")
known_biden_image = face_recognition.load_image_file("dxc.png")
# Get the face encodings for the known images
obama_face_encoding = face_recognition.face_encodings(known_obama_image)[0]
biden_face_encoding = face_recognition.face_encodings(known_biden_image)[0]
known_encodings = [
    obama_face_encoding,
    biden_face_encoding
]
time1=time.localtime()
# 3. 接收数据
while True:
    time2=(time.localtime())
    time_=time2.tm_sec-time1.tm_sec
    recv_data = udp_socket.recvfrom(1024)
    # recv_data这个变量中存储的是一个元组(接收到的数据，(发送方的ip, port))
    recv_msg = recv_data[0]  # 存储接收的数据
    send_addr = recv_data[1]  # 存储发送方的地址信息
    # 4. 打印接收到的数据
    #print(type(recv_msg))
    #print(recv_data)
    #print((recv_msg.decode('utf-8','ignore').split(',')[2]))
   # try:
    mark = recv_msg.decode('utf-8','ignore').split('$')[0]
    if mark == 'GPS':
        N=(recv_msg.decode('utf-8','ignore').split('$')[1]).split(',')[0]
        E=(recv_msg.decode('utf-8','ignore').split('$')[1]).split(',')[1]
        NUM=(recv_msg.decode('utf-8','ignore').split('$')[1]).split(',')[2]
    
        N=str(float(N)/100+0.3538102)
        E=str(float(E)/100+0.2119751)
        
        text=(N)+','+(E)+','+(NUM)
    
        print(text)
        id_num+=1
        time1=time.localtime()
        
        if(time_%5==0):
            GPS(text,id_num)
            time.sleep(1)
    elif mark == 'IMG':
        txt=str(recv_msg.decode('HEX','ignore'))
        file = open("123.txt",'w')
        file.write(txt)
        file.close()
        print(txt)
        input()
        mid=[[]for i in range(1000)]
        end=[]
        for i in range(0,len(txt),2):
            fir=int(txt[i],16)<<8
            sec=int(txt[i+1],16)
            val=(sec+fir)
            inter=1
            r,g,b=0,0,0
            rgbval=0
            while val != 0:
                tem = val%2
                val = val//2
                if inter<=5:
                    rgbval += tem*pow(2,inter-1)
                    b = rgbval
                    if inter == 5:
                        rgbval=0
                if 5< inter<= 11:
                    rgbval += tem*pow(2,inter-6)
                    g = rgbval
                    if inter == 11:
                        rgbval=0
                if inter > 11:
                    rgbval += tem*pow(2,inter-12)
                    r = rgbval
                inter+=1
            mid[i//640].append([r,g,b])
            if (i+2)%640==0:
                end.append(mid[i//640])
        end = np.array(end)
        image_to_test = face_recognition.load_image_file("mym2.jpg")
        image_to_test_encoding = face_recognition.face_encodings(image_to_test)[0]
        
        # See how far apart the test image is from the known faces
        face_distances = face_recognition.face_distance(known_encodings, image_to_test_encoding)
        
        for i, face_distance in enumerate(face_distances):
            print("The test image has a distance of {:.2} from known image #{}".format(face_distance, i))
            print("- With a normal cutoff of 0.6, would the test image match the known image? {}".format(face_distance < 0.5))
            print("- With a very strict cutoff of 0.5, would the test image match the known image? {}".format(face_distance < 0.35))
            print()
    #except:1325.2322,145687.2323
     #   print(recv_msg)
    #print("%s:%s" % (str(send_addr), recv_msg.decode("utf-8")))
    #print("%s:%s" % (str(send_addr), recv_msg.decode("gbk")))
# 5. 关闭套接字
udp_socket.close()

