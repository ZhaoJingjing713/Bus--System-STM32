import pymysql
import sys
id_num=0

def GPS(text,id_num):
    n=((text.split(",")[0]))
    s=((text.split(",")[1]))
    number=((text.split(",")[2]))
    id_num+=1
    db = pymysql.connect('140.143.54.132','root','6a406dee537e603d','STM32_GPS')

    cursor = db.cursor()
        
    cursor.execute('INSERT INTO NS(N,E,num,id) '
                        'VALUES (%s,%s,%s,%d)'%(n,s,number,id_num))
    print('INSERT INTO NS(N,E,num,id) '
                    'VALUES (%s,%s,%s,%d)'%(n,s,number,id_num))
    db.commit()

    db.close()
    cursor.close()
text="122.1231,32.1342339,6"

GPS(text,id_num)
