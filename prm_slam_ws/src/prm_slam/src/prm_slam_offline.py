#!/usr/bin/python3


import rospy
import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import os
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import sin, cos, pi, sqrt

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

bandera=0
path_metros=[
[	0.403792957746479	,	-0.253895535714286	],
[	-0.05047411971831	,	-0.558570178571428	],
[	0.302844718309859	,	-1.26947767857143	],
[	0.504741197183099	,	-1.52337321428571	],
[	1.26185299295775	,	-0.964803035714286	],
[	2.0694389084507	,	-0.914023928571428	],
[	2.32180950704225	,	-0.101558214285714	],
[	1.81706830985916	,	0.35545375	],
[	2.17038714788732	,	1.21869857142857	],
[	2.47323186619718	,	1.67571053571429	],
[	2.92749894366197	,	2.18350160714286	],
[	3.48271426056338	,	2.38661803571429	],
[	4.13887781690141	,	2.48817625	],
[	4.13887781690141	,	1.77726875	],
[	4.49219665492958	,	1.47259410714286	],
[	4.79504137323944	,	1.06636125	],
[	4.94646373239437	,	0.253895535714286	],
[	4.99693785211268	,	0.101558214285714	],
[	5.29978257042254	,	-0.457011964285714	],
[	5.85499788732394	,	-0.761686607142857	],
[	5.70357552816901	,	-0.101558214285714	],
[	6.05689436619718	,	0.558570178571428	],
[	6.51116144366197	,	0.457011964285714	],
[	6.71305792253521	,	1.16791946428571	],
[	6.96542852112676	,	2.64051357142857	],
[	7.01590264084507	,	3.96077035714286	],
[	7.72254031690141	,	4.265445	],
[	7.72254031690141	,	4.87479428571429	],
[	8.42917799295775	,	5.89037642857143	],
[	8.73202271126761	,	6.19505107142857	],
[	9.33771214788732	,	7.31219142857143	],
[	10.0443498239437	,	7.56608696428571	],
[	10.5995651408451	,	7.97231982142857	],
[	11.6090475352113	,	7.46452875	],
[	12.3661593309859	,	6.85517946428571	],
[	12.4671075704225	,	6.44894660714286	],
[	12.9718487676056	,	5.83959732142857	],
[	13.3251676056338	,	5.48414357142857	],
[	14.2841758802817	,	5.33180625	],
[	14.5365464788732	,	5.43336446428571	],
[	15.5965029929577	,	5.43336446428571	],
[	15.8488735915493	,	5.07791071428571	],
[	16.8583559859155	,	5.43336446428571	],
[	18.1202089788732	,	5.23024803571429	],
[	19.2811137323944	,	5.53492267857143	],
[	19.6849066901408	,	4.92557339285714	],
[	20.0886996478873	,	4.41778232142857	],
[	20.694389084507	,	4.46856142857143	],
[	21.1486561619718	,	4.46856142857143	],
[	21.8048197183099	,	4.51934053571429	],
[	21.8048197183099	,	3.85921214285714	],
[	22.2086126760563	,	3.24986285714286	],
[	22.3600350352113	,	2.69129267857143	],
[	22.6124056338028	,	2.23428071428571	],
[	23.4199915492958	,	2.08194339285714	],
[	23.8237845070423	,	1.52337321428571	],
[	24.5808963028169	,	1.37103589285714	],
[	24.7827927816901	,	0.812465714285714	],
[	25.0856375	,	0.101558214285714	],
[	25.0351633802817	,	-0.660128392857143	],
[	24.7827927816901	,	-1.11714035714286	],
[	24.7323186619718	,	-1.52337321428571	],
[	24.8332669014084	,	-2.1327225	],
[	25.2875339788732	,	-1.92960607142857	],
[	25.4894304577465	,	-1.47259410714286	],
[	26.0951198943662	,	-1.32025678571429	],
[	26.1455940140845	,	-0.914023928571428	],
[	26.2970163732394	,	-0.457011964285714	]]

path_metros=[
[	0.858060035211268	,	-0.253895535714286	],
[	3.0789213028169	,	0	],
[	7.06637676056338	,	-1.421815	],
[	4.89598961267606	,	-4.67167785714286	],
[	6.20831672535211	,	-5.94115553571429	],
[	6.20831672535211	,	-5.94115553571429	]]



V=0
V_max = 0.22; W_max = 2.84; #Timer, initial time,W=0
T = 100.0

def publish_markers(self):
        # Publish visual data in Rviz
        markerArray = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.type = marker.CYLINDER
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.01
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = self.punto_desado_xh
        marker.pose.position.y = self.punto_desado_yh
        marker.pose.position.z = 0
        
        markerArray.markers.append(marker)
        self.publisher.publish(markerArray)



def Fatrac(self):
    xd=0; yd=1
    tetha=self.yaw
    h=0.1
    d=sqrt((self.x-xd)*(self.x-xd)+(self.y-yd)*(self.y-yd))
    Qd=5
    Katrac=0.2

    
    if (d<Qd):
        Fx_atrac=-Katrac*(self.x-xd)
        Fy_atrac=-Katrac*(self.y-yd)
    else:
        Fx_atrac=-Katrac*(self.x-xd)*Qd/d
        Fy_atrac=-Katrac*(self.y-yd)*Qd/d
    

    Ux=Fx_atrac
    Uy=Fy_atrac
    self.V= Ux*cos(tetha)+Uy*sin(tetha)
    self.W=-Ux*sin(tetha)/h+Uy*cos(tetha)/h
    #Evitar la saturacion en las velocidades 
    if (abs(self.V)>V_max):
        self.V = V_max*abs(self.V)/self.V
        print("Saturacion en V\t")
    if (abs(self.W)>W_max):
        self.W = W_max*abs(self.W)/self.W
        print("Saturacion en W\t")


def control_velocidad(self):
    k=1; 
    self.h=0.1    
    tetha=self.yaw

    self.xh = self.x+self.h*cos(tetha)
    self.yh = self.y+self.h*sin(tetha)

    ex = self.xh-self.punto_desado_xh;  ey = self.yh-self.punto_desado_yh
    Ux = -k*ex;  Uy =-k*ey
    self.Velocidad_Lineal= Ux*cos(tetha)+Uy*sin(tetha)
    self.Velocidad_Angular=-Ux*sin(tetha)/self.h+Uy*cos(tetha)/self.h
    
    #Evitar la saturacion en las velocidades 
    if (abs(self.Velocidad_Lineal)>V_max):
        self.Velocidad_Lineal = V_max*abs(self.Velocidad_Lineal)/self.Velocidad_Lineal
    if (abs(self.Velocidad_Angular)>W_max):
        self.Velocidad_Angular = W_max*abs(self.Velocidad_Angular)/self.Velocidad_Angular



class Nodo(object):
    def __init__(self):
        self.loop_rate = rospy.Rate(10)
        #inicializar
        self.x = None
        self.punto_desado_xh=0
        self.punto_desado_yh=0
        self.Velocidad_Lineal=0
        self.Velocidad_Angular=0
        # Subscribirse
        odom_sub =rospy.Subscriber('/odom',Odometry, self.Callback)
        self.ph_pub =rospy.Publisher('/punto_h',Odometry, queue_size=10)
        self.odom_msg = Odometry()
        #Publicar
        #self.pub = rospy.Publisher('/cmd_vel_mux/input/navi',Twist,queue_size=10)
        self.pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
        self.publisher = rospy.Publisher("goal_point", MarkerArray, queue_size=3)
        #Inicializar Tiempo
        self.t0 = rospy.Time.now().to_sec()


    def Callback(self,msg):
        #Posicion
        self.x=msg.pose.pose.position.x
        self.y=msg.pose.pose.position.y
        print("X={0} Y={1}".format(self.x,self.y))
        print("Xhd={0} Yhd={1}".format(self.punto_desado_xh,self.punto_desado_yh))
        #Inicializa velocidad
        self.velocidad=Twist()
        #Orientacion
        quater = msg.pose.pose.orientation
        quater_list = [quater.x, quater.y, quater.z, quater.w]
        (roll, pitch, self.yaw) = euler_from_quaternion(quater_list)
        #Tiempo
        self.time = rospy.Time.now().to_sec()-self.t0
        #Funcion de control
        control_velocidad(self)
        publish_markers(self)
        #Fatrac(self)
        

    def start(self):
        rospy.loginfo("Comenzando codigo")
        bandera=0
        while not rospy.is_shutdown():   
            if self.x is not None:    
                self.velocidad.linear.x = self.Velocidad_Lineal 
                self.velocidad.angular.z = self.Velocidad_Angular
                self.pub.publish(self.velocidad)
                #Publicación de ph en RVIZ
                self.odom_msg.pose.pose.position.x = self.xh
                self.odom_msg.pose.pose.position.y = self.yh
                self.ph_pub.publish(self.odom_msg)
                distancia_control = sqrt((self.x - self.punto_desado_xh)**2 + (self.y - self.punto_desado_yh)**2)
                
                if distancia_control < 0.13:
                    #sp,j=metros_a_pixeles(self,-1,2)
                    #rp,t=pixeles_a_metros(self,sp,j)

                    if bandera == (len(path_metros)):
                        self.Velocidad_Lineal=0
                        self.Velocidad_Angular=0
                        self.velocidad.linear.x = self.Velocidad_Lineal 
                        self.velocidad.angular.z = self.Velocidad_Angular
                        self.pub.publish(self.velocidad)
                        print("pausaaaaaaaaaaa")
                        #time.sleep(5)

                    if bandera < len(path_metros):
                        self.punto_desado_xh= path_metros[bandera][0]-0.6
                        self.punto_desado_yh = path_metros[bandera][1]#+0.2
                        
                        bandera=bandera+1
                        print("posicion xd")
                        print(self.punto_desado_xh, self.punto_desado_yh)
                    
                    
                control_velocidad(self)




            self.loop_rate.sleep()


if __name__ == '__main__':
    rospy.init_node("MOVIMIENTO", anonymous=True)


    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    try:
        my_node = Nodo()
        my_node.start()
    except rospy.ROSInterruptException:
        pass
    
    
