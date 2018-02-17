
#***************************************************************************
# Title        : Assignment2_template.py
#
# Description  : This file is a starting point for assignment 2 it contains
#                the main parts and pseudo code for you to complete with your 
#                own code.
#
# Environment  : Python 2.7 Code. 
#
# License      : GNU GPL version 3
#
# Editor Used  : Sublime Text
#
#****************************************************************************

#****************************************************************************
# Imported functions, classes and methods
#****************************************************************************

#****************************************************************************
#   Method Name     : key
#
#   Description     : Callback for TkInter Key events
#
#   Parameters      : Event: tkinter event containing the key that was pressed
#
#   Return Value    : None
#
#   Author           : Alan Espinoza Saldana A01631791
#
#****************************************************************************

import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
import Tkinter as tk

#A continuacion se define la variable para la velocidad que alcanzara el dorne
def set_velocity_body(vehicle, vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111, #-- BITMASK -> Consider only the velocities
            0, 0, 0,        #-- POSITION
            vx, vy, vz,     #-- VELOCITY
            0, 0, 0,        #-- ACCELERATIONS
            0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()
#BASE TOMADA DE LAS INSTRUCCIONES DE LA TAREA 2

#Aca definire las direcciones en las que quiero que el drone se dirija
def key(event):
    if event.char == event.keysym: #-- standard keys
        if event.keysym == 'r':
            drone.mode=VehicleMode("RTL")
            ### Add your code for what you want to happen when you press r #####
            
    else: #-- non standard keys
        if event.keysym == 'Up':
            set_velocity_body(drone,5,0,0)
            #dando la instruccion que el drone se dirija hacia arriba
        elif event.keysym == 'Down':
            set_velocity_body(drone,-5,0,0)
            #instruccion para que se dirija hacia abajo, poniendo el signo negativo del 5
        elif event.keysym == 'Left':
            set_velocity_body(drone,0,-5,0)
            #poniendo el signo negativo del 5, para que gire hacia la izquierda
        elif event.keysym == 'Right':
            set_velocity_body(drone,0,5,0) 
            #darle la instruccion de que gire a la derecha

drone = connect('udp:127.0.0.1:14551', wait_ready=True)
#variable copiada del video del profesor

#Aqui definimos el despegue del drone, dejando en claro que la latitud que alcance se multiplicara por .95, es decir que sera menor de lo indicado
def arm_and_takeoff(TargetAltitude):
    print ("Executing Takeoff")

    while not drone.is_armable:
        print ("Vehicle is not armable, waiting...")
        time.sleep(1)

    print ("Warning> Arming Motors")
    drone.mode = VehicleMode("GUIDED")
    drone.armed = True

    while not drone.armed:
        print ("waiting for arming...")
        time.sleep(1)

    print ("Taking off...")
    drone.simple_takeoff(TargetAltitude)

    while True:
        Altitude = drone.location.global_relative_frame.alt
        print ("Altitude: ", Altitude)
        time.sleep(1)
#aqui se le asigno a la altitud ser multiplicada por .95
        if Altitude >= TargetAltitude * .95:
            print ("Altitude Reached")
            break
#CABE MENCIONAR QUE ESTA BASE LA TOME DEL VIDEO REALIZADO DPOR EL PROFESOR

arm_and_takeoff(10)
#instruccion para que se elve en una altitud de 10

#base tomada de las instrucciones de la actividad
root = tk.Tk()
print(">> Control the drone with the arrow keys. Press r for RTL mode")
root.bind_all('<Key>', key)
root.mainloop()