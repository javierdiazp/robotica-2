---
layout: post
title:  "El baile de la Tortuga I"
date:   2016-10-06 15:00:00
author: Javier Díaz, Patricio Merino
categories: ROS
---

# El baile de la Tortuga I

Queremos diseñar e implementar un para un robot controlado con ROS. Utilizaremos el lenguaje Python para usar la API de ROS [rospy](http://wiki,ros.org/rospy). Un baile se considera como un loop infinito de pasos.

Lo primero que hicimos fué mover el robot con el teclado (usando teleop) y utilizando las herramientas de ROS observamos los tipos de mensajes que eran enviados a los tópicos de control y de estado del robot.

Estudiando los mensajes de control (enviados a teleop), pudimos determinar que los argumentos entregados son la velocidad angular y velocidad lineal. Similarmente, la información de estado tiene, entre otros, la distancia recorrida y velocidad de cada rueda.

Dadas estas restricciones, el baile diseñado corresponde a formar un triángulo equilátero en cada loop. Para esto, requerimos que el robot avance una distancia igual para formar cada lado y que en los vértices gire 120°. Esto lo conseguimos moviendo al robot lentamente, tanto al avanzar como al girar, para que no se resbale y reporte movimientos erróneos.

El script utilizado es el siguiente:
	
	#!/usr/bin/env python

	import rospy
	from geometry_msgs.msg import Twist, Vector3
	from sensor_msgs.msg import JointState

	# Constantes de estado
	INIT_LOOP = 0
	MOV_FORWARD = 1
	TURNING = 2

	# Estado del robot
	state = INIT_LOOP

	# Posición inicial de cada rueda [izq, der]
	iPos = [0, 0]

	# Tópico donde se publican los comando para que el robot se mueva.
	pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=10)

	# Retorna True si el robot se está moviendo (con margen de error eps). False si no.
	def isMoving(vel):
	    eps = 0.001
	    return abs(vel[0]) > eps or abs(vel[1]) > eps

	# Función que se ejecutra cada vez que llega información del robot.
	def callback(data):
	    global iPos
	    global state
	    
	    # Distancia que debe avanzar el robot antes de frenar.
	    # Forma un lado del triángulo.
	    distLin = 27
	    
	    # Distancia que avanzan las ruedas (en sentido contrario) para girar al robot.
	    # Forma un vértice del triángulo.
	    distAng = 7
	    
	    vLin = 0
	    vAng = 0

	    if state == INIT_LOOP:
		iPos = data.position
		state = MOV_FORWARD
	    
	    if state == MOV_FORWARD:
		if data.position[0] - iPos[0] < distLin or data.position[1] - iPos[1] < distLin:
		    vLin = 0.3
		elif not isMoving(data.velocity):
		    iPos = data.position 
		    state = TURNING
	    
	    if state == TURNING:
		if data.position[0] - iPos[0] < distAng or data.position[1] - iPos[1] > 0-distAng:
		    vAng = -0.3
		elif not isMoving(data.velocity):
		    state = INIT_LOOP

	    pub.publish(Vector3(vLin, 0, 0), Vector3(0, 0, vAng))

	# Función que se subscribe al tópico joint_states, que recibe información del estado del robot
	def listener():
	    rospy.init_node('baile1', anonymous=True)
	    rospy.Subscriber("/joint_states", JointState , callback)
	    rospy.spin()

	if __name__ == '__main__':
	    try:
		listener()
	    except rospy.ROSInterruptException:
		pass

A grandes rasgos el algoritmo utiliza una variable de estado para definir en qué parte del triángulo está el robot y saber si necesita moverse o girar en cada caso.


El siguiente video muestra los resultados del código anterior en ejecución. [El baile de la Tortuga I](https://youtu.be/j6XdlPVipDs) 

Finalmente concluimos que, al menos para realizar una tarea tan básica como esta, ROS se comporta de manera adecuada y tiene las herramientas suficientes para determinar las funcionalidades de los nodos y tópicos de un robot, y comenzar a controlarlo con un script. Sin embargo, es discutible la usabilidad de las herramientas de ROS para realizar estas investigaciones, dado que hay que realizar suposiciones sobre los tópicos, o bien revisar cada uno de ellos para determinar donde se encuentran y cuáles permiten definir el comportamiento del robot.