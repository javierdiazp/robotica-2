---
layout: post
title:  "El baile de la Tortuga I"
date:   2016-10-06 16:15:00
author: Javier Díaz, Patricio Merino
categories: ROS
---

# El baile de la Tortuga II

Queremos diseñar e implementar un para un robot controlado con ROS. Utilizaremos el lenguaje Python para usar la API de ROS [rospy](http://wiki,ros.org/rospy) con  [actionlib](http://wiki.ros.org/actionlib), que define una interfaz cliente-servidor . Un baile se considera como un loop infinito de pasos.

Para el diseño del baile utilizamos la misma configuración expuesta en el post anterior El baile de la Tortuga I.

El algoritmo es una adaptación del código utilizado anteriormente, para funcionar en el modelo cliente-servidor provisto por actionlib.

Asignamos el rol de Publisher al Servidor, y de Subscriber al Cliente. Entonces el cliente recibe la información del estado del robot, y le propone al servidor una meta que consiste en el tipo de movimiento que este tiene que realizar (avanzar recto o girar). Luego, de acuerdo al tipo de movimiento recibido, el servidor mueve al robot. Esto implica que el servidor no utiliza el feedback sobre el cliente para retornar ninguna información relevante ya que esta se obtiene directamente del robot.

[Video](https://youtu.be/YWrdbegYXXA) 