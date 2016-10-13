---
layout: post
title:  "Problemas con ROS"
date:   2016-10-03 17:00:00
author: Javier Díaz, Patricio Merino
categories: ROS
---

# Problemas con ROS al cambiar la versión de Turtlebot

Cuando intentamos cambiar la base del turtlebot V2 (Kobuki) a V1 (Create) surgió un problema en el reconocimiento de los comandos de teleop que impedía mover el robot en Gazebo. Dado esto, describiremos la metodología que utilizamos para intentar diagnosticar el problema.

## Herramientas utilizadas
ROS nos otorga varias herramientas desde la terminal. Algunas que nos fueron útiles son:
    * rosnode list: Permite conocer cuales son los nodos activos, a partir del cual identificamos los nodos relacionados con teleop.
    * rosnode info: Muestra los tópicos relacionados al nodo.
    * rostopic info: Muestra el tipo de mensaje asociado al tópico y los nodos publicadores y subscritos. A partir de esta información (y junto a rosnode info) podemos saber el flujo de los mensajes.
    * rosmsg show: Muestra el tipo de información que lleva cada mensaje, lo que permite identificar si el mensaje comunica información relacionada al problema.
    * rostopic echo: Muestra los mensajes que le llegan al tópico, los que nos permite asegurar si los mensajes están llegando correctamente.

## Diagnóstico del problema
A partir de estas herramientas logramos detectar que posiblemente la falla radica en que el mensaje relacionado con el movimiento, que procede de teleop, no está llegando finalmente a Gazebo.

El procedimiento que usamos para realizar esta hipótesis fue hacer un seguimiento de los mensajes publicados por teleop que llevan la información sobre el movimiento del robot, a través de varios nodos intermedios, para determinar si llegan a Gazebo.
En la práctica no parecía existir un flujo que llevara el mensaje a Gazebo, con lo que éste no podía conocer la posición del robot.

La falla radica en la falta de un nodo intermediario que traduzca la información de teleop y la transporte  a Gazebo, con lo cuál la solución más evidente es añadir dicho nodo al flujo del sistema.

## Conclusión
Debido a la aparición de esta situación observamos ciertos defectos en la herramienta ROS a la hora de diagnosticar problemas en el flujo de información. Lo más relevante es hacer más amigable el proceso a través de una herramienta visual de ROS con buenos filtros para identificar correctamente la interacción entre nodos.