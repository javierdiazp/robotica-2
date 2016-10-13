---
layout: post
title:  "El baile de la Tortuga II"
date:   2016-10-11 16:00:00
author: Javier Díaz, Patricio Merino
categories: ROS
---

# El baile de la Tortuga II

Queremos diseñar e implementar un baile para un robot controlado con ROS. Utilizaremos el lenguaje Python para usar la API de ROS [rospy](http://wiki,ros.org/rospy) con  [actionlib](http://wiki.ros.org/actionlib), que define una interfaz cliente-servidor . Un baile se considera como un loop infinito de pasos.

Para el diseño del baile utilizamos la misma configuración expuesta en el post anterior, [El baile de la Tortuga I](https://javierdiazp.github.io/robotica2/ros/2016/10/06/El-baile-de-la-Tortuga-I.html), un triángulo equilátero.

El algoritmo es una adaptación del código utilizado anteriormente, para funcionar en el modelo cliente-servidor provisto por actionlib. (Ver código fuente del proyecto [aquí](https://github.com/javierdiazp/baile2))

## El modelo cliente-servidor

Actionlib provee un sistema de metas que son inicializadas por el cliente y enviadas al servidor para que este realice acciones hasta alcanzarla. Una vez conseguida la meta, el servidor informa al cliente que ha terminado y le entrega un resultado.

## Adaptación del algoritmo para actionlib

### El cliente
El método utilizado para realizar el movimiento consiste en un ciclo infinito de metas alternadas que el cliente le envía al servidor. Dichas metas consisten en, o bien avanzar cierta distancia hasta detenerse, o bien girar cierto ángulo hasta detenerse, esperando a que se complete la tarea en cada paso antes de iniciar la siguiente.

### El servidor
Entonces el servidor se encarga de la tarea de comprobar el estado actual del robot para saber si ya se cumplió el objetivo. Esto significa que el servidor debe actuar como Subscriber del tópico JoinStates (`/joint_states`), el cual entrega, entre otras cosas, la distancia recorrida y velocidad de cada una de las ruedas del robot.

Además, el mismo servidor se encarga también de indicarle al robot la velocidad lineal o angular que define el movimiento en cada caso. Para esto el servidor debe actuar como Publisher sobre el tópico Teleop (`/cmd_vel_mux/input/teleop`), el cual maneja la entrada que mueve al robot.

Para adaptar el callback que se recibe siendo subscriptor a un modelo que también recibe callback desde el cliente utilizamos dos variables globales `isMoving` y `curPos` (si sigue en movimiento y su posición actual respectivamente), para que la información que nos interesa del estado del robot esté siempre disponible. La llamada del suscriptor actualiza estas variables y la llamada del cliente las lee para saber si la meta ya fue completada.

## Conclusión

El siguiente video muestra los resultados del código anterior en ejecución.

[![Baile II]({{site.baseurl}}/assets/baile1.png)](https://youtu.be/lyHz-3T-oVM)

Para este problema no hicimos un uso profundo de las características del modelo cliente-servidor dado que solo necesitábamos saber si las metas fueron cumplidas exitosamente, pero no obtener feedback o un resultado final desde el servidor. Por esto, nuestro proceso se tradujo en dividir las tareas del algoritmo anterior y repartirlas entre el cliente y el servidor, conservando la esencia de su comportamiento. En cambio, otros problemas que son adecuados para este sistema tienen características diferentes que requieren de una estructura que maneje tareas independientemente y otro proceso que esté siempre activo, pero ese no era el caso para esta ocasión.