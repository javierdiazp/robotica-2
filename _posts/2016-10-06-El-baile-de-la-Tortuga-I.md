---
layout: post
title:  "El baile de la Tortuga I"
date:   2016-10-06 15:00:00
author: Javier Díaz, Patricio Merino
categories: ROS
---

# El baile de la Tortuga I

Queremos diseñar e implementar un baile para un robot controlado con ROS. Utilizaremos el lenguaje Python para aprovechar la API de ROS [rospy](http://wiki,ros.org/rospy). Un baile se considera como un loop infinito de pasos.

## Estudiando el movimiento del robot
Lo primero que hicimos fué mover el robot con el teclado (usando teleop) y utilizando las herramientas de ROS observamos los tipos de mensajes que eran enviados a los tópicos de control y de estado del robot.

1. Para encontrar el tópico asociado a teleop utilizamos `rostopic list`, que entrega una lista de tópicos activos. El nombre del tópico candidato es `/cmd_vel_mux/input/teleop`

* Para comprobar que este recibe los comandos de movimiento desde el teclado, usamos `rostopic echo`, el cual muestra la información leída por el tópico. Entonces ejecutamos acciones sobre el teclado y observamos que el tópico efectivamente recibía información de estas.

* Luego utilizamos `rostopic info` para averiguar qué tipo de mensaje recibe el tópico hallado, una estructura llamada `geometry_msgs/Twist`.

* Entonces, usando `rosmsg show` determinamos los parámetros asociados a dicho tipo de mensaje, los cuales son dos arreglos de `float64`. 

* Con la información entregada por `rosmsg show` y `rostopic echo` logramos inferir que el primer arreglo se refiere a la velocidad lineal en los ejes X, Y, Z; y el segundo se refiere a la velocidad angular en torno a estos ejes.

Lo siguiente era averiguar una forma de conocer información sobre el estado del robot.

1. Para esto, usando `rostopic list`, encontramos un tópico con el nombre `/joint_states` que pensamos que tenía relación con la información del estado del robot.

* Luego, a partir de `rostopic echo` nos dimos cuenta que nuestra hipótesis era correcta.

* Después, utilizando `rostopic info` averiguamos que el tipo de mensaje es una estructura llamada `sensor_msgs/JointState`

* Entonces, con `rosmsg show` vimos que aquella estructura contenía, entre otras cosas, la distancia recorrida y velocidad de cada rueda.

* A partir de esta información podemos determinar la distancia recorrida por el robot, la cantidad que ha girado y si se está moviendo.

## Diseño del baile
Dadas estas restricciones, el baile diseñado corresponde a formar un triángulo equilátero en cada loop. Para esto, requerimos que el robot avance una distancia igual para formar cada lado y que en los vértices gire 120°. Esto lo conseguimos moviendo al robot lentamente, tanto al avanzar como al girar, para que no se resbale y reporte movimientos erróneos.

Para determinar la distancia que corresponde al ángulo deseado, se realizaron pruebas con diferentes valores hasta lograr que se formara el triángulo.

## El algoritmo
### Comunicación con el robot
Para este cometido se construyó un script (ver código fuente [aquí](https://github.com/javierdiazp/baile/blob/master/src/baile.py) ) que posee un listener suscrito a un tópico llamado Joint States (`/joint_states`) que se encarga de entregar feedback acerca del estado del robot en cuanto a la distancia recorrida y velocidad de cada una de sus ruedas.

Además, de manera global, el script publica en el tópico Teleop (`/cmd_vel_mux/input/teleop`), el cual se encarga de entregar la información de movimiento al robot. Dicha información es una estructura que contiene una velocidad lineal y una angular.

Entonces, el listener se encarga de ejecutar un callback cada vez que recibe información de estado del robot. Dicha función ejecuta la parte central del código que controla el comportamiento de las ruedas definiendo las velocidades lineal y angular, según cada caso.

### Estados de movimiento
Se utiliza una variable de estado para definir en qué parte del triángulo está el robot, lo que se verifica en cada llamada del callback. Existen tres posibles configuraciones:

* `INIT_LOOP` indica que el robot está en la posición inicial del ciclo  (esto es, detenido en uno de los vértices, mirando hacia su siguiente objetivo). Inicializa el estado `MOV_FORWARD`.

* `MOV_FORWARD` indica que el robot tiene que moverse hacia adelante hasta haber recorrido un distancia previamente definida y detenerse. Esto se consigue comprobando lo siguiente: 

	* Si la distancia no ha sido alcanzada, se impone una velocidad lineal en el robot para lograr el objetivo.
	* Si la distancia ya ha sido alcanzada y el robot se está moviendo, se impone una velocidad nula, para indicar que este debe detenerse.
	* Si la distancia ya ha sido alcanzada y el robot está detenido, se cambia el estado a `TURNING`.

* `TURNING` indica que el robot debe girar los 120° que permiten formar el triángulo equilátero. Hacer que el robot gire se traduce en mover sus ruedas una distancia dada, en dirección contraria. Entonces se comprueba lo siguiente:
	
	* Si la distancia no ha sido alcanzada, se impone una velocidad angular en el robot para lograr el objetivo.
	* Si la distancia ya ha sido alcanzada y el robot se está moviendo, se impone una velocidad nula, para indicar que este debe detenerse.
	* Si la distancia ya ha sido alcanzada y el robot está detenido, se cambia el estado a `INIT_LOOP`.

## Conclusión
El siguiente video muestra los resultados del código anterior en ejecución.

[![Baile I]({{site.baseurl}}/assets/baile1.png)](http://www.youtube.com/watch?v=j6XdlPVipDs)

Finalmente concluimos que, al menos para realizar una tarea tan básica como esta, ROS se comporta de manera adecuada y tiene las herramientas suficientes para determinar las funcionalidades de los nodos y tópicos de un robot, y comenzar a controlarlo con un script. Sin embargo, es discutible la usabilidad de las herramientas de ROS para realizar estas investigaciones, dado que hay que realizar suposiciones sobre los tópicos, o bien revisar cada uno de ellos para determinar donde se encuentran y cuáles permiten definir el comportamiento del robot.